// Late server job failure feedback: the actual GeminiLiveService, RobotBridge,
// matcher and receipt cache, with fake fetch/audio. No SDK, network or TTS.
import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';

const stub = (filter, contents) => ({ name: 'stub', setup(b) {
  b.onResolve({ filter }, () => ({ path: 'stub', namespace: 'stub' }));
  b.onLoad({ filter: /.*/, namespace: 'stub' }, () => ({ contents }));
}});
async function load(relative, plugins = []) {
  const result = await build({
    entryPoints: [fileURLToPath(new URL(relative, import.meta.url))],
    bundle: true, write: false, format: 'esm', platform: 'node', jsx: 'transform',
    tsconfigRaw: { compilerOptions: { jsx: 'react' } }, plugins,
  });
  return import('data:text/javascript;base64,' + Buffer.from(result.outputFiles[0].text).toString('base64'));
}
const { GeminiLiveService } = await load('../services/geminiLive.ts', [stub(/^@google\/genai$/,
  "export class GoogleGenAI { constructor() { throw new Error('no SDK in late job tests'); } } export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};")]);
const { ReceiptAudioCache } = await load('../services/receiptAudioCache.ts');
const { default: ExecutionStatus } = await load('../components/ExecutionStatus.tsx', [stub(/^react$/,
  "const h=()=>globalThis.__statusHooks; export const useState=i=>h().useState(i); export const useRef=v=>h().useRef(v); export const useEffect=(f,d)=>h().useEffect(f,d); export default {createElement:(type,props,...children)=>({type,props:{...props,children}})};")]);

const INST = 'a'.repeat(32), OTHER = 'b'.repeat(32);
const EPOCH = `${INST}:1`, EPOCH_AFTER_STOP = `${INST}:2`;
const FAIL_TEXT = 'That request failed. Check the controls for details.';

async function flush(n = 8) { for (let i = 0; i < n; i++) await new Promise(r => setImmediate(r)); }
function deferred() {
  let resolve, reject;
  const promise = new Promise((a, b) => { resolve = a; reject = b; });
  return { promise, resolve, reject };
}
function replaceGlobal(t, name, value) {
  const descriptor = Object.getOwnPropertyDescriptor(globalThis, name);
  Object.defineProperty(globalThis, name, { configurable: true, writable: true, value });
  t.after(() => descriptor ? Object.defineProperty(globalThis, name, descriptor) : delete globalThis[name]);
}

// The failure clip is 4 bytes (2 frames); Okay is 6 bytes (3 frames).
async function cacheWith(texts) {
  const files = new Map();
  const clips = texts.map((text, index) => {
    const bytes = new Uint8Array(text === FAIL_TEXT ? 4 : 6).fill(index + 1);
    const file = `clip${index}.pcm`; files.set(file, bytes);
    return { text, transcript: text, file, sha256: createHash('sha256').update(bytes).digest('hex'),
      bytes: bytes.length, duration_s: bytes.length / 48000, review_status: 'approved_human' };
  });
  return ReceiptAudioCache.load({ schema: 'pupper_receipt_audio_v1', model: 'gemini-3.1-flash-live-preview', voice: 'Puck',
    encoding: 'pcm_s16le', sample_rate_hz: 24000, channels: 1, clips }, async file => files.get(file));
}
const fullCache = await cacheWith([FAIL_TEXT, 'Okay.']);

function fakeAudio() {
  const started = [];
  let resume = null;
  const ctx = { state: 'running', currentTime: 0,
    createBuffer: (_channels, frames, rate) => ({ frames, duration: frames / rate, getChannelData: () => new Float32Array(frames) }),
    createBufferSource: () => { const source = { buffer: null, connect() {}, disconnect() {}, start() { started.push(source); }, stop() { source.stopped = true; } }; return source; },
    resume: () => new Promise(r => { resume = () => { ctx.state = 'running'; r(); }; }),
    close: async () => { ctx.state = 'closed'; } };
  return { ctx, cues: () => started.filter(source => source.buffer.frames === 2).length, release: () => resume?.() };
}

let servers = 0;
function setup(t, { policy = 'conversation', voice = 'Puck', cache = fullCache } = {}) {
  const base = `http://late-${++servers}.test`;
  const server = { epoch: EPOCH, replies: [], paths: [] };
  t.mock.method(globalThis, 'fetch', async url => {
    const path = url.slice(base.length); server.paths.push(path);
    let body;
    if (path === '/health') body = { command_epoch_required: true, command_epoch: server.epoch };
    else if (path === '/stop/immediate') {
      server.epoch = EPOCH_AFTER_STOP;
      body = { success: true, message: 'Stop requested.', command_epoch: server.epoch, command_epoch_required: true };
    } else if (path.startsWith('/tools/')) body = await server.replies.shift();
    else throw new Error(`unexpected request ${path}`);
    return { ok: true, json: async () => body };
  });
  const env = { base, server, logs: [], dispatches: [], audio: fakeAudio(), tts: 0 };
  env.make = () => {
    const s = new GeminiLiveService({ modelName: 'gemini-3.1-flash-live-preview', voiceName: voice, robotApiUrl: base, outputPolicy: policy },
      e => env.logs.push(e), () => {}, () => {}, e => env.dispatches.push(e), () => {}, () => {}, undefined, () => {}, cache ?? undefined);
    s.connected = true;
    s.sessionPromise = Promise.resolve({ sendToolResponse() {}, sendRealtimeInput() {}, close() {} });
    s.audioContext = env.audio.ctx; s.analyser = { connect() {} };
    s.synthesizeAndPlaySpeech = async () => { env.tts++; };
    return s;
  };
  env.s = env.make();
  return env;
}

const submission = n => ({ success: true, message: 'Move queued.', details: { submission: {
  schema: 'pupper_submission_v1', state: 'queued', job_id: `${INST}:${n}`, server_instance: INST } } });
const call = id => ({ id, name: 'move', args: { direction: 'forward', speed: 0.2, duration: 1 } });
async function accept(env, s, id, n) {
  env.server.replies.push(submission(n));
  await s.handleMessage({ toolCall: { functionCalls: [call(id)] } });
  await s.handleMessage({ serverContent: { turnComplete: true } });
}
const row = (n, terminal_state, execution_started = true) => ({ job_id: `${INST}:${n}`, server_instance: INST, terminal_state,
  execution_started, success: terminal_state === 'succeeded', event_seq: n, message: 'servo bus error 0x7f' });
function snapshot({ outcomes = [], epoch = EPOCH, instance = INST } = {}) {
  return { queued: 0, running: false, following: false, state: 'locomotion', controller_fault: null, last_outcome: null,
    session_context: { schema: 'pupper_session_context_v1', scope: 'this_server_session', server_instance: instance },
    command_epoch: epoch, current_action: null, recent_outcomes: { capacity: 32, dropped: 0, outcomes } };
}
function report(s, status, overrides = {}) {
  const now = Date.now();
  return { bridge: s.robotBridge, serverUrl: s.robotBridge.baseUrl, status, pollStartedAt: now, receivedAt: now,
    generation: s.robotBridge.statusGeneration(), ...overrides };
}
const failed = (...ns) => snapshot({ outcomes: ns.map(n => row(n, 'failed')) });

test('earlier accepted job failing after a later acceptance speaks one cue and pauses actions', async t => {
  const env = setup(t), { s } = env;
  await accept(env, s, 'a', 1);
  await accept(env, s, 'b', 2);
  assert.equal(env.dispatches.filter(d => d.status === 'accepted').at(-1).id, 'b', 'job 2 owns the latest UI receipt');
  assert.equal(s.hasTrackedJobs(), true, 'tracked work selects the cheap status snapshot');
  s.observeQueueStatus(report(s, failed(1)));
  await flush();
  assert.equal(env.audio.cues(), 1, 'cue survives its own pause');
  assert.equal(s.robotToolsPaused, true);
  assert.ok(env.logs.some(e => e.type === 'error' && e.message.includes(`job ${INST}:1`)));
  assert.equal(env.tts, 0);
  assert.equal(s.hasTrackedJobs(), false, 'normal controller observation resumes');
  for (let i = 0; i < 3; i++) s.observeQueueStatus(report(s, failed(1, 2)));
  await flush();
  assert.equal(env.audio.cues(), 1, 'repeated snapshots never repeat the cue');
  const requests = env.server.paths.length;
  await s.handleMessage({ toolCall: { functionCalls: [call('c')] } });
  assert.equal(env.server.paths.length, requests, 'paused model motion sends no HTTP');
});

test('same-turn failures and dependent cancellations coalesce into one cue', async t => {
  const env = setup(t), { s } = env;
  env.server.replies.push(submission(1), submission(2), submission(3));
  await s.handleMessage({ toolCall: { functionCalls: [call('a'), call('b'), call('c')] } });
  s.observeQueueStatus(report(s, snapshot({ outcomes: [row(1, 'failed'), row(2, 'failed'), row(3, 'cancelled', false)] })));
  await flush();
  assert.equal(env.audio.cues(), 1);
  assert.equal(env.logs.filter(e => e.type === 'error' && e.message.includes('later failed on the server')).length, 2);
});

test('terminal success and cancellation stay silent and keep actions available', async t => {
  const env = setup(t), { s } = env;
  await accept(env, s, 'a', 1);
  await accept(env, s, 'b', 2);
  s.observeQueueStatus(report(s, snapshot({ outcomes: [row(1, 'succeeded'), row(2, 'cancelled', false)] })));
  await flush();
  assert.equal(env.audio.cues(), 0);
  assert.equal(s.robotToolsPaused, false);
  assert.equal(s.hasTrackedJobs(), false);
  assert.ok(!env.logs.some(e => e.type === 'error'));
});

test('stale, malformed, cross-URL, restarted, re-epoched or missing evidence never speaks', async t => {
  const cases = {
    stale: s => report(s, failed(1), { receivedAt: Date.now() - 10_000 }),
    malformed: s => report(s, snapshot({ outcomes: [{ ...row(1, 'failed'), event_seq: 0 }] })),
    crossUrl: s => report(s, failed(1), { serverUrl: 'http://elsewhere.test' }),
    otherBridge: s => report(s, failed(1), { bridge: {} }),
    oldGeneration: s => report(s, failed(1), { generation: s.robotBridge.statusGeneration() - 1 }),
    epochMismatch: s => report(s, snapshot({ outcomes: [row(1, 'failed')], epoch: `${INST}:7` })),
    restarted: s => report(s, snapshot({ instance: OTHER })),
    missingJob: s => report(s, failed(9)),
    oldPollBeforeAcceptance: (s, startedEarly) => report(s, failed(1), { pollStartedAt: startedEarly }),
  };
  for (const [name, evidence] of Object.entries(cases)) await t.test(name, async t2 => {
    const env = setup(t2), { s } = env;
    const startedEarly = Date.now() - 1;
    await accept(env, s, 'a', 1);
    s.observeQueueStatus(evidence(s, startedEarly));
    await flush();
    assert.equal(env.audio.cues(), 0);
    assert.equal(s.robotToolsPaused, false);
    if (name === 'restarted' || name === 'epochMismatch') assert.equal(s.hasTrackedJobs(), false);
    if (name === 'oldPollBeforeAcceptance') {
      s.observeQueueStatus(report(s, failed(1))); await flush();
      assert.equal(env.audio.cues(), 1, 'a poll started after acceptance may report it');
    }
  });
});

test('Stop clears tracking; a deferred cue decode is cancelled by a later Stop', async t => {
  await t.test('Stop before evidence', async t2 => {
    const env = setup(t2), { s } = env;
    await accept(env, s, 'a', 1);
    const oldGeneration = s.robotBridge.statusGeneration();
    s.pauseRobotTools(); await s.robotBridge.immediateStop();
    assert.equal(s.hasTrackedJobs(), false);
    s.observeQueueStatus(report(s, snapshot({ outcomes: [row(1, 'failed')], epoch: EPOCH_AFTER_STOP })));
    s.observeQueueStatus(report(s, failed(1), { generation: oldGeneration }));
    await flush();
    assert.equal(env.audio.cues(), 0);
  });
  await t.test('deferred decode without Stop still plays', async t2 => {
    const env = setup(t2), { s } = env;
    await accept(env, s, 'a', 1);
    env.audio.ctx.state = 'suspended';
    s.observeQueueStatus(report(s, failed(1)));
    await flush(); env.audio.release(); await flush();
    assert.equal(env.audio.cues(), 1);
  });
  await t.test('deferred decode then Stop', async t2 => {
    const env = setup(t2), { s } = env;
    await accept(env, s, 'a', 1);
    env.audio.ctx.state = 'suspended';
    s.observeQueueStatus(report(s, failed(1)));
    await flush();
    s.pauseRobotTools(); const stop = s.robotBridge.immediateStop();
    env.audio.release(); await stop; await flush();
    assert.equal(env.audio.cues(), 0);
  });
});

test('disconnect, reconnect and old poll reports cannot speak', async t => {
  const env = setup(t), { s } = env;
  await accept(env, s, 'a', 1);
  env.audio.ctx.state = 'suspended';
  const stale = report(s, failed(1));
  await s.disconnect();
  s.observeQueueStatus(stale);
  const reconnected = env.make();
  reconnected.observeQueueStatus(stale);
  reconnected.observeQueueStatus(report(reconnected, failed(1)));
  env.audio.release(); await flush();
  assert.equal(env.audio.cues(), 0);
  assert.equal(reconnected.robotToolsPaused, false);
});

test('disconnect while the cue decode is pending suppresses it', async t => {
  const env = setup(t), { s } = env;
  await accept(env, s, 'a', 1);
  env.audio.ctx.state = 'suspended';
  s.observeQueueStatus(report(s, failed(1)));
  await flush();
  await s.disconnect();
  env.audio.release(); await flush();
  assert.equal(env.audio.cues(), 0);
});

test('stale HTTP acceptance after pause or Stop never creates tracking', async t => {
  for (const withStop of [false, true]) await t.test(withStop ? 'Stop' : 'pause', async t2 => {
    const env = setup(t2), { s } = env;
    const reply = deferred(); env.server.replies.push(reply.promise);
    const pending = s.handleMessage({ toolCall: { functionCalls: [call('a')] } });
    await flush();
    s.pauseRobotTools();
    const stop = withStop ? s.robotBridge.immediateStop() : null;
    reply.resolve(submission(1)); await pending; await stop;
    assert.equal(s.hasTrackedJobs(), false);
    s.observeQueueStatus(report(s, failed(1))); await flush();
    assert.equal(env.audio.cues(), 0);
  });
});

test('missing cache, unsupported voice or receipt-only mode logs without speech or TTS', async t => {
  const partial = await cacheWith(['Okay.']);
  for (const [name, options] of Object.entries({ noCache: { cache: null }, noClip: { cache: partial },
      unsupportedVoice: { voice: 'Kore' }, receiptsOnly: { policy: 'receipts' } })) await t.test(name, async t2 => {
    const env = setup(t2, options), { s } = env;
    await accept(env, s, 'a', 1);
    s.observeQueueStatus(report(s, failed(1))); await flush();
    assert.equal(env.audio.cues(), 0);
    assert.equal(env.tts, 0);
    assert.equal(s.robotToolsPaused, true);
    assert.ok(env.logs.some(e => e.type === 'error' && e.message.includes(`job ${INST}:1`)));
    if (name !== 'receiptsOnly') assert.ok(env.logs.some(e => /Failure cue audio unavailable/.test(e.message)));
  });
});

test('interruption keeps accepted jobs; unrelated reports leave conversation and tools working', async t => {
  const env = setup(t), { s } = env;
  s.observeQueueStatus(report(s, failed(5)));
  assert.equal(s.hasTrackedJobs(), false);
  assert.equal(s.robotToolsPaused, false);
  await s.handleMessage({ serverContent: { modelTurn: { parts: [{ text: 'Hello there.' }] } } });
  await s.handleMessage({ serverContent: { turnComplete: true } });
  await accept(env, s, 'a', 1);
  await s.handleMessage({ serverContent: { interrupted: true } });
  assert.equal(s.hasTrackedJobs(), true, 'interruption cancels audio, not accepted robot jobs');
  s.observeQueueStatus(report(s, failed(1))); await flush();
  assert.equal(env.audio.cues(), 1);
});

test('ExecutionStatus reports each fresh poll and reads observation mode per poll without restarting', async t => {
  t.mock.timers.enable({ apis: ['setTimeout', 'setInterval'] });
  const cells = [], effects = []; let cursor = 0;
  replaceGlobal(t, '__statusHooks', {
    useState: init => { const i = cursor++; if (!(i in cells)) cells[i] = typeof init === 'function' ? init() : init;
      return [cells[i], v => { cells[i] = typeof v === 'function' ? v(cells[i]) : v; }]; },
    useRef: v => { const i = cursor++; if (!(i in cells)) cells[i] = { current: v }; return cells[i]; },
    useEffect: (fn, deps) => { const i = cursor++; if (!cells[i] || deps.some((d, k) => d !== cells[i][k])) { cells[i] = deps; effects.push(fn); } },
  });
  const valid = { queued: 0, running: false, following: false, state: 'locomotion', controller_fault: null, last_outcome: null };
  const modes = [], first = [], second = [];
  let reply = valid;
  const bridge = { baseUrl: 'http://status.test', statusGeneration: () => 4, getQueueStatus: async observe => { modes.push(observe); return reply; } };
  const props = (reports, observe) => ({ robotBridge: bridge, lastDispatch: null, modelActionsPaused: false, onStop: async () => {},
    stopMessage: '', onStatusReport: r => reports.push(r), observeControllers: () => observe });
  const render = p => { cursor = 0; return ExecutionStatus(p); };
  render(props(first, true));
  const cleanup = effects.shift()(); t.after(() => cleanup?.());
  await flush();
  assert.deepEqual(modes, [true]);
  assert.equal(first.length, 1);
  assert.equal(first[0].bridge, bridge);
  assert.equal(first[0].serverUrl, 'http://status.test');
  assert.equal(first[0].generation, 4);
  assert.ok(first[0].pollStartedAt <= first[0].receivedAt);
  assert.deepEqual(first[0].status, valid);
  render(props(second, false));
  assert.equal(effects.length, 0, 'new callbacks do not restart the poll');
  t.mock.timers.tick(1000); await flush();
  assert.deepEqual(modes, [true, false]);
  assert.equal(first.length, 1); assert.equal(second.length, 1);
  reply = { success: false, message: 'offline' };
  t.mock.timers.tick(1000); await flush();
  assert.equal(second.length, 1, 'invalid status is never reported');
});


test('ordinary speech interruption while HTTP is pending retains accepted job tracking', async t => {
  const env = setup(t), { s } = env, reply = deferred();
  env.server.replies.push(reply.promise);
  const pending = s.handleMessage({ toolCall: { functionCalls: [call('a')] } });
  await flush();
  await s.handleMessage({ serverContent: { interrupted: true } });
  reply.resolve(submission(1)); await pending;
  assert.equal(s.hasTrackedJobs(), true, 'accepted work survives obsolete acceptance audio/UI');
  assert.equal(env.dispatches.some(d => d.status === 'accepted'), false, 'old display receipt is intentionally suppressed');
  s.observeQueueStatus(report(s, failed(1))); await flush();
  assert.equal(env.audio.cues(), 1);
});

test('late failure fences a different action already in flight with the existing shared Stop', async t => {
  const env = setup(t), { s } = env;
  await accept(env, s, 'a', 1);
  const reply = deferred(); env.server.replies.push(reply.promise);
  const pending = s.handleMessage({ toolCall: { functionCalls: [call('b')] } });
  await flush();
  s.observeQueueStatus(report(s, failed(1))); await flush();
  assert.equal(env.server.paths.filter(p => p === '/stop/immediate').length, 1);
  reply.resolve(submission(2)); await pending; await flush();
  assert.equal(s.hasTrackedJobs(), false);
  assert.equal(s.robotToolsPaused, true);
  s.observeQueueStatus(report(s, failed(2))); await flush();
  assert.ok(env.audio.cues() <= 1, 'Stop may interrupt the one cue but never duplicates it');
  assert.equal(env.server.paths.filter(p => p === '/stop/immediate').length, 1);
});
