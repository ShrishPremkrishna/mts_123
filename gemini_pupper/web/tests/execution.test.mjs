// Record individual responses from singleton or batched SDK envelopes.
// provider-response-batch.test.mjs separately checks actual send counts/boundaries.
import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { fileURLToPath } from 'node:url';
import { execFileSync } from 'node:child_process';

async function load(relative, stubSdk = false) {
  const result = await build({
    entryPoints: [fileURLToPath(new URL(relative, import.meta.url))],
    bundle: true, write: false, format: 'esm', platform: 'node',
    plugins: stubSdk ? [{ name: 'no-network-sdk', setup(builder) {
      builder.onResolve({ filter: /^@google\/genai$/ }, () => ({ path: 'sdk', namespace: 'test' }));
      builder.onLoad({ filter: /.*/, namespace: 'test' }, () => ({ contents:
        "export class GoogleGenAI { constructor(options) { return globalThis.__fakeLiveSdkFactory(options); } } export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};" }));
    }}] : [],
  });
  return import('data:text/javascript;base64,' + Buffer.from(result.outputFiles[0].text).toString('base64'));
}
const { parseQueueStatus, queueSummary, toolResponseStatus, requestIndependentStop, updateLatestDispatch, runtimeSummary } = await load('../services/executionStatus.ts');
const { default: RobotBridge } = await load('../services/robotBridge.ts');
const { GeminiLiveService } = await load('../services/geminiLive.ts', true);

const valid = { queued: 0, running: false, following: false, state: 'locomotion', controller_fault: null, last_outcome: null };

test('only valid server evidence becomes a queue report', () => {
  assert.deepEqual(parseQueueStatus(valid), valid);
  for (const bad of [null, {}, { success: false, message: 'offline' }, { ...valid, queued: -1 },
      { ...valid, queued: 0.5 }, { ...valid, running: 'false' },
      { ...valid, last_outcome: { name: 'move', success: 'true', message: 'done' } }]) {
    assert.equal(parseQueueStatus(bad), null);
  }
  const text = queueSummary(valid);
  assert.match(text, /no queued action running/);
  assert.doesNotMatch(text, /stopped|relaxed|standing still|motors off/);
});

test('acceptance cannot become completion and network loss is unknown', () => {
  assert.equal(toolResponseStatus({ success: true, message: 'move queued' }), 'accepted');
  assert.equal(toolResponseStatus({ success: false, message: 'invalid speed' }), 'rejected');
  assert.equal(toolResponseStatus({ success: false, outcome_unknown: true }), 'unknown');
  assert.equal(toolResponseStatus({ message: 'done' }), 'unknown');
});

test('direct Stop uses immediate endpoint without a Gemini session', async (t) => {
  let seen;
  t.mock.method(globalThis, 'fetch', async (url, options) => {
    seen = { url, options };
    return { ok: true, json: async () => ({ success: true, message: 'zero speed requested' }) };
  });
  const result = await new RobotBridge('http://test-server/').immediateStop();
  assert.equal(seen.url, 'http://test-server/stop/immediate');
  assert.equal(seen.options.method, 'POST');
  assert.equal(result.success, true);
});

test('unreachable stop does not claim rejection or physical stopping', async (t) => {
  t.mock.method(console, 'error', () => {});
  t.mock.method(globalThis, 'fetch', async () => { throw new Error('link lost'); });
  const bridge = new RobotBridge('http://unreachable-stop-server');
  const result = await bridge.immediateStop();
  assert.equal(result.success, false);
  assert.equal(result.outcome_unknown, true);
  assert.equal(await bridge.getSystemStatus(), null);
});

test('status request has a bounded timeout', async (t) => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  let signal;
  t.mock.method(globalThis, 'fetch', (_url, options) => new Promise((_resolve, reject) => {
    signal = options.signal;
    signal.addEventListener('abort', () => reject(new Error('timed out')));
  }));
  const pending = new RobotBridge('http://test-server').getQueueStatus();
  assert.equal(signal.aborted, false);
  t.mock.timers.tick(5000);
  const result = await pending;
  assert.equal(signal.aborted, true);
  assert.equal(result.success, false);
  assert.equal(parseQueueStatus(result), null);
});

function service(outputPolicy = 'receipts') {
  const dispatches = [], logs = [], responses = [];
  const config = { modelName: 'gemini-3.1-flash-live-preview', robotApiUrl: 'http://test-server', outputPolicy };
  const s = new GeminiLiveService(config, e => logs.push(e), () => {}, () => {}, e => dispatches.push(e));
  s.connected = true;
  s.sessionPromise = Promise.resolve({ sendToolResponse: r => responses.push(...[r.functionResponses].flat().map(functionResponses => ({functionResponses}))) });
  return { s, dispatches, logs, responses };
}

test('unsupported model narration creates no execution evidence', async () => {
  const { s, dispatches } = service();
  let calls = 0;
  s.robotBridge.callTool = async () => { calls++; };
  await s.handleMessage({ serverContent: { modelTurn: { parts: [{ text: 'I requested a stop.' }] } } });
  assert.equal(calls, 0);
  assert.deepEqual(dispatches, []);
});

test('actual dispatch and server acceptance are separate from later queue outcome', async () => {
  const { s, dispatches } = service();
  s.robotBridge.callTool = async () => ({ success: true, message: 'move queued' });
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'a', name: 'move', args: {} }] } });
  assert.deepEqual(dispatches.map(x => x.status), ['sending', 'accepted']);
  assert.ok(dispatches.every(x => x.id === 'a' && x.name === 'move'));
  assert.ok(dispatches.every(x => x.status !== 'completed'));
});

test('pausing dispatch blocks remaining buffered calls after an in-flight response', async () => {
  const { s, dispatches, responses } = service();
  let resolve, calls = 0;
  s.robotBridge.callTool = () => { calls++; return new Promise(r => { resolve = r; }); };
  const pending = s.handleMessage({ toolCall: { functionCalls: [
    { id: 'a', name: 'wait', args: { duration: 1 } },
    { id: 'b', name: 'move', args: {} },
  ] } });
  assert.equal(calls, 1);
  s.pauseRobotTools();
  resolve({ success: true, message: 'wait queued' });
  await pending;
  await Promise.resolve();
  assert.equal(calls, 1);
  assert.deepEqual(dispatches.map(entry => entry.status), ['sending']); // Late acceptance is stale after Stop/pause.
  assert.equal(responses[1].functionResponses.response.result.success, false);
  assert.match(responses[1].functionResponses.response.result.message, /paused/);
});


test('independent stop pauses dispatch before contacting server and never queues a posture', async () => {
  const events = [];
  let resolve;
  const model = { pauseRobotTools() { events.push('paused'); } };
  const bridge = {
    immediateStop() { events.push('direct-stop'); return new Promise(r => { resolve = r; }); },
    queueDeactivate() { throw new Error('must not queue lie_down'); },
  };
  const pending = requestIndependentStop(model, bridge);
  assert.deepEqual(events, ['paused', 'direct-stop']);
  resolve({ success: true, message: 'zero requested' });
  assert.equal((await pending).success, true);
});


test('unknown command outcome pauses later motion but preserves stop and status access', async () => {
  const { s, dispatches } = service();
  const calls = [];
  s.robotBridge.callTool = async name => {
    calls.push(name);
    return name === 'move'
      ? { success: false, outcome_unknown: true, message: 'link lost' }
      : { success: true, message: 'server response' };
  };
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'a', name: 'move', args: {} }] } });
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'b', name: 'move', args: {} }] } });
  await s.handleMessage({ toolCall: { functionCalls: [
    { id: 'c', name: 'stop', args: {} }, { id: 'd', name: 'robot_status', args: {} },
  ] } });
  assert.deepEqual(calls, ['move', 'stop', 'robot_status']);
  assert.equal(dispatches[1].status, 'unknown');
});


test('a late response cannot replace a newer request or a cleared voice session', () => {
  const a = { id: 'a', name: 'move', status: 'sending', timestamp: 1, message: '' };
  const b = { id: 'b', name: 'stop', status: 'sending', timestamp: 2, message: '' };
  let current = updateLatestDispatch(null, a);
  current = updateLatestDispatch(current, b);
  assert.equal(updateLatestDispatch(current, { ...a, status: 'accepted', timestamp: 3 }), current);
  assert.equal(updateLatestDispatch(current, { ...b, status: 'accepted' }).status, 'accepted');
  assert.equal(updateLatestDispatch(null, { ...a, status: 'accepted' }), null);
});

test('automatic Live close blocks buffered motion and suppresses its late response callback', async () => {
  const { s, dispatches } = service();
  let resolve;
  const calls = [];
  s.robotBridge.callTool = name => { calls.push(name); return new Promise(r => { resolve = r; }); };
  const pending = s.handleMessage({ toolCall: { functionCalls: [
    { id: 'a', name: 'wait', args: {} },
    { id: 'b', name: 'move', args: {} },
  ] } });
  s.cleanup();
  resolve({ success: true, message: 'wait queued' });
  await pending;
  assert.deepEqual(calls, ['wait']);
  assert.deepEqual(dispatches.map(e => e.status), ['sending']);
});


const { manualMove, MANUAL_DEFAULTS, MANUAL_DURATIONS } = await load('../services/manualControls.ts');
test('manual defaults are bounded and left/right match the HTTP convention', async (t) => {
  assert.deepEqual(manualMove('forward', MANUAL_DEFAULTS), { vx: 0.2, vy: 0, wz: 0, duration: 0.5 });
  assert.deepEqual(manualMove('left', MANUAL_DEFAULTS), { vx: 0, vy: 0, wz: -20, duration: 0.5 });
  assert.equal(manualMove('right', MANUAL_DEFAULTS).wz, 20);
  assert.equal(manualMove('backward', MANUAL_DEFAULTS).vx, -0.2);
  let request;
  t.mock.method(globalThis, 'fetch', async (url, options) => {
    if (url.endsWith('/health')) return { ok: true, json: async () => ({ command_epoch_required: true, command_epoch: 'test:0' }) };
    request = { url, body: JSON.parse(options.body) };
    return { ok: true, json: async () => ({ success: true, message: 'queued' }) };
  });
  await new RobotBridge('http://manual-server').queueMove(manualMove('left', MANUAL_DEFAULTS));
  assert.equal(request.url, 'http://manual-server/move/velocity');
  assert.equal(request.body.turning_velocity, -20);
  assert.equal(request.body.duration, 0.5);
});
test('invalid manual configurations never form a movement request', () => {
  for (const settings of [{ ...MANUAL_DEFAULTS, speed: 0.5 }, { ...MANUAL_DEFAULTS, duration: 60 },
      { ...MANUAL_DEFAULTS, turn: NaN }, { ...MANUAL_DEFAULTS, speed: -0.2 },
      { ...MANUAL_DEFAULTS, speed: 0.1 }, { ...MANUAL_DEFAULTS, turn: 10 }]) {
    assert.throws(() => manualMove('forward', settings));
  }
  assert.throws(() => manualMove('invalid', MANUAL_DEFAULTS));
});


test('all displayed manual presets pass the actual Python stock movement validator', () => {
  const commands = ['forward', 'backward', 'left', 'right'].flatMap(direction =>
    MANUAL_DURATIONS.map(duration => manualMove(direction, { ...MANUAL_DEFAULTS, duration })));
  // Execute only the real pure dataclass/validator definitions, without ROS imports.
  const script = `import ast, dataclasses, json, math, pathlib, sys
scope = {'dataclass': dataclasses.dataclass, 'math': math}
for path, name in [(sys.argv[1], 'MoveLimits'), (sys.argv[1], 'Result'), (sys.argv[2], '_motion_refusal'), (sys.argv[2], '_check_move_result'), (sys.argv[2], 'check_move')]:
    tree = ast.parse(pathlib.Path(path).read_text())
    node = next(node for node in tree.body if getattr(node, 'name', None) == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), path, 'exec'), scope)
    if name == 'MoveLimits': scope['LIMITS'] = scope['MoveLimits']()
limits = scope['MoveLimits']()
print(json.dumps([scope['check_move'](c['vx'], -c['vy'], -c['wz'], c['duration'], limits) for c in json.loads(sys.argv[3])]))
`;
  const output = execFileSync('python3', ['-c', script,
    fileURLToPath(new URL('../../robot/robot_api.py', import.meta.url)),
    fileURLToPath(new URL('../../lab/part2_walk.py', import.meta.url)), JSON.stringify(commands)], { encoding: 'utf8' });
  assert.deepEqual(JSON.parse(output), commands.map(() => null));
});


test('runtime labels distinguish fake queues from observed hardware', () => {
  assert.match(runtimeSummary({ ...valid, runtime: 'fake', hardware_connected: false }), /FAKE REHEARSAL.*no robot hardware/);
  assert.match(runtimeSummary({ ...valid, runtime: 'robot', hardware_connected: null }), /require observation/);
  assert.match(runtimeSummary(valid), /not identified/);
});

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
function mediaEnvironment(t, sdkConnect, getUserMedia, outputPolicy = 'receipts') {
  const contexts = [], tracks = [];
  class FakeStream {
    constructor(owned = []) { this.tracks = owned; }
    addTrack(track) { this.tracks.push(track); }
    getTracks() { return this.tracks; }
    getAudioTracks() { return this.tracks.filter(t => t.kind === 'audio'); }
    getVideoTracks() { return this.tracks.filter(t => t.kind === 'video'); }
  }
  const track = { kind: 'audio', stopped: 0, stop() { this.stopped++; } }; tracks.push(track);
  const node = () => ({ connect() {}, disconnect() {} });
  class FakeAudioContext {
    constructor({ sampleRate } = {}) { this.sampleRate = sampleRate; this.state = 'running'; this.destination = {}; contexts.push(this); }
    createAnalyser() { return { ...node(), getByteTimeDomainData() {} }; }
    createMediaStreamSource() { return node(); }
    createScriptProcessor(size) { this.inputBufferSize=size;return node(); }
    async close() { this.state = 'closed'; }
  }
  replaceGlobal(t, 'window', { AudioContext: FakeAudioContext, setInterval, speechSynthesis: { cancel() {} } });
  replaceGlobal(t, 'MediaStream', FakeStream);
  replaceGlobal(t, 'navigator', { mediaDevices: { getUserMedia: getUserMedia ?? (async () => new FakeStream([track])) } });
  replaceGlobal(t, '__fakeLiveSdkFactory', () => ({ live: { connect: sdkConnect } }));
  const logs = [], stops = [];
  const config = { modelName: 'gemini-3.1-flash-live-preview', robotApiUrl: 'http://test-server', enableAudio: true, enableVideo: false, outputPolicy };
  const s = new GeminiLiveService(config, e => logs.push(e), () => {}, () => {}, () => {}, message => stops.push(message));
  s.robotBridge.getTools = async () => ({ tools: [] });
  t.after(() => s.disconnect());
  return { s, config, contexts, tracks, logs, stops, FakeStream };
}

test('SDK failure releases mic and both contexts, scrubs key, and permits a fresh session', async t => {
  const env = mediaEnvironment(t, async () => { throw new Error('failed ?key=TEST_SECRET&x=1 TEST_SECRET'); });
  await assert.rejects(env.s.connect('TEST_SECRET', env.config, {}), error => {
    assert.doesNotMatch(error.message, /TEST_SECRET/);
    return true;
  });
  assert.equal(env.tracks[0].stopped, 1);
  assert.equal(env.contexts.length, 2);
  assert.ok(env.contexts.every(c => c.state === 'closed'));
  assert.equal(env.s.volumeInterval, null);
  assert.equal(env.s.connected, false);
  globalThis.__fakeLiveSdkFactory = () => ({ live: { connect: async ({ callbacks }) => {
    callbacks.onopen(); return { close() {} };
  } } });
  const fresh = new GeminiLiveService(env.config, () => {}, () => {}, () => {});
  fresh.robotBridge.getTools = async () => ({ tools: [] });
  await fresh.connect('TEST_SECRET', env.config, {});
  assert.equal(fresh.connected, true);
  await fresh.disconnect();
});

test('microphone denial fails voice connection instead of displaying listening', async t => {
  let connections = 0;
  const env = mediaEnvironment(t, () => { connections++; }, async () => { throw new Error('permission denied'); });
  await assert.rejects(env.s.connect('test', env.config, {}), /permission denied/);
  assert.equal(connections, 0);
  assert.ok(env.contexts.every(c => c.state === 'closed'));
});

test('disconnect while microphone permission is pending stops late tracks and never opens SDK', async t => {
  const permission = deferred(); let connections = 0;
  const env = mediaEnvironment(t, () => { connections++; }, () => permission.promise);
  const pending = env.s.connect('test', env.config, {});
  const rejection = assert.rejects(pending, /cancelled/);
  await env.s.disconnect();
  await rejection; // Cancellation settles without waiting for the browser permission promise.
  permission.resolve(new env.FakeStream(env.tracks));
  await Promise.resolve();
  assert.equal(env.tracks[0].stopped, 1);
  assert.equal(connections, 0);
});

test('disconnect never waits for pending SDK; late open cannot revive microphone or session', async t => {
  const connection = deferred(), started = deferred(); let callbacks, closed = 0;
  const env = mediaEnvironment(t, request => { callbacks = request.callbacks; started.resolve(); return connection.promise; });
  const pending = env.s.connect('test', env.config, {});
  const rejection = assert.rejects(pending, /cancelled/);
  await started.promise;
  await env.s.disconnect();
  assert.equal(env.tracks[0].stopped, 1);
  callbacks.onopen();
  assert.equal(env.s.connected, false);
  assert.equal(env.s.inputScriptProcessor, null);
  connection.resolve({ close() { closed++; } });
  await rejection;
  assert.equal(closed, 1);
});

test('interruption invalidates an already decoding audio buffer without stopping robot actions', async () => {
  const { s } = service('conversation'); const decoded = deferred(); let sources = 0, stops = 0;
  s.audioContext = { state: 'running', currentTime: 0, decodeAudioData: () => decoded.promise,
    createBufferSource() { sources++; throw new Error('stale audio must not start'); } };
  s.analyser = {};
  s.robotBridge.immediateStop = async () => { stops++; };
  const pending = s.playAudioBase64(Buffer.from('RIFF').toString('base64'));
  await s.handleMessage({ serverContent: { interrupted: true } });
  decoded.resolve({ duration: 1 });
  await pending;
  assert.equal(sources, 0);
  assert.equal(stops, 0);
});

test('interrupted pending TTS never resumes speech or falls back to browser speech', async () => {
  const { s } = service('conversation'); const response = deferred(); let playback = 0;
  s.config.ttsModelName = 'gemini-2.5-flash-preview-tts';
  s.ai = { models: { generateContent: () => response.promise } };
  s.playAudioBase64 = async () => { playback++; };
  s.speakWithWebSpeech = () => { playback++; };
  const pending = s.synthesizeAndPlaySpeech('stale response');
  await s.handleMessage({ serverContent: { interrupted: true } });
  response.resolve({ candidates: [{ content: { parts: [{ inlineData: { data: 'stale' } }] } }] });
  await pending;
  assert.equal(playback, 0);
});

test('all native audio parts are played, not only the first', async () => {
  const { s } = service('conversation'); const seen = [];
  s.playAudioBase64 = async audio => seen.push(audio);
  await s.handleMessage({ serverContent: { modelTurn: { parts: [
    { text: 'speech' }, { inlineData: { data: 'first' } }, { inlineData: { data: 'second' } }
  ] }, outputTranscription: {text: 'Hello there'}, turnComplete: true } });
  await new Promise(r => setImmediate(r));
  assert.deepEqual(seen, ['first', 'second']);
});

test('Live cancellation skips buffered tool before dispatch without inventing cancellation outcome', async () => {
  const { s, responses } = service(); const first = deferred(); const names = [];
  s.robotBridge.callTool = name => { names.push(name); return first.promise; };
  const pending = s.handleMessage({ toolCall: { functionCalls: [
    { id: 'a', name: 'wait', args: {} }, { id: 'b', name: 'move', args: {} }
  ] } });
  await s.handleMessage({ toolCallCancellation: { ids: ['b'] } });
  first.resolve({ success: true, message: 'queued' });
  await pending;
  assert.deepEqual(names, ['wait']);
  assert.equal(responses.length, 1);
});

test('cancellation of a sent action pauses dispatch and requests exactly one global Stop', async () => {
  const { s } = service(); const first = deferred(); const messages = []; let stops = 0;
  s.onAutomaticStop = message => messages.push(message);
  s.robotBridge.callTool = () => first.promise;
  s.robotBridge.immediateStop = async () => { stops++; return { success: false, outcome_unknown: true }; };
  const pending = s.handleMessage({ toolCall: { functionCalls: [{ id: 'a', name: 'move', args: {} }] } });
  await s.handleMessage({ toolCallCancellation: { ids: ['a'] } });
  await s.handleMessage({ toolCallCancellation: { ids: ['a'] } });
  assert.equal(stops, 1);
  assert.equal(s.robotToolsPaused, true);
  assert.equal(messages.at(-1), 'I could not confirm that request.');
  first.resolve({ success: true, message: 'queued' });
  await pending;
});

test('unexpected Live close after a sent action releases resources and requests global Stop', async t => {
  let callbacks, stops = 0, disconnected = 0;
  const env = mediaEnvironment(t, async request => { callbacks = request.callbacks; callbacks.onopen(); return { close() {} }; });
  env.s.onDisconnect = () => { disconnected++; };
  env.s.robotBridge.callTool = async () => ({ success: true, message: 'queued' });
  env.s.robotBridge.immediateStop = async () => { stops++; return { success: true, message: 'zero requested' }; };
  await env.s.connect('test', env.config, {});
  await env.s.handleMessage({ toolCall: { functionCalls: [{ id: 'a', name: 'move', args: {} }] } });
  await callbacks.onclose({ code: 1006, reason: 'link lost' });
  assert.equal(stops, 1);
  assert.equal(disconnected, 1);
  assert.equal(env.s.robotToolsPaused, true);
  assert.equal(env.tracks[0].stopped, 1);
  assert.equal(env.stops.at(-1), 'Stop requested.');
});

test('failed pending connection does not allow reconnect before its global Stop response', async t => {
  const connection = deferred(), started = deferred(), stop = deferred(); let callbacks, settled = false;
  const env = mediaEnvironment(t, request => {
    callbacks = request.callbacks; callbacks.onopen(); started.resolve(); return connection.promise;
  });
  env.s.robotBridge.callTool = async () => ({ success: true, message: 'queued' });
  env.s.robotBridge.immediateStop = () => stop.promise;
  const pending = env.s.connect('test', env.config, {}).finally(() => { settled = true; });
  const rejection = assert.rejects(pending, /cancelled/);
  await started.promise;
  await env.s.handleMessage({ toolCall: { functionCalls: [{ id: 'a', name: 'move', args: {} }] } });
  const closing = callbacks.onclose({ code: 1006, reason: 'gone' });
  connection.resolve({ close() {} });
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(settled, false);
  stop.resolve({ success: true, message: 'zero requested' });
  await closing;
  await rejection;
});

test('close before any action does not send an unsolicited global Stop', async t => {
  let callbacks, stops = 0;
  const env = mediaEnvironment(t, async request => { callbacks = request.callbacks; callbacks.onopen(); return { close() {} }; });
  env.s.robotBridge.immediateStop = async () => { stops++; };
  await env.s.connect('test', env.config, {});
  await callbacks.onclose({ code: 1006, reason: 'gone' });
  assert.equal(stops, 0);
});

test('GoAway retires an idle session before server deadline and exposes explicit reconnect', async t => {
  let callbacks, closed = 0, disconnected = 0, stops = 0;
  const env = mediaEnvironment(t, async request => {
    callbacks = request.callbacks; callbacks.onopen();
    return { close() { closed++; }, sendRealtimeInput() {} };
  });
  env.s.onDisconnect = () => { disconnected++; };
  env.s.robotBridge.immediateStop = async () => { stops++; return { success: true }; };
  await env.s.connect('test', env.config, {});
  assert.equal(env.s.beginAudioInput(), true);
  await callbacks.onmessage({ goAway: { timeLeft: '60s' } });
  assert.equal(env.s.connected, false);
  assert.equal(env.s.disposed, true);
  assert.equal(env.s.audioInputActive, false);
  assert.equal(env.tracks[0].stopped, 1);
  assert.ok(env.contexts.every(context => context.state === 'closed'));
  assert.equal(closed, 1);
  assert.equal(disconnected, 1);
  assert.equal(stops, 0);
  assert.ok(env.logs.some(entry => /session.*expir|expir.*session/i.test(entry.message)));
  assert.ok(env.logs.some(entry => /Reconnect/i.test(entry.message)));
});

test('GoAway fences a pending action batch, closes immediately, and waits only for global Stop', async t => {
  let callbacks, closed = 0, disconnected = 0;
  const reply = deferred(), stop = deferred(), calls = [], stops = [], pending = [], dispatches = [];
  const env = mediaEnvironment(t, async request => {
    callbacks = request.callbacks; callbacks.onopen();
    return { close() { closed++; }, sendToolResponse() {} };
  });
  env.s.onDisconnect = () => { disconnected++; };
  env.s.onStopPending = value => pending.push(value);
  env.s.onToolDispatch = value => dispatches.push(value);
  env.s.robotBridge.callTool = name => { calls.push(name); return reply.promise; };
  env.s.robotBridge.immediateStop = () => { stops.push('stop'); return stop.promise; };
  await env.s.connect('test', env.config, {});
  const action = callbacks.onmessage({ toolCall: { functionCalls: [
    { id: 'old-move', name: 'move', args: {} }, { id: 'old-turn', name: 'turn', args: {} },
  ] } });
  const retiring = callbacks.onmessage({ goAway: { timeLeft: '1s' } });
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(env.s.disposed, true);
  assert.equal(env.s.robotToolsPaused, true);
  assert.equal(closed, 1, 'SDK close cannot wait for robot HTTP');
  assert.deepEqual(stops, ['stop']);
  assert.deepEqual(pending, [true]);
  assert.equal(disconnected, 0, 'reconnect remains gated while Stop unresolved');
  await callbacks.onmessage({ goAway: {} });
  await callbacks.onclose({ code: 1008, reason: 'old socket closes late' });
  reply.resolve({ success: true, details: { submission: { schema: 'pupper_submission_v1', state: 'queued' } } });
  await action;
  assert.deepEqual(calls, ['move']);
  assert.deepEqual(dispatches.map(value => value.status), ['sending']);
  stop.resolve({ success: true }); await retiring;
  assert.deepEqual(pending, [true, false]);
  assert.equal(disconnected, 1);
  assert.deepEqual(stops, ['stop']);
});

test('GoAway takes priority over content and tools in the same packet', async t => {
  let callbacks, calls = 0, output = 0;
  const env = mediaEnvironment(t, async request => {
    callbacks = request.callbacks; callbacks.onopen(); return { close() {} };
  }, undefined, 'conversation');
  env.s.robotBridge.callTool = async () => { calls++; return { success: true }; };
  env.s.playAudioBase64 = async () => { output++; };
  await env.s.connect('test', env.config, {});
  await callbacks.onmessage({ goAway: { timeLeft: 'malformed' },
    toolCall: { functionCalls: [{ id: 'late', name: 'move', args: {} }] },
    serverContent: { outputTranscription: { text: 'Hello.' }, modelTurn: { parts: [{ inlineData: { data: 'old' } }] }, turnComplete: true },
  });
  assert.equal(calls, 0); assert.equal(output, 0); assert.equal(env.s.disposed, true);
});

test('GoAway does not wait for a pending SDK promise; a late session closes once', async t => {
  const sdk = deferred(), started = deferred(); let callbacks, closed = 0, disconnected = 0;
  const env = mediaEnvironment(t, request => {
    callbacks = request.callbacks; callbacks.onopen(); started.resolve(); return sdk.promise;
  });
  env.s.onDisconnect = () => { disconnected++; };
  const connecting = env.s.connect('test', env.config, {});
  const rejected = assert.rejects(connecting, /cancelled/);
  await started.promise;
  await callbacks.onmessage({ goAway: {} });
  assert.equal(disconnected, 1);
  assert.equal(env.tracks[0].stopped, 1);
  sdk.resolve({ close() { closed++; } }); await rejected;
  assert.equal(closed, 1);
});

test('GoAway never resumes or retries commands and a fresh explicit session can connect', async t => {
  let callbacks, connects = 0;
  const env = mediaEnvironment(t, async request => {
    connects++; callbacks = request.callbacks; callbacks.onopen();
    assert.equal(request.config.sessionResumption, undefined);
    return { close() {} };
  });
  await env.s.connect('test', env.config, {});
  await callbacks.onmessage({ sessionResumptionUpdate: { resumable: true, newHandle: 'untrusted-unused-handle' } });
  await callbacks.onmessage({ goAway: {} });
  assert.equal(connects, 1);
  assert.ok(env.logs.every(entry => !entry.message.includes('untrusted-unused-handle')));
  const fresh = new GeminiLiveService(env.config, () => {}, () => {}, () => {});
  fresh.robotBridge.getTools = async () => ({ tools: [] });
  await fresh.connect('test', env.config, {});
  assert.equal(connects, 2); assert.equal(fresh.connected, true);
  await fresh.disconnect();
});


test('Stop anywhere in one Live batch runs first and prevents partial motion dispatch', async () => {
  const { s, dispatches, responses } = service(); const calls = [];
  s.robotBridge.callTool = async name => { calls.push(name); return { success: true, message: 'accepted' }; };
  await s.handleMessage({ toolCall: { functionCalls: [
    { id: 'a', name: 'stand_up', args: {} }, { id: 'b', name: 'wait', args: { duration: 1 } },
    { id: 'c', name: 'stop', args: {} }, { id: 'd', name: 'robot_status', args: {} },
    { id: 'e', name: 'move', args: {} },
  ] } });
  assert.deepEqual(calls, ['stop', 'robot_status']);
  assert.deepEqual(dispatches.filter(d => d.status === 'sending').map(d => d.name), ['stop', 'robot_status']);
  const blocked = responses.map(r => r.functionResponses).filter(r => ['a', 'b', 'e'].includes(r.id));
  assert.equal(blocked.length, 3);
  assert.ok(blocked.every(r => !r.response.result.success && /Not dispatched/.test(r.response.result.message)));
});


test('audio decode in a mixed Live packet cannot delay Stop dispatch', async () => {
  const { s } = service('conversation'); const audio = deferred(), events = [];
  s.robotBridge.callTool = async name => { events.push(name); return { success: true, message: 'zero requested' }; };
  s.playAudioBase64 = () => { events.push('audio-decode'); return audio.promise; };
  const pending = s.handleMessage({
    serverContent: { modelTurn: { parts: [{ inlineData: { data: 'audio' } }] } },
    toolCall: { functionCalls: [{ id: 'stop', name: 'stop', args: {} }] },
  });
  assert.deepEqual(events, ['stop']);
  await new Promise(resolve => setImmediate(resolve));
  assert.deepEqual(events, ['stop']); // Stop invalidates this packet's obsolete speech.
  audio.resolve(); await pending;
});


test('model Stop pending lifecycle is balanced after disconnect without a final dispatch receipt', async () => {
  const { s, dispatches } = service(); const pending = deferred(), lifecycle = [];
  s.onStopPending = value => lifecycle.push(value);
  s.robotBridge.callTool = () => pending.promise;
  const handling = s.handleMessage({ toolCall: { functionCalls: [{ id: 'model-stop', name: 'stop', args: {} }] } });
  assert.deepEqual(lifecycle, [true]);
  s.connected = false; s.disposed = true;
  pending.resolve({ success: true, message: 'zero requested' }); await handling;
  assert.deepEqual(lifecycle, [true, false]);
  assert.deepEqual(dispatches.map(d => d.status), ['sending']);
});

test('throwing model Stop still releases its lifecycle gate', async () => {
  const { s } = service(); const lifecycle = [];
  s.onStopPending = value => lifecycle.push(value);
  s.robotBridge.callTool = async () => { throw new Error('transport failed'); };
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'failed-stop', name: 'stop', args: {} }] } });
  assert.deepEqual(lifecycle, [true, false]);
});

test('automatic Stop emits one typed lifecycle for a coalesced request even if its receipt fails', async () => {
  const { s } = service(); const pending = deferred(), lifecycle = [];
  s.onStopPending = value => lifecycle.push(value);
  s.robotBridge.immediateStop = () => pending.promise;
  const first = s.requestAutomaticStop('cancelled action');
  const second = s.requestAutomaticStop('connection closed');
  assert.deepEqual(lifecycle, [true]);
  s.connected = false; s.disposed = true;
  pending.reject(new Error('transport lost')); await Promise.all([first, second]);
  assert.deepEqual(lifecycle, [true, false]);
});


test('overlapping Live packets cannot send new motion while model Stop is pending', async () => {
  const { s, responses } = service(); const pending = deferred(), calls = [];
  s.robotBridge.callTool = async name => {
    calls.push(name);
    return name === 'stop' ? pending.promise : { success: true, message: 'accepted' };
  };
  const stopping = s.handleMessage({ toolCall: { functionCalls: [{ id: 'stop-pending', name: 'stop', args: {} }] } });
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'overlap', name: 'move', args: { direction: 'forward' } }] } });
  assert.deepEqual(calls, ['stop']);
  assert.match(JSON.stringify(responses), /Stop response is pending/);
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'check', name: 'robot_status', args: {} }] } });
  assert.deepEqual(calls, ['stop', 'robot_status']);
  pending.resolve({ success: true, message: 'zero requested' }); await stopping;
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'new-request', name: 'move', args: { direction: 'forward' } }] } });
  assert.deepEqual(calls, ['stop', 'robot_status', 'move']);
});


test('Live setup selects HIGH only for the tested 3.1 Flash Live model', async t => {
  for (const [model, thinking] of [
    ['gemini-3.1-flash-live-preview', { thinkingLevel: 'HIGH' }],
    ['models/gemini-3.1-flash-live-preview', { thinkingLevel: 'HIGH' }],
    ['gemini-2.5-flash-native-audio-preview', undefined],
    ['gemini-3.1-flash-tts-preview', undefined],
    ['gemini-robotics-er-1.5-preview', undefined],
  ]) {
    await t.test(model, async t => {
      let setup;
      const env = mediaEnvironment(t, async request => {
        setup = request;
        request.callbacks.onopen();
        return { close() {} };
      });
      env.config.modelName = model;
      await env.s.connect('test', env.config, {});
      assert.deepEqual(setup.config.thinkingConfig, thinking);
      assert.equal(setup.model, model === 'gemini-2.5-flash-native-audio-preview'
        ? 'gemini-2.5-flash-native-audio-preview-12-2025' : model);
    });
  }
});


test('timing separates incoming Stop dispatch from a prior blocked audio handler', async () => {
  const { s } = service('conversation'); const audio = deferred(), timing = [], calls = [];
  s.onTiming = event => timing.push(event);
  s.playAudioBase64 = () => audio.promise;
  s.robotBridge.callTool = async name => { calls.push(name); return { success: true, message: 'zero requested' }; };
  const playing = s.handleMessage({ serverContent: { modelTurn: { parts: [{ inlineData: { data: 'synthetic' } }] }, outputTranscription: {text: 'Hello there'}, turnComplete: true } });
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'timed-stop', name: 'stop', args: {} }] } });
  assert.deepEqual(calls, ['stop']);
  const start = timing.find(e => e.phase === 'output_audio_handle_start');
  const received = timing.find(e => e.phase === 'server_message_callback' && e.metadata.tool_count === 1);
  const sent = timing.find(e => e.phase === 'tool_call_start');
  const replied = timing.find(e => e.phase === 'tool_call_resolved');
  assert.ok(start.atMs <= received.atMs && received.atMs <= sent.atMs && sent.atMs <= replied.atMs);
  assert.equal(sent.metadata.message_sequence, received.metadata.message_sequence);
  assert.equal(timing.some(e => e.phase === 'output_audio_handle_return'), false);
  audio.resolve(); await playing; await new Promise(r => setImmediate(r));
  assert.ok(timing.find(e => e.phase === 'output_audio_handle_return').atMs >= replied.atMs);
});

test('diagnostics omit transcript/arguments and a throwing observer cannot prevent Stop', async () => {
  const { s } = service(); const timing = [], calls = [];
  s.onTiming = event => timing.push(event);
  s.robotBridge.callTool = async name => { calls.push(name); return { success: true, message: 'accepted' }; };
  await s.handleMessage({ serverContent: { inputTranscription: { text: 'SYNTHETIC_PRIVATE_TEXT', finished: true } },
    toolCall: { functionCalls: [{ id: 'unlock-probe', name: 'unlock', args: { password: 'SYNTHETIC_PASSWORD' } }] } });
  assert.doesNotMatch(JSON.stringify(timing), /SYNTHETIC_PRIVATE_TEXT|SYNTHETIC_PASSWORD/);
  assert.equal(timing[0].metadata.input_transcription_finished, true);
  assert.equal(timing[0].metadata.input_transcription_chars, 22);
  s.onTiming = () => { throw new Error('observer failed'); };
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'observer-stop', name: 'stop', args: {} }] } });
  assert.deepEqual(calls, ['unlock', 'stop']);
});

test('audio diagnostics identify captured frames and SDK send invocation without retaining PCM', async t => {
  const sent = [], timing = [];
  const env = mediaEnvironment(t, async ({ callbacks }) => {
    callbacks.onopen(); return { close() {}, sendRealtimeInput: input => sent.push(input) };
  });
  env.s.onTiming = event => timing.push(event);
  await env.s.connect('synthetic-key', env.config, {});
  assert.equal(env.s.beginAudioInput(),true);
  env.s.inputScriptProcessor.onaudioprocess({ inputBuffer: {
    sampleRate: 16000, getChannelData: () => new Float32Array([0, 0.1, -0.2, 0]),
  } });
  await Promise.resolve();
  assert.equal(sent.length, 1); assert.equal(sent[0].audio.mimeType, 'audio/pcm;rate=16000');
  assert.equal(env.s.inputAudioContext.inputBufferSize,512);
  assert.equal(timing.find(e=>e.phase==='audio_input_setup').metadata.buffer_frames,512);
  const capture = timing.find(e => e.phase === 'audio_capture');
  const send = timing.find(e => e.phase === 'audio_send_start');
  assert.equal(capture.metadata.samples, 4); assert.equal(capture.metadata.sample_rate, 16000);
  assert.equal(capture.metadata.audio_sequence, send.metadata.audio_sequence);
  assert.ok(send.atMs >= capture.atMs);
  assert.equal(timing.some(e => e.phase === 'audio_send_return'), true);
  assert.doesNotMatch(JSON.stringify(timing), /synthetic-key|"data":|"mimeType":/);
});


test('epoch rejection pauses subsequent model replans without hiding Stop or status', async () => {
  const { s, dispatches } = service(); const calls = [];
  s.robotBridge.callTool = async name => {
    calls.push(name);
    return name === 'move' ? { success: false, command_epoch_required: true, not_dispatched: true,
      outcome_unknown: false, message: 'stale epoch' } : { success: true };
  };
  await s.handleMessage({ toolCall: { functionCalls: [
    { id: 'old', name: 'move', args: {} }, { id: 'retry', name: 'move', args: {} }
  ] } });
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'replan', name: 'stand_up', args: {} }] } });
  await s.handleMessage({ toolCall: { functionCalls: [{ id: 'status', name: 'robot_status', args: {} }, { id: 'stop', name: 'stop', args: {} }] } });
  assert.deepEqual(calls, ['move', 'stop', 'robot_status']);
  assert.equal(dispatches.find(d => d.id === 'old' && d.status !== 'sending').status, 'rejected');
  assert.equal(dispatches.some(d => ['retry', 'replan'].includes(d.id)), false);
});

function videoEnvironment(t, source = 'browser') {
  const { s, logs } = service(), sent = [], readers = []; let tick, frame = 0;
  s.sessionPromise = Promise.resolve({ sendRealtimeInput(value) { sent.push(value); }, close() {} });
  replaceGlobal(t, 'window', { setInterval(fn) { tick = fn; return 54321; }, speechSynthesis: { cancel() {} } });
  replaceGlobal(t, 'FileReader', class {
    readAsDataURL(blob) { this.blob = blob; readers.push(this); }
    finish() { this.result = `data:image/jpeg;base64,${this.blob.id}`; this.onloadend(); }
  });
  replaceGlobal(t, 'document', { createElement() { return {
    getContext() { return { drawImage() {} }; },
    toBlob(callback) { callback({ id: `frame-${++frame}` }); },
  }; } });
  s.setupVideoInput({ videoWidth: 640, videoHeight: 480 }, source);
  t.after(() => s.disconnect());
  return { s, tick: () => tick(), readers, sent, logs };
}
const flushCallbacks = () => new Promise(resolve => setImmediate(resolve));

test('a slower old browser video conversion cannot replace a newer sent frame', async t => {
  const env = videoEnvironment(t);
  await env.tick(); await env.tick();
  env.readers[1].finish(); await flushCallbacks();
  env.readers[0].finish(); await flushCallbacks();
  assert.deepEqual(env.sent.map(value => value.video.data), ['frame-2']);
});

test('video conversion finishing after disconnect never reaches the SDK', async t => {
  const env = videoEnvironment(t);
  await env.tick(); await env.s.disconnect();
  env.readers[0].finish(); await flushCallbacks();
  assert.deepEqual(env.sent, []);
});

test('out-of-order camera HTTP responses cannot overwrite newer video frames', async t => {
  const env = videoEnvironment(t, 'server'), first = deferred(), second = deferred(); let requests = 0;
  env.s.robotBridge.getCameraImage = () => ++requests === 1 ? first.promise : second.promise;
  const old = env.tick(), fresh = env.tick();
  second.resolve({ id: 'new' }); await flushCallbacks(); env.readers[0].finish(); await fresh; await flushCallbacks();
  first.resolve({ id: 'old' }); await flushCallbacks();
  if (env.readers[1]) env.readers[1].finish();
  await old; await flushCallbacks();
  assert.deepEqual(env.sent.map(value => value.video.data), ['new']);
});


test('server camera availability logs once per transition without polling spam', async t => {
  const env = videoEnvironment(t, 'server'); let available = false;
  env.s.robotBridge.getCameraImage = async () => available ? { id: 'fresh' } : null;
  await env.tick(); await env.tick();
  assert.equal(env.logs.filter(e => e.type === 'error').length, 1);
  available = true;
  let pending = env.tick(); await flushCallbacks(); env.readers[0].finish(); await pending;
  pending = env.tick(); await flushCallbacks(); env.readers[1].finish(); await pending;
  assert.equal(env.logs.filter(e => e.type === 'info').length, 1);
  assert.equal(env.logs.length, 2);
});

test('late older camera failure cannot overwrite a newer received-frame state', async t => {
  const env = videoEnvironment(t, 'server'), first = deferred(), second = deferred(); let requests = 0;
  env.s.robotBridge.getCameraImage = () => ++requests === 1 ? first.promise : second.promise;
  const old = env.tick(), fresh = env.tick();
  second.resolve({ id: 'new' }); await flushCallbacks(); env.readers[0].finish(); await fresh;
  first.resolve(null); await old;
  assert.deepEqual(env.sent.map(v => v.video.data), ['new']);
  assert.equal(env.logs.filter(e => e.type === 'error').length, 0);
  assert.equal(env.logs.filter(e => e.type === 'info').length, 1);
});

test('newer unavailable camera response invalidates an older unfinished conversion', async t => {
  const env = videoEnvironment(t, 'server'); let requests = 0;
  env.s.robotBridge.getCameraImage = async () => ++requests === 1 ? { id: 'old' } : null;
  const old = env.tick(); await flushCallbacks();
  await env.tick(); env.readers[0].finish(); await old;
  assert.deepEqual(env.sent, []);
  assert.equal(env.logs.filter(e => e.type === 'error').length, 1);
});

test('camera failure arriving after disconnect emits neither a frame nor availability log', async t => {
  const env = videoEnvironment(t, 'server'), response = deferred();
  env.s.robotBridge.getCameraImage = () => response.promise;
  const pending = env.tick(); await env.s.disconnect(); response.resolve(null); await pending;
  assert.deepEqual(env.sent, []); assert.deepEqual(env.logs, []);
});


test('conversation advertises all robot tools and freezes policy before connect', async t => {
  let setup;
  const env = mediaEnvironment(t, async request => { setup = request; request.callbacks.onopen(); return {close(){}}; }, undefined, 'conversation');
  env.s.robotBridge.getTools = async()=>({tools:['stand_up','move','unlock','lock_tricks','play_trick','follow','stop','robot_status'].map(name=>({name}))});
  env.config.outputPolicy = 'receipts'; // Mutating settings cannot widen an existing session.
  await env.s.connect('test',env.config,{});
  assert.deepEqual(setup.config.tools[0].functionDeclarations.map(t=>t.name),['stand_up','move','unlock','lock_tricks','play_trick','follow','stop','robot_status']);
  assert.deepEqual(setup.config.outputAudioTranscription, {});
  assert.equal(env.s.outputPolicy,'conversation');
});

test('receipt-only text model never advertises generative speak', async t => {
  let setup;
  const env = mediaEnvironment(t, async request => {setup=request;request.callbacks.onopen();return {close(){}};});
  env.config.modelName='gemini-robotics-er-2-streaming-preview';
  env.s.robotBridge.getTools=async()=>({tools:[{name:'move'},{name:'stop'}]});
  await env.s.connect('test',env.config,{});
  assert.deepEqual(setup.config.tools[0].functionDeclarations.map(t=>t.name),['move','stop']);
});


test('aggregate timing keeps input unknown without transcription and records tool/receipt before boundary', async t => {
  let now = 1000; replaceGlobal(t, 'performance', {now:()=>now});
  const {s, logs} = service();
  s.robotBridge.callTool = async()=>{now=1040;return {success:true};};
  await s.handleMessage({toolCall:{functionCalls:[{id:'timed',name:'move',args:{}}]}});
  now=1100;await s.handleMessage({serverContent:{turnComplete:true}});
  const lines=logs.filter(x=>x.message.startsWith('Timing '));assert.equal(lines.length,1);
  const result=JSON.parse(lines[0].message.slice(7));
  assert.equal(result.input_transcript_first_ms,null);assert.equal(result.input_transcript_end_ms,null);
  assert.equal(result.observed_input_end_to_model_ms,null);assert.equal(result.tool_entry_ms,0);
  assert.equal(result.tool_reply_ms,40);assert.equal(result.receipt_ms,40);assert.equal(result.turn_complete_ms,100);
});

test('aggregate timing binds late HTTP receipt to its original turn and never retains transcript contents', async t => {
  let now=1000;replaceGlobal(t,'performance',{now:()=>now});const {s,logs}=service();const pending=deferred();
  await s.handleMessage({serverContent:{inputTranscription:{text:'PRIVATE USER WORDS',finished:true}}});
  now=1010;s.robotBridge.callTool=()=>pending.promise;
  const working=s.handleMessage({toolCall:{functionCalls:[{id:'timed',name:'move',args:{private:'SECRET_ARGUMENT'}}]}});
  now=1020;await s.handleMessage({serverContent:{turnComplete:true}});
  assert.equal(logs.some(x=>x.message.startsWith('Timing ')),false);
  now=1030;await s.handleMessage({serverContent:{modelTurn:{parts:[{text:'PRIVATE MODEL WORDS'}]},turnComplete:true}});
  now=1050;pending.resolve({success:true});await working;
  const summaries=logs.filter(x=>x.message.startsWith('Timing ')).map(x=>JSON.parse(x.message.slice(7)));
  const original=summaries.find(x=>x.turn===1);assert.equal(original.tool_reply_ms,50);assert.equal(original.turn_complete_ms,20);
  assert.equal(original.input_transcript_end_ms,0);assert.equal(original.observed_input_end_to_model_ms,10);
  assert.doesNotMatch(JSON.stringify(summaries),/PRIVATE|SECRET_ARGUMENT|timed/);
});

test('native timing reports scheduling after deferred decode without adding a playback wait',async t=>{
  let now=2000;replaceGlobal(t,'performance',{now:()=>now});const {s,logs}=service('conversation');const decoded=deferred();
  s.audioContext={state:'running',currentTime:1,decodeAudioData:()=>decoded.promise,
    createBufferSource:()=>({connect(){},start(){},stop(){}})};s.analyser={};
  await s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:Buffer.from('RIFF').toString('base64')}}]},outputTranscription:{text:'Hello there'},turnComplete:true}});
  let summaries=logs.filter(x=>x.message.startsWith('Timing ')).map(x=>JSON.parse(x.message.slice(7)));
  assert.equal(summaries.length,1);assert.equal(summaries[0].native_playback_scheduled_ms,null);
  now=2042;decoded.resolve({duration:1});await new Promise(r=>setImmediate(r));
  summaries=logs.filter(x=>x.message.startsWith('Timing ')).map(x=>JSON.parse(x.message.slice(7)));
  assert.equal(summaries.length,2);assert.equal(summaries[1].turn_complete_to_playback_ms,42);
  assert.equal(summaries[1].reason,'playback-update');
});

test('optional timing logger failure cannot prevent Stop or its real tool response',async()=>{
  const {s,responses}=service();s.onLog=entry=>{if(entry.message.startsWith('Timing '))throw Error('observer failed');};
  let calls=0;s.robotBridge.callTool=async name=>{assert.equal(name,'stop');calls++;return {success:true};};
  await s.handleMessage({toolCall:{functionCalls:[{id:'stop',name:'stop',args:{}}]},serverContent:{turnComplete:true}});await Promise.resolve();
  assert.equal(calls,1);assert.equal(responses.at(-1).functionResponses.response.result.success,true);
});


function audioFrame(s) {
 s.inputScriptProcessor.onaudioprocess({inputBuffer:{sampleRate:16000,getChannelData:()=>new Float32Array([.1,-.2,.3,0])}});
}

test('hold-to-talk defaults closed, disables idle tracks, flushes release and keeps Live connected',async t=>{
 const sent=[];let closed=0;
 const e=mediaEnvironment(t,async({callbacks})=>{callbacks.onopen();return {close(){closed++;},sendRealtimeInput:x=>sent.push(x)};});
 await e.s.connect('TEST',e.config,{});
 assert.equal(e.tracks[0].enabled,false);audioFrame(e.s);await Promise.resolve();assert.deepEqual(sent,[]);
 assert.equal(e.s.beginAudioInput(),true);assert.equal(e.tracks[0].enabled,true);audioFrame(e.s);await Promise.resolve();
 assert.ok(sent[0].audio);e.s.endAudioInput();assert.equal(e.tracks[0].enabled,false);
 audioFrame(e.s);await Promise.resolve();assert.deepEqual(sent.map(x=>x.audio?'audio':x),['audio',{audioStreamEnd:true}]);
 assert.equal(e.s.connected,true);assert.equal(closed,0);e.s.endAudioInput();await Promise.resolve();assert.equal(sent.length,2);
});

test('release invalidates captured PCM awaiting SDK callback, including rapid repress ordering',async t=>{
 const sent=[];const e=mediaEnvironment(t,async({callbacks})=>{callbacks.onopen();return {close(){},sendRealtimeInput:x=>sent.push(x)};});
 await e.s.connect('TEST',e.config,{});
 e.s.beginAudioInput();audioFrame(e.s);e.s.endAudioInput();e.s.beginAudioInput();audioFrame(e.s);
 await Promise.resolve();assert.deepEqual(sent.map(x=>x.audio?'audio':x),[{audioStreamEnd:true},'audio']);
 e.s.endAudioInput();await Promise.resolve();assert.deepEqual(sent.at(-1),{audioStreamEnd:true});
});

test('unresolved SDK promise cannot leak old capture after release or end a new stream out of order',async t=>{
 const opened=deferred(),sent=[];
 const e=mediaEnvironment(t,async({callbacks})=>{callbacks.onopen();return opened.promise;});
 const connecting=e.s.connect('TEST',e.config,{});await new Promise(r=>setImmediate(r));
 assert.equal(e.s.beginAudioInput(),true);audioFrame(e.s);e.s.endAudioInput();e.s.beginAudioInput();audioFrame(e.s);
 opened.resolve({close(){},sendRealtimeInput:x=>sent.push(x)});await connecting;await Promise.resolve();
 assert.deepEqual(sent.map(x=>x.audio?'audio':x),[{audioStreamEnd:true},'audio']);
});

test('Stop and disconnect close hold gate synchronously and stale capture never resumes',async t=>{
 for(const action of ['stop','disconnect']) {
  const sent=[],e=mediaEnvironment(t,async({callbacks})=>{callbacks.onopen();return {close(){},sendRealtimeInput:x=>sent.push(x)};});
  await e.s.connect('TEST',e.config,{});e.s.beginAudioInput();audioFrame(e.s);
  if(action==='stop')e.s.pauseRobotTools();else await e.s.disconnect();
  await Promise.resolve();assert.equal(e.s.audioInputActive,false);assert.equal(sent.some(x=>x.audio),false);
  if(action==='disconnect')assert.equal(e.s.beginAudioInput(),false);
 }
});

test('failed release boundary prevents further audio until fresh connection while Stop remains callable',async t=>{
 let stop=0;const e=mediaEnvironment(t,async({callbacks})=>{callbacks.onopen();return {close(){},sendRealtimeInput(){throw Error('closed');}};});
 await e.s.connect('TEST',e.config,{});e.s.beginAudioInput();e.s.endAudioInput();await Promise.resolve();
 assert.equal(e.s.beginAudioInput(),false);assert.equal(e.tracks[0].enabled,false);
 e.s.robotBridge.immediateStop=async()=>{stop++;return {success:true};};
 await requestIndependentStop(e.s,e.s.robotBridge);assert.equal(stop,1);
 assert.ok(e.logs.some(x=>x.message==='Audio release could not be sent. Reconnect before sending more speech.'));
});

test('release timing records SDK-send boundary separately from unobserved acoustic/transcript end',async t=>{
 let now=1000;replaceGlobal(t,'performance',{now:()=>now});
 const e=mediaEnvironment(t,async({callbacks})=>{callbacks.onopen();return {close(){},sendRealtimeInput(){}};});
 await e.s.connect('TEST',e.config,{});e.s.beginAudioInput();e.s.endAudioInput();await Promise.resolve();now=1250;
 await e.s.handleMessage({serverContent:{outputTranscription:{text:'Hello'},turnComplete:true}});
 const timing=JSON.parse(e.logs.find(x=>x.message.startsWith('Timing ')).message.slice(7));
 assert.equal(timing.latest_audio_release_sent_at_ms,1000);assert.equal(timing.latest_audio_release_to_first_callback_ms,250);
 assert.equal(timing.input_transcript_end_ms,null);assert.match(timing.note,/not acoustic end or guaranteed turn association/);
});


test('model-issued Stop closes audio gate before dispatch without waiting for boundary receipt',async t=>{
 const sent=[];const e=mediaEnvironment(t,async({callbacks})=>{callbacks.onopen();return {close(){},sendRealtimeInput:x=>sent.push(x),sendToolResponse(){}};});
 await e.s.connect('TEST',e.config,{});e.s.beginAudioInput();audioFrame(e.s);
 let called=false;e.s.robotBridge.callTool=async()=>{called=true;assert.equal(e.s.audioInputActive,false);return {success:true};};
 await e.s.handleMessage({toolCall:{functionCalls:[{id:'stop-hold',name:'stop',args:{}}]}});
 assert.equal(called,true);assert.equal(sent.some(x=>x.audio),false);assert.deepEqual(sent,[{audioStreamEnd:true}]);
});


test('combined ER mode returns text directly and keeps the same robot tool catalog',async t=>{
 let setup;const env=mediaEnvironment(t,async x=>{setup=x;x.callbacks.onopen();return {close(){}};},undefined,'conversation');
 env.config.modelName='gemini-robotics-er-2-streaming-preview';env.s.robotBridge.getTools=async()=>({prompt:'Pupper prompt',tools:[{name:'move'},{name:'stop'}]});
 await env.s.connect('TEST',env.config,{});
 assert.deepEqual(setup.config.responseModalities,['TEXT']);assert.deepEqual(setup.config.tools[0].functionDeclarations.map(t=>t.name),['move','stop']);
 assert.match(setup.config.systemInstruction,/Return conversation directly as text/);assert.doesNotMatch(setup.config.systemInstruction,/MUST invoke/);
});


test('ignored microphone permission times out, releases contexts and stops a late granted stream', async t => {
  const permission=deferred(); let connections=0, timeout;
  const env=mediaEnvironment(t,()=>{connections++;},()=>permission.promise);
  const nativeSetTimeout=globalThis.setTimeout;
  replaceGlobal(t,'setTimeout',(callback,ms,...args)=>ms===30_000 ? (timeout=callback,987654) : nativeSetTimeout(callback,ms,...args));
  const pending=env.s.connect('TEST',env.config,{});
  const rejected=assert.rejects(pending,/Microphone access timed out.*30 seconds.*site controls.*reconnect/);
  assert.ok(timeout);timeout();await rejected;
  assert.equal(connections,0);assert.equal(env.s.connecting,false);
  assert.ok(env.contexts.every(c=>c.state==='closed'));
  permission.resolve(new env.FakeStream(env.tracks));await Promise.resolve();
  assert.equal(env.tracks[0].stopped,1);assert.equal(env.s.mediaStream,null);
});

test('microphone platform errors give actionable guidance and never open Live',async t=>{
  for(const [name,expected] of [['NotAllowedError',/site controls.*privacy settings/],['NotFoundError',/Connect or select/],['NotReadableError',/Close other apps/]]) {
    let connections=0;
    const env=mediaEnvironment(t,()=>{connections++;},()=>Promise.reject(Object.assign(new Error('capture failed'),{name})));
    await assert.rejects(env.s.connect('TEST',env.config,{}),expected);
    assert.equal(connections,0);assert.ok(env.contexts.every(c=>c.state==='closed'));
  }
});

test('optional camera timeout retains disabled microphone and rejects late camera ownership',async t=>{
  const camera=deferred();let timeout,calls=0,connections=0;
  const env=mediaEnvironment(t,async({callbacks})=>{connections++;callbacks.onopen();return {close(){}};},()=>++calls===1?Promise.resolve(new env.FakeStream(env.tracks)):camera.promise);
  env.config.enableVideo=true;env.config.cameraSource='browser';env.s.setupVideoInput=()=>{};
  const nativeSetTimeout=globalThis.setTimeout;
  replaceGlobal(t,'setTimeout',(callback,ms,...args)=>ms===30_000 ? (timeout=callback,987655) : nativeSetTimeout(callback,ms,...args));
  const pending=env.s.connect('TEST',env.config,{play:async()=>{}});
  await new Promise(r=>setImmediate(r));timeout();await pending;
  assert.equal(connections,1);assert.equal(env.tracks[0].enabled,false);
  assert.ok(env.logs.some(x=>/Camera unavailable: Camera access timed out/.test(x.message)));
  const track={kind:'video',stopped:0,stop(){this.stopped++;}};
  camera.resolve(new env.FakeStream([track]));await Promise.resolve();
  assert.equal(track.stopped,1);assert.equal(env.s.mediaStream.getVideoTracks().length,0);
});


test('stream adoption failure promptly rejects connection and stops unowned tracks',async t=>{
 let connections=0;
 const env=mediaEnvironment(t,()=>{connections++;});
 env.FakeStream.prototype.addTrack=()=>{throw Error('cannot adopt');};
 await assert.rejects(env.s.connect('TEST',env.config,{}),/Microphone capture could not be initialized/);
 assert.equal(connections,0);assert.equal(env.tracks[0].stopped,1);
 assert.equal(env.s.cancelMediaAcquisition,null);assert.ok(env.contexts.every(c=>c.state==='closed'));
});


test('transport-ended event precedes deferred Stop but final disconnect waits and unknown stays unknown', async t => {
 let callbacks; const stop=deferred(), events=[];
 const env=mediaEnvironment(t,async request=>{callbacks=request.callbacks;callbacks.onopen();return {close(){},sendToolResponse(){}};});
 env.s.onTransportEnded=event=>events.push({event,connected:env.s.connected,paused:env.s.robotToolsPaused});
 env.s.onDisconnect=()=>events.push('released');
 env.s.robotBridge.callTool=async()=>({success:true});env.s.robotBridge.immediateStop=()=>stop.promise;
 await env.s.connect('SECRET_TEST_KEY',env.config,{});
 await callbacks.onmessage({toolCall:{functionCalls:[{id:'m',name:'move',args:{}}]}});
 const closing=callbacks.onclose({code:1008,reason:'The operation was aborted. SECRET_TEST_KEY ?key=other'});
 assert.equal(events.length,1);assert.equal(events[0].event.kind,'closed');assert.equal(events[0].event.code,1008);
 assert.doesNotMatch(events[0].event.reason,/SECRET_TEST_KEY|key=other/);
 assert.equal(events[0].connected,false);assert.equal(events[0].paused,true);
 await callbacks.onclose({code:1008,reason:'duplicate'});assert.equal(events.length,1);
 stop.resolve({success:false,outcome_unknown:true});await closing;
 assert.equal(events.at(-1),'released');assert.equal(env.stops.at(-1),'I could not confirm that request.');
});

test('GoAway reports transport end even without actions and a throwing observer cannot block cleanup',async t=>{
 let callbacks,notifications=0,released=0,closed=0,stops=0;
 const env=mediaEnvironment(t,async request=>{callbacks=request.callbacks;callbacks.onopen();return {close(){closed++;}};});
 env.s.onTransportEnded=event=>{assert.equal(event.kind,'goaway');notifications++;throw new Error('observer failed');};
 env.s.onDisconnect=()=>released++;env.s.robotBridge.immediateStop=async()=>{stops++;return {success:true};};
 await env.s.connect('test',env.config,{});await callbacks.onmessage({goAway:{timeLeft:'1s'}});
 assert.equal(notifications,1);assert.equal(released,1);assert.equal(closed,1);assert.equal(stops,0);
 await callbacks.onclose({code:1008,reason:'after GoAway'});assert.equal(notifications,1);
});


test('actual Live setup declares camera selection and unknown body alignment without changing tools',async t=>{
 for(const [enabled,source,label] of [[false,'browser','disabled'],[true,'browser','browser'],[true,'server','server']]){
  let setup;const e=mediaEnvironment(t,async x=>{setup=x;x.callbacks.onopen();return {close(){}};},undefined,'conversation');
  e.config.enableVideo=enabled;e.config.cameraSource=source;e.config.enableAudio=false;
  e.s.setupVideoInput=()=>{};e.s.robotBridge.getTools=async()=>({prompt:'Pupper prompt',tools:[{name:'move'},{name:'stop'}]});
  await e.s.connect('TEST',e.config,{play:async()=>{}});
  assert.match(setup.config.systemInstruction,new RegExp('Camera source context: '+label));
  assert.match(setup.config.systemInstruction,/does not establish a current frame/);
  if(enabled)assert.match(setup.config.systemInstruction,/orientation and mirroring are unverified/);
  if(source==='browser'&&enabled)assert.match(setup.config.systemInstruction,/Do not use browser image position to choose a robot paw/);
  if(!enabled)assert.match(setup.config.systemInstruction,/No camera images are sent by this connection/);
  assert.deepEqual(setup.config.tools[0].functionDeclarations.map(x=>x.name),['move','stop']);
  await e.s.disconnect();
 }
});


test('hands-free explicitly arms one existing PCM pipeline with provider VAD and AEC',async t=>{
 const sent=[];let setup,mediaConstraints;
 const e=mediaEnvironment(t,async x=>{setup=x;x.callbacks.onopen();return {close(){},sendRealtimeInput:x=>sent.push(x)};},async c=>{
  mediaConstraints=c;return new e.FakeStream(e.tracks);
 });
 e.config.voiceInputMode='hands_free';await e.s.connect('TEST',e.config,{});
 assert.equal(e.s.audioInputActive,true);assert.equal(e.tracks[0].enabled,true);
 assert.equal(mediaConstraints.audio.echoCancellation,true);assert.equal(mediaConstraints.audio.noiseSuppression,true);
 assert.notEqual(setup.config.realtimeInputConfig?.automaticActivityDetection?.disabled,true);
 audioFrame(e.s);await Promise.resolve();audioFrame(e.s);await Promise.resolve();
 assert.equal(sent.filter(x=>x.audio).length,2);assert.equal(e.contexts.length,2);
 e.s.endAudioInput();await Promise.resolve();assert.equal(e.s.beginAudioInput(),false);
 assert.equal(e.s.connected,true);assert.equal(e.tracks[0].enabled,false);
 assert.equal(sent.filter(x=>x.audioStreamEnd).length,1);
});
test('hands-free model Stop disarms immediately and cannot admit a later model movement',async t=>{
 const sent=[],calls=[];
 const e=mediaEnvironment(t,async({callbacks})=>{callbacks.onopen();return {close(){},sendRealtimeInput:x=>sent.push(x),sendToolResponse(){}};});
 e.config.voiceInputMode='hands_free';await e.s.connect('TEST',e.config,{});audioFrame(e.s);
 e.s.robotBridge.callTool=async name=>{calls.push(name);assert.equal(e.s.audioInputActive,false);return {success:true};};
 await e.s.handleMessage({toolCall:{functionCalls:[{id:'stop',name:'stop',args:{}}]}});
 assert.deepEqual(calls,['stop']);assert.equal(e.s.beginAudioInput(),false);assert.equal(e.s.robotToolsPaused,true);
 audioFrame(e.s);await Promise.resolve();assert.equal(sent.some(x=>x.audio),false);
 await e.s.handleMessage({toolCall:{functionCalls:[{id:'late',name:'move',args:{}}]}});
 assert.deepEqual(calls,['stop']);
});
for(const ending of ['independent-stop','error','close','disconnect','send-error']){
 test(`hands-free ${ending} leaves input disarmed without reconnect`,async t=>{
  let callbacks,connections=0;const sent=[];
  const e=mediaEnvironment(t,async x=>{callbacks=x.callbacks;connections++;callbacks.onopen();return {
   close(){},sendRealtimeInput:value=>{if(ending==='send-error')throw Error('send failed');sent.push(value);}};});
  e.config.voiceInputMode='hands_free';await e.s.connect('TEST',e.config,{});assert.equal(e.s.audioInputActive,true);
  e.s.robotBridge.immediateStop=async()=>({success:true});
  if(ending==='independent-stop')await requestIndependentStop(e.s,e.s.robotBridge);
  if(ending==='error')callbacks.onerror({message:'test transport error'});
  if(ending==='close')await callbacks.onclose({code:1006});
  if(ending==='disconnect')await e.s.disconnect();
  if(ending==='send-error'){audioFrame(e.s);await Promise.resolve();await Promise.resolve();}
  assert.equal(e.s.audioInputActive,false);assert.equal(e.s.beginAudioInput(),false);assert.equal(e.tracks[0].enabled,false);
  assert.equal(connections,1);
 });
}
test('Stop during hands-free connection prevents late onopen from arming the microphone',async t=>{
 const opened=deferred();let callbacks;
 const e=mediaEnvironment(t,async x=>{callbacks=x.callbacks;await opened.promise;callbacks.onopen();return {close(){},sendRealtimeInput(){}};});
 e.config.voiceInputMode='hands_free';const connecting=e.s.connect('TEST',e.config,{});await new Promise(r=>setImmediate(r));
 e.s.pauseRobotTools();opened.resolve();await connecting;
 assert.equal(e.s.audioInputActive,false);assert.equal(e.s.beginAudioInput(),false);assert.equal(e.tracks[0].enabled,false);
});


for(const failure of ['provider-error','pcm-send-error']){
 test(`hands-free ${failure} after accepted follow sends one independent Stop even without socket close`,async t=>{
  let callbacks,stops=0;const received=[],stop=deferred();
  const e=mediaEnvironment(t,async x=>{callbacks=x.callbacks;callbacks.onopen();return {
   close(){},sendRealtimeInput(value){if(failure==='pcm-send-error'&&value.audio)throw Error('send failed');},sendToolResponse(){}};});
  e.config.voiceInputMode='hands_free';await e.s.connect('TEST',e.config,{});
  e.s.robotBridge.callTool=async name=>{received.push(name);return {success:true};};
  e.s.robotBridge.immediateStop=()=>{stops++;return stop.promise;};
  await e.s.handleMessage({toolCall:{functionCalls:[{id:'active-follow',name:'follow',args:{}}]}});
  assert.deepEqual(received,['follow']);
  if(failure==='provider-error'){callbacks.onerror({message:'transport failed'});callbacks.onerror({message:'still failed'});}
  else{audioFrame(e.s);await Promise.resolve();await Promise.resolve();audioFrame(e.s);}
  assert.equal(e.s.audioInputActive,false);assert.equal(e.s.beginAudioInput(),false);assert.equal(stops,1);
  await e.s.handleMessage({toolCall:{functionCalls:[{id:'late-move',name:'move',args:{}}]}});
  assert.deepEqual(received,['follow']);
  stop.resolve({success:true});await e.s.automaticStop;
 });
}
