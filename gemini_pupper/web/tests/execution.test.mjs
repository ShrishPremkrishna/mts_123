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
  const bridge = new RobotBridge('http://test-server');
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

function service() {
  const dispatches = [], logs = [], responses = [];
  const config = { modelName: 'gemini-3.1-flash-live-preview', robotApiUrl: 'http://test-server' };
  const s = new GeminiLiveService(config, e => logs.push(e), () => {}, () => {}, e => dispatches.push(e));
  s.connected = true;
  s.sessionPromise = Promise.resolve({ sendToolResponse: r => responses.push(r) });
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
  assert.equal(dispatches.length, 2);
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
    request = { url, body: JSON.parse(options.body) };
    return { ok: true, json: async () => ({ success: true, message: 'queued' }) };
  });
  await new RobotBridge('http://test-server').queueMove(manualMove('left', MANUAL_DEFAULTS));
  assert.equal(request.url, 'http://test-server/move/velocity');
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
for path, name in [(sys.argv[1], 'MoveLimits'), (sys.argv[2], 'check_move')]:
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
function mediaEnvironment(t, sdkConnect, getUserMedia) {
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
    constructor() { this.state = 'running'; this.destination = {}; contexts.push(this); }
    createAnalyser() { return { ...node(), getByteTimeDomainData() {} }; }
    createMediaStreamSource() { return node(); }
    createScriptProcessor() { return node(); }
    async close() { this.state = 'closed'; }
  }
  replaceGlobal(t, 'window', { AudioContext: FakeAudioContext, setInterval, speechSynthesis: { cancel() {} } });
  replaceGlobal(t, 'MediaStream', FakeStream);
  replaceGlobal(t, 'navigator', { mediaDevices: { getUserMedia: getUserMedia ?? (async () => new FakeStream([track])) } });
  replaceGlobal(t, '__fakeLiveSdkFactory', () => ({ live: { connect: sdkConnect } }));
  const logs = [], stops = [];
  const config = { modelName: 'gemini-3.1-flash-live-preview', robotApiUrl: 'http://test-server', enableAudio: true, enableVideo: false };
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
  permission.resolve(new env.FakeStream(env.tracks));
  await rejection;
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
  const { s } = service(); const decoded = deferred(); let sources = 0, stops = 0;
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
  const { s } = service(); const response = deferred(); let playback = 0;
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
  const { s } = service(); const seen = [];
  s.playAudioBase64 = async audio => seen.push(audio);
  await s.handleMessage({ serverContent: { modelTurn: { parts: [
    { text: 'speech' }, { inlineData: { data: 'first' } }, { inlineData: { data: 'second' } }
  ] } } });
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
  assert.match(messages.at(-1), /unknown/);
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
  assert.match(env.stops.at(-1), /does not prove physical stillness/);
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
  const { s } = service(); const audio = deferred(), events = [];
  s.robotBridge.callTool = async name => { events.push(name); return { success: true, message: 'zero requested' }; };
  s.playAudioBase64 = () => { events.push('audio-decode'); return audio.promise; };
  const pending = s.handleMessage({
    serverContent: { modelTurn: { parts: [{ inlineData: { data: 'audio' } }] } },
    toolCall: { functionCalls: [{ id: 'stop', name: 'stop', args: {} }] },
  });
  assert.deepEqual(events, ['stop']);
  await new Promise(resolve => setImmediate(resolve));
  assert.deepEqual(events, ['stop', 'audio-decode']);
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
