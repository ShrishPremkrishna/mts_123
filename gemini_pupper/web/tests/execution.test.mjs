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
        "export class GoogleGenAI {} export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};" }));
    }}] : [],
  });
  return import('data:text/javascript;base64,' + Buffer.from(result.outputFiles[0].text).toString('base64'));
}
const { parseQueueStatus, queueSummary, toolResponseStatus, requestIndependentStop, updateLatestDispatch } = await load('../services/executionStatus.ts');
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
  await s.handleMessage({ toolCall: { functionCalls: [
    { id: 'a', name: 'move', args: {} },
    { id: 'b', name: 'move', args: {} },
    { id: 'c', name: 'stop', args: {} },
    { id: 'd', name: 'robot_status', args: {} },
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
