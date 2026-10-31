import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { fileURLToPath } from 'node:url';
const bundle = await build({ entryPoints: [fileURLToPath(new URL('../services/robotBridge.ts', import.meta.url))], bundle: true, write: false, format: 'esm', platform: 'node' });
const { default: RobotBridge } = await import('data:text/javascript;base64,' + Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const reply = (value, status = 200) => ({ ok: status < 400, status, json: async () => value });
const epoch = (value = 'instance:0') => ({ command_epoch_required: true, command_epoch: value });
const accepted = value => reply({ success: true, message: 'zero requested', ...epoch(value) });
function deferred() { let resolve; const promise = new Promise(r => { resolve = r; }); return { promise, resolve }; }
let testNumber = 0;
let base;
test.beforeEach(() => { base = `http://fake-${++testNumber}`; });
const move = b => b.callTool('move', { direction: 'forward', speed: 0.2, duration: 0.5 });

test('old or unavailable server fails closed before any mutation POST', async t => {
  const calls = [];
  t.mock.method(globalThis, 'fetch', async (url, options) => { calls.push([url, options.method]); return reply({ status: 'running' }); });
  const result = await move(new RobotBridge(base));
  assert.equal(result.success, false); assert.equal(result.not_dispatched, true);
  assert.deepEqual(calls, [[`${base}/health`, 'GET']]);
});

test('Stop overtakes a pending epoch handshake without reauthorizing its old intent', async t => {
  const health = deferred(), calls = [];
  t.mock.method(globalThis, 'fetch', async (url, options) => {
    calls.push([url, options.method]);
    return url.endsWith('/health') ? health.promise : accepted('instance:1');
  });
  const b = new RobotBridge(base); const old = move(b);
  await b.immediateStop(); health.resolve(reply(epoch('instance:0')));
  const result = await old;
  assert.equal(result.not_dispatched, true); assert.match(result.message, /Stop occurred/);
  assert.deepEqual(calls.map(c => c[0]), [`${base}/health`, `${base}/stop/immediate`]);
});

test('missing/stale token response is explicit rejection and never automatically retried or adopted', async t => {
  const calls = []; let generation = 0, reject = true;
  t.mock.method(globalThis, 'fetch', async (url, options) => {
    calls.push({ url, headers: options.headers });
    if (url.endsWith('/health')) return reply(epoch(`instance:${generation}`));
    if (reject) { generation++; return reply({ success: false, message: 'stale command epoch', ...epoch(`instance:${generation}`) }, 409); }
    return reply({ success: true, message: 'queued' });
  });
  const b = new RobotBridge(base); const result = await move(b);
  assert.equal(result.not_dispatched, true); assert.equal(result.outcome_unknown, false);
  assert.equal(calls.length, 2); assert.equal(calls[1].headers['X-Pupper-Command-Epoch'], 'instance:0');
  reject = false; assert.equal((await move(b)).success, true);
  assert.equal(calls[2].url, `${base}/health`);
  assert.equal(calls[3].headers['X-Pupper-Command-Epoch'], 'instance:1');
});

test('POST held before application entry retains old epoch and cannot execute after Stop', async t => {
  const held = deferred(); let generation = 0, executions = 0, headers;
  t.mock.method(globalThis, 'fetch', async (url, options) => {
    if (url.endsWith('/health')) return reply(epoch(`instance:${generation}`));
    if (url.endsWith('/stop/immediate')) return accepted(`instance:${++generation}`);
    headers = options.headers; await held.promise;
    if (headers['X-Pupper-Command-Epoch'] !== `instance:${generation}`) return reply({ success: false, message: 'stale', ...epoch(`instance:${generation}`) }, 409);
    executions++; return reply({ success: true, message: 'queued' });
  });
  const b = new RobotBridge(base); const old = move(b);
  while (!headers) await Promise.resolve();
  await b.immediateStop(); held.resolve(); const result = await old;
  assert.equal(headers['X-Pupper-Command-Epoch'], 'instance:0');
  assert.equal(executions, 0); assert.equal(result.superseded, true);
  assert.equal(result.outcome_unknown, false);
});

test('pending/unknown Stop blocks motion, status stays available, later confirmed Stop recovers', async t => {
  const stop = deferred(); let stops = 0, motions = 0;
  t.mock.method(globalThis, 'fetch', async (url) => {
    if (url.endsWith('/health')) return reply(epoch('instance:2'));
    if (url.endsWith('/stop/immediate')) return ++stops === 1 ? stop.promise : accepted('instance:2');
    if (url.endsWith('/tools/robot_status')) return reply({ success: true, message: 'state available' });
    motions++; return reply({ success: true, message: 'queued' });
  });
  const b = new RobotBridge(base); const stopping = b.immediateStop();
  assert.equal((await move(b)).not_dispatched, true);
  assert.equal((await b.callTool('robot_status', {})).success, true);
  stop.resolve(reply({ success: false, outcome_unknown: true, message: 'response lost' })); await stopping;
  assert.match((await move(b)).message, /previous Stop failed or its outcome is unknown/); assert.equal(motions, 0);
  await b.immediateStop(); assert.equal((await move(b)).success, true); assert.equal(motions, 1);
});

test('late previous acceptance is superseded and unrelated old status cannot refresh epoch', async t => {
  const motion = deferred(), status = deferred(); const requests = []; let generation = 0, postCount = 0;
  t.mock.method(globalThis, 'fetch', async (url, options) => {
    requests.push({ url, headers: options.headers });
    if (url.endsWith('/health')) return reply(epoch(`instance:${generation}`));
    if (url.endsWith('/queue/status')) return status.promise;
    if (url.endsWith('/stop/immediate')) return accepted(`instance:${++generation}`);
    return ++postCount === 1 ? motion.promise : reply({ success: true, message: 'new queued' });
  });
  const b = new RobotBridge(base); const old = move(b);
  while (postCount === 0) await Promise.resolve();
  const oldStatus = b.getQueueStatus(); await b.immediateStop();
  motion.resolve(reply({ success: true, message: 'old accepted before Stop' }));
  const late = await old; assert.equal(late.success, false); assert.equal(late.superseded, true);
  assert.equal(late.outcome_unknown, true); assert.equal(late.original_response.success, true);
  status.resolve(reply(epoch('instance:0'))); await oldStatus;
  assert.equal((await move(b)).success, true);
  assert.equal(requests.at(-2).url, `${base}/health`);
  assert.equal(requests.at(-1).headers['X-Pupper-Command-Epoch'], 'instance:1');
});

test('all Stop aliases and read-only tool status bypass epoch handshake and header', async t => {
  const calls = [];
  t.mock.method(globalThis, 'fetch', async (url, options) => { calls.push({ url, options }); return accepted('instance:1'); });
  const b = new RobotBridge(base);
  await b.immediateStop(); await b.queueStop(); await b.callTool('stop', {}); await b.callTool('robot_status', {});
  assert.equal(calls.length, 4);
  assert.ok(calls.every(c => !c.url.endsWith('/health') && !('X-Pupper-Command-Epoch' in c.options.headers)));
});


test('second same-server bridge Stop invalidates old handshake even returning new epoch', async t => {
  const held = deferred(); let posts = 0;
  t.mock.method(globalThis, 'fetch', async url => {
    if (url.endsWith('/health')) return held.promise;
    if (url.endsWith('/stop/immediate')) return accepted('instance:1');
    posts++; return reply({ success: true });
  });
  const first = new RobotBridge(base), second = new RobotBridge(base.toUpperCase() + ':80/');
  const old = move(first); await second.immediateStop(); held.resolve(reply(epoch('instance:1')));
  assert.equal((await old).not_dispatched, true); assert.equal(posts, 0);
});

test('replacement bridges preserve failed/unknown Stop latch and isolate other servers', async t => {
  let success = false, unknown = false, motions = 0;
  t.mock.method(globalThis, 'fetch', async url => {
    if (url.endsWith('/health')) return reply(epoch('instance:1'));
    if (url.endsWith('/stop/immediate')) return reply({ success, outcome_unknown: unknown, ...epoch('instance:1') });
    motions++; return reply({ success: true });
  });
  await new RobotBridge(base).immediateStop();
  assert.equal((await move(new RobotBridge(base))).not_dispatched, true);
  unknown = true; await new RobotBridge(base).immediateStop();
  assert.equal((await move(new RobotBridge(base))).not_dispatched, true);
  assert.equal((await move(new RobotBridge(base + '-other'))).success, true);
  assert.equal(motions, 1);
  success = true; unknown = false; await new RobotBridge(base).immediateStop();
  assert.equal((await move(new RobotBridge(base))).success, true);
});

test('late accepted POST is superseded by another same-server bridge Stop', async t => {
  const held = deferred(); let submitted = false;
  t.mock.method(globalThis, 'fetch', async url => {
    if (url.endsWith('/health')) return reply(epoch('instance:0'));
    if (url.endsWith('/stop/immediate')) return accepted('instance:1');
    submitted = true; return held.promise;
  });
  const old = move(new RobotBridge(base));
  while (!submitted) await Promise.resolve();
  await new RobotBridge(base).immediateStop();
  held.resolve(reply({ success: true, message: 'previously queued' }));
  const result = await old;
  assert.equal(result.superseded, true); assert.equal(result.outcome_unknown, true); assert.equal(result.success, false);
});
