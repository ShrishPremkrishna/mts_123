import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
const bundle = await build({ entryPoints: ['rehearsal/fakeOnly.ts'], bundle: true, write: false, format: 'esm', platform: 'node' });
const { loopbackBase, assertFake, fakeOnlyFetch } = await import('data:text/javascript;base64,' + Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const health = { runtime: 'fake', hardware_connected: false, tools: true, command_epoch_required: true, command_epoch: 'test:0' };
const reply = value => ({ ok: true, json: async () => value });
test('rehearsal refuses non-loopback, credentials and ambiguous base URLs', () => {
  assert.equal(loopbackBase('http://127.0.0.1:18091/'), 'http://127.0.0.1:18091');
  for (const bad of ['https://example.com', 'http://127.0.0.1.evil', 'file:///tmp/api', 'http://u:p@localhost', 'http://localhost/api', 'http://localhost?api=x']) {
    assert.throws(() => loopbackBase(bad));
  }
});
test('rehearsal checks fake markers and epoch protocol, never trusting a localhost native API', async () => {
  await assertFake('http://localhost', async () => reply(health));
  for (const partial of [{ runtime: 'robot' }, { hardware_connected: null }, { tools: false }, { command_epoch: null }, { command_epoch_required: false }]) {
    await assert.rejects(assertFake('http://localhost', async () => reply({ ...health, ...partial })));
  }
});
test('each mutation rechecks fake identity before dispatch and forbids every other origin', async () => {
  const calls = []; let native = false;
  const wrapped = fakeOnlyFetch('http://localhost:18091', async (url) => {
    calls.push(url); return reply(native ? { ...health, runtime: 'robot' } : health);
  });
  await wrapped('http://localhost:18091/tools/wait', { method: 'POST' });
  assert.deepEqual(calls, ['http://localhost:18091/health', 'http://localhost:18091/tools/wait']);
  native = true; await assert.rejects(wrapped('http://localhost:18091/tools/stop', { method: 'POST' }));
  assert.equal(calls.at(-1), 'http://localhost:18091/health');
  await assert.rejects(wrapped('https://generativelanguage.googleapis.com/x'));
  assert.equal(calls.length, 3);
});
