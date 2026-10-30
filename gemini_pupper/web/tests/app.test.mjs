import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { fileURLToPath } from 'node:url';

// Render the actual App handlers with minimal hooks and inert child components.
// No DOM/media, real SDK, HTTP, or additional test framework is involved.
const bundle = await build({
  entryPoints: [fileURLToPath(new URL('../App.tsx', import.meta.url))],
  bundle: true, write: false, format: 'esm', platform: 'node', jsx: 'transform', tsconfigRaw: { compilerOptions: { jsx: 'react' } },
  plugins: [{ name: 'app-boundary', setup(b) {
    b.onResolve({ filter: /^react$/ }, () => ({ path: 'react', namespace: 'stub' }));
    b.onResolve({ filter: /\/components\// }, a => ({ path: a.path.split('/').at(-1), namespace: 'stub' }));
    b.onResolve({ filter: /\/services\/geminiLive$/ }, () => ({ path: 'live', namespace: 'stub' }));
    b.onLoad({ filter: /.*/, namespace: 'stub' }, a => ({ contents: a.path === 'react'
      ? `const h=globalThis.__appHooks; export const {useState,useEffect,useRef,useMemo,useCallback}=h; export default {createElement:(type,props,...children)=>({type,props:{...props,children}})};`
      : a.path === 'live' ? `export const GeminiLiveService=globalThis.__appLive;`
      : `export default '${a.path}';` }));
  }}],
});
function deferred() {
  let resolve, reject;
  const promise = new Promise((a, b) => { resolve = a; reject = b; });
  return { promise, resolve, reject };
}
function find(node, predicate) {
  if (!node || typeof node !== 'object') return null;
  if (predicate(node)) return node;
  for (const child of node.props?.children?.flat(Infinity) ?? []) {
    const match = find(child, predicate); if (match) return match;
  }
  return null;
}
function wake(tree) { return find(tree, n => n.type === 'button' && n.props.children.includes(' WAKE PUPPER')); }
async function app(t) {
  const cells = [], effects = [], instances = []; let cursor = 0;
  const setGlobal = (name, value) => {
    const old = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, writable: true, value });
    t.after(() => old ? Object.defineProperty(globalThis, name, old) : delete globalThis[name]);
  };
  const state = init => {
    const index = cursor++;
    if (!(index in cells)) cells[index] = typeof init === 'function' ? init() : init;
    return [cells[index], value => { cells[index] = typeof value === 'function' ? value(cells[index]) : value; }];
  };
  setGlobal('__appHooks', {
    useState: state, useRef: value => state(() => ({ current: value }))[0],
    useMemo: fn => state(fn)[0], useCallback: fn => fn,
    useEffect(fn, deps) { const [done, setDone] = state(false); if (!done && deps.length === 0) { effects.push(fn); setDone(true); } },
  });
  setGlobal('__appLive', class {
    constructor(...callbacks) {
      this.callbacks = callbacks; this.connection = deferred(); this.stop = deferred();
      this.robotBridge = { immediateStop: () => this.stop.promise }; instances.push(this);
    }
    connect() { return this.connection.promise; }
    pauseRobotTools() {}
    async disconnect() { this.connection.reject(new Error('Voice connection cancelled')); }
  });
  setGlobal('localStorage', { getItem: key => key === 'pupster_api_key' ? 'TEST_ONLY' : null });
  setGlobal('window', { location: { origin: 'http://test-server' } });
  setGlobal('document', { createElement: () => ({}) });
  const { default: App } = await import('data:text/javascript;base64,' + Buffer.from(bundle.outputFiles[0].text).toString('base64') + '#' + Math.random());
  const render = () => { cursor = 0; return App(); };
  render(); effects.splice(0).forEach(fn => fn());
  return { render, instances };
}

test('App holds reconnect while explicit disconnect waits for independent Stop', async t => {
  const { render, instances } = await app(t);
  const staleWake = wake(render()).props.onClick;
  const connecting = staleWake();
  const service = instances[0];
  const disconnecting = find(render(), n => n.props.title === 'Sleep / Disconnect').props.onClick();
  await connecting; // Rejected connect settles before the delayed HTTP Stop.
  assert.equal(wake(render()), null, 'must not expose Wake while old Stop can still arrive');
  await staleWake();
  assert.equal(instances.length, 1, 'even a stale Wake handler cannot reconnect');
  service.stop.resolve({ success: true, message: 'zero requested' });
  await disconnecting;
  assert.ok(wake(render()));
  const retry = wake(render()).props.onClick();
  assert.equal(instances.length, 2);
  instances[1].connection.resolve();
  await retry;
});

test('late automatic Stop response cannot overwrite a newer direct Stop receipt', async t => {
  const { render, instances } = await app(t);
  const connecting = wake(render()).props.onClick();
  const service = instances[0]; service.connection.resolve(); await connecting;
  service.callbacks[5]('Automatic Stop pending');
  const stopping = find(render(), n => n.type === 'ExecutionStatus').props.onStop();
  service.stop.resolve({ success: true, message: 'new explicit Stop accepted' });
  await stopping;
  service.callbacks[5]('old automatic response');
  assert.equal(find(render(), n => n.type === 'ExecutionStatus').props.stopMessage, 'new explicit Stop accepted');
});


test('remote-close callback cannot release explicit Disconnect ownership before its Stop settles', async t => {
  const { render, instances } = await app(t);
  const connecting = wake(render()).props.onClick();
  const service = instances[0]; service.connection.resolve(); await connecting;
  service.callbacks[5]('Automatic Stop pending after remote close');
  const disconnecting = find(render(), n => n.props.title === 'Sleep / Disconnect').props.onClick();
  service.callbacks[3](); // Older automatic Stop finished before the explicit Stop.
  assert.equal(wake(render()), null);
  service.stop.resolve({ success: true, message: 'explicit Stop accepted' });
  await disconnecting;
  assert.ok(wake(render()));
});
