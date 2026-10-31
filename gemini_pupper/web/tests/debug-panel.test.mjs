import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { fileURLToPath } from 'node:url';

// Actual component effects/handlers; inert hooks, no DOM/network/ROS/media.
const bundle = await build({
  entryPoints: [fileURLToPath(new URL('../components/DebugPanel.tsx', import.meta.url))],
  bundle: true, write: false, format: 'esm', platform: 'node', jsx: 'transform',
  tsconfigRaw: { compilerOptions: { jsx: 'react' } },
  plugins: [{ name: 'hooks-only', setup(b) {
    b.onResolve({ filter: /^react$/ }, () => ({ path: 'react', namespace: 'stub' }));
    b.onLoad({ filter: /.*/, namespace: 'stub' }, () => ({ contents:
      `const h=globalThis.__panelHooks; export const {useState,useEffect,useRef}=h; export default {createElement:(type,props,...children)=>({type,props:{...props,children}})};` }));
  }}],
});
function deferred() { let resolve; const promise = new Promise(r => { resolve = r; }); return { promise, resolve }; }
function nodes(tree) {
  if (!tree || typeof tree !== 'object') return [];
  return [tree, ...(tree.props?.children?.flat(Infinity) ?? []).flatMap(nodes)];
}
function text(tree) {
  if (typeof tree === 'string' || typeof tree === 'number') return String(tree);
  return (tree?.props?.children?.flat(Infinity) ?? []).map(text).join(' ');
}
const button = (tree, label) => nodes(tree).find(n => n.type === 'button'
  && (n.props['aria-label'] === label || text(n).trim() === label));
const skill = { description: 'Sit and stay until another action is requested.' };
const action = { trick: 'sit_stay_shake', description: 'Shake left paw then stay seated.' };
async function flush() { for (let i = 0; i < 12; i++) await Promise.resolve(); }
async function panel(t, bridge, stop = async () => {}) {
  const cells = [], effects = [], restorers = [], timers = new Map(); let cursor = 0, timerId = 0;
  const setGlobal = (name, value) => {
    const old = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, writable: true, value });
    restorers.push(() => old ? Object.defineProperty(globalThis, name, old) : delete globalThis[name]);
  };
  const state = initial => {
    const i = cursor++;
    if (!(i in cells)) cells[i] = typeof initial === 'function' ? initial() : initial;
    return [cells[i], value => { cells[i] = typeof value === 'function' ? value(cells[i]) : value; }];
  };
  setGlobal('__panelHooks', {
    useState: state, useRef: value => state(() => ({ current: value }))[0],
    useEffect(fn, deps) {
      const i = cursor++, prior = cells[i];
      if (!prior || deps.some((d, j) => !Object.is(d, prior.deps[j]))) {
        effects.push(() => { prior?.cleanup?.(); cells[i] = { deps, cleanup: fn() }; });
      }
    },
  });
  setGlobal('setTimeout', (fn, ms) => { timers.set(++timerId, { fn, ms }); return timerId; });
  setGlobal('clearTimeout', id => timers.delete(id));
  setGlobal('window', { removeEventListener() {}, location: { origin: 'http://fake' } });
  const { default: DebugPanel } = await import('data:text/javascript;base64,'
    + Buffer.from(bundle.outputFiles[0].text).toString('base64') + '#' + Math.random());
  const props = { logs: [], videoElement: null, robotBridge: bridge, systemStatus: null,
    onStatusUpdate() {}, cameraSource: 'disabled', onStop: stop, stopPending: false, isStopPending: () => false };
  const render = () => { cursor = 0; const tree = DebugPanel(props); effects.splice(0).forEach(fn => fn()); return tree; };
  const unmount = () => cells.forEach(c => c?.cleanup?.());
  t.after(() => { unmount(); restorers.reverse().forEach(restore => restore()); }); render();
  return { render, unmount, props, timers, poll: async () => {
    const [id, timer] = [...timers][0]; timers.delete(id); timer.fn(); await flush();
  } };
}
function bridge(overrides = {}) {
  return { getSystemStatus: async () => null, getAnimations: async () => ({}),
    getInteractiveActions: async () => ({}), ...overrides };
}
test('unavailable/malformed catalog never advertises built-in skills; empty profile is distinct', async t => {
  let response = { success: false, message: 'offline' };
  const p = await panel(t, bridge({ getAnimations: async () => response }));
  assert.equal(button(p.render(), 'Play twerk'), undefined);
  await flush(); assert.match(text(p.render()), /Skill catalog unavailable/);
  for (const invalid of [[], { bad: null }, { bad: { description: 7 } }]) {
    response = invalid; await p.poll(); assert.match(text(p.render()), /Skill catalog unavailable/);
    assert.equal(nodes(p.render()).filter(n => n.props?.['aria-label']?.startsWith('Play ')).length, 0);
  }
  response = {}; await p.poll(); assert.match(text(p.render()), /No skills available in this profile/);
});
test('poll refreshes selected skills/actions after external unlock and relock', async t => {
  let unlocked = false;
  const p = await panel(t, bridge({
    getAnimations: async () => ({ sit_stay_shake: skill, ...(unlocked ? { secret_dance: skill } : {}) }),
    getInteractiveActions: async () => ({ shake_left: action }),
  }));
  await flush(); assert.ok(button(p.render(), 'Play sit stay shake'));
  assert.ok(button(p.render(), 'Action shake left')); assert.equal(button(p.render(), 'Play secret dance'), undefined);
  unlocked = true; await p.poll(); assert.ok(button(p.render(), 'Play secret dance'));
  unlocked = false; await p.poll(); assert.equal(button(p.render(), 'Play secret dance'), undefined);
});
test('skill/action receipts distinguish acceptance, rejection and unknown without claiming completion', async t => {
  const sent = []; let response = { success: true, message: 'trick queued' };
  const p = await panel(t, bridge({
    getAnimations: async () => ({ sit_stay_shake: skill }), getInteractiveActions: async () => ({ shake_left: action }),
    queueAnimation: async id => { sent.push(id); return response; },
    interactiveAction: async id => { sent.push(id); return response; },
  }));
  await flush(); button(p.render(), 'Play sit stay shake').props.onClick(); await flush(); p.render(); await flush();
  assert.match(text(p.render()), /accepted.*trick queued/); assert.doesNotMatch(text(p.render()), /completed/);
  response = { success: false, message: 'play the skill first' };
  button(p.render(), 'Action shake left').props.onClick(); await flush(); p.render(); await flush();
  assert.match(text(p.render()), /rejected.*play the skill first/);
  response = { success: false, outcome_unknown: true, message: 'response unavailable' };
  button(p.render(), 'Play sit stay shake').props.onClick(); await flush(); p.render(); await flush();
  assert.match(text(p.render()), /unknown.*response unavailable/);
  assert.deepEqual(sent, ['sit_stay_shake', 'shake_left', 'sit_stay_shake']);
});
test('Stop stays independent while a skill request is pending and hides unlocked entries', async t => {
  const command = deferred(), stop = deferred(); let stops = 0, unlocked = true;
  const p = await panel(t, bridge({
    getAnimations: async () => unlocked ? { secret_dance: skill } : {}, queueAnimation: () => command.promise,
  }), () => { stops++; return stop.promise; });
  await flush(); button(p.render(), 'Play secret dance').props.onClick(); await flush();
  assert.equal(button(p.render(), 'Play secret dance').props.disabled, true);
  assert.notEqual(button(p.render(), 'STOP').props.disabled, true);
  button(p.render(), 'STOP').props.onClick(); assert.equal(stops, 1);
  assert.equal(button(p.render(), 'Play secret dance'), undefined);
  await p.poll(); assert.equal(button(p.render(), 'Play secret dance'), undefined, 'unlocked response cannot reappear during Stop');
  unlocked = false; stop.resolve(); await flush(); p.render(); await flush();
  assert.equal(button(p.render(), 'Play secret dance'), undefined);
  command.resolve({ success: true, message: 'accepted before Stop' }); await flush();
});
test('polls never overlap and old bridge responses are discarded after replacement/unmount', async t => {
  const old = deferred(); let calls = 0;
  const p = await panel(t, bridge({ getAnimations: () => { calls++; return old.promise; } }));
  await flush(); assert.equal(calls, 1); assert.equal(p.timers.size, 0);
  p.props.robotBridge = bridge(); p.render(); await flush(); assert.match(text(p.render()), /No skills available/);
  old.resolve({ obsolete_skill: skill }); await flush(); assert.equal(button(p.render(), 'Play obsolete skill'), undefined);
  assert.equal(p.timers.size, 1); p.unmount(); assert.equal(p.timers.size, 0);
});


test('a pre-Stop catalog response cannot resurrect a secret after Stop completes', async t => {
  const old = deferred(); let requests = 0, stops = 0;
  const p = await panel(t, bridge({ getAnimations: () => ++requests === 1
    ? Promise.resolve({ secret_dance: skill }) : old.promise }), async () => { stops++; });
  await flush(); assert.ok(button(p.render(), 'Play secret dance'));
  const poll = p.poll(); // second request remains in flight
  button(p.render(), 'STOP').props.onClick(); await flush();
  old.resolve({ secret_dance: skill }); await poll; await flush();
  assert.equal(stops, 1);
  assert.equal(button(p.render(), 'Play secret dance'), undefined);
});


test('pending panel Stop blocks even a stale manual handler and unlocks after its response', async t => {
  const stop = deferred(); let moves = 0;
  const p = await panel(t, bridge({ queueMove: async () => { moves++; return { success: true, message: 'queued' }; } }), () => stop.promise);
  await flush(); const staleMove = button(p.render(), 'Move forward').props.onClick;
  button(p.render(), 'STOP').props.onClick();
  assert.equal(button(p.render(), 'Move forward').props.disabled, true);
  staleMove(); await flush(); assert.equal(moves, 0);
  stop.resolve(); await flush(); p.render(); await flush();
  assert.notEqual(button(p.render(), 'Move forward').props.disabled, true);
  button(p.render(), 'Move forward').props.onClick(); await flush(); assert.equal(moves, 1);
});

test('shared header/automatic Stop gate blocks manual dispatch before rerender', async t => {
  let pending = false, moves = 0;
  const p = await panel(t, bridge({ queueMove: async () => { moves++; return { success: true, message: 'queued' }; } }));
  p.props.isStopPending = () => pending; await flush();
  const staleMove = button(p.render(), 'Move forward').props.onClick;
  pending = true; staleMove(); await flush(); assert.equal(moves, 0);
  p.props.stopPending = true;
  assert.equal(button(p.render(), 'Move forward').props.disabled, true);
  pending = false; p.props.stopPending = false;
  button(p.render(), 'Move forward').props.onClick(); await flush(); assert.equal(moves, 1);
});

test('old-server manual receipts cannot overwrite a new-server request or unlock its busy state', async t => {
  const old = deferred(), next = deferred(); let oldMoves = 0;
  const p = await panel(t, bridge({ queueMove: () => { oldMoves++; return old.promise; } }));
  await flush(); const oldHandler = button(p.render(), 'Move forward').props.onClick;
  oldHandler(); await flush();
  p.props.robotBridge = bridge({ queueMove: () => next.promise }); p.render(); await flush();
  assert.match(text(p.render()), /No manual command sent on this server/);
  oldHandler(); assert.equal(oldMoves, 1, 'old rendered handler cannot dispatch to previous server');
  button(p.render(), 'Move forward').props.onClick(); await flush();
  old.resolve({ success: true, message: 'OLD SERVER ACCEPTED' }); await flush();
  assert.doesNotMatch(text(p.render()), /OLD SERVER/);
  assert.equal(button(p.render(), 'Move forward').props.disabled, true);
  next.resolve({ success: false, message: 'new server refused' }); await flush();
  assert.match(text(p.render()), /rejected.*new server refused/);
});
