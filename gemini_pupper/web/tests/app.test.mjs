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
async function app(t, options = {}) {
  const cells = [], effects = [], effectRecords = [], instances = [], cleanups=[], stored=[], scrubbed=[]; const events={}; let cursor = 0;
  t.after(()=>cleanups.forEach(fn=>fn()));
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
    useEffect(fn, deps) {
      const [record] = state(() => { const record = { deps: null, cleanup: null, setup: null }; effectRecords.push(record); return record; });
      record.setup = fn;
      const changed = !record.deps || deps.some((v, i) => v !== record.deps[i]);
      if (changed && (options.allEffects || deps.length === 0)) {
        record.deps = deps;
        effects.push(() => { record.cleanup?.(); record.cleanup = fn(); return () => record.cleanup?.(); });
      }
    },
  });
  setGlobal('__appLive', class {
    constructor(...callbacks) {
      this.callbacks = callbacks; this.connection = deferred(); this.stop = deferred();
      this.robotBridge = { immediateStop: () => this.stop.promise }; instances.push(this);
    }
    connect(...args) { this.connectArguments=args; return this.connection.promise; }
    get voiceInputMode() { return this.callbacks[0].voiceInputMode; }
    beginAudioInput() { this.begins=(this.begins??0)+1;this.active=true;return true; }
    endAudioInput() { this.ends=(this.ends??0)+1;this.active=false; }
    pauseRobotTools() {}
    async disconnect() { this.connection.reject(new Error('Voice connection cancelled')); }
  });
  setGlobal('localStorage', { getItem: key => key === 'pupster_api_key' ? 'TEST_ONLY' : options.savedConfig ? JSON.stringify(options.savedConfig) : null, setItem:(key,value)=>stored.push([key,value]) });
  const location={ origin:'http://localhost:8000',hostname:'localhost',pathname:'/',search:'',hash:options.hash??'' };
  setGlobal('window', { location,history:{replaceState:(_state,_title,url)=>{scrubbed.push(url);location.hash='';}},addEventListener:(n,f)=>events[n]=f,removeEventListener:(n)=>delete events[n] });
  if(options.fetch) setGlobal('fetch',options.fetch);
  if(options.allEffects){setGlobal('setInterval',()=>123);setGlobal('clearInterval',()=>{});}
  setGlobal('document', { createElement: () => ({}),visibilityState:'visible',addEventListener:(n,f)=>events[n]=f,removeEventListener:(n)=>delete events[n] });
  const { default: App } = await import('data:text/javascript;base64,' + Buffer.from(bundle.outputFiles[0].text).toString('base64') + '#' + Math.random());
  const render = () => { cursor = 0; return App(); };
  const flushEffects=()=>effects.splice(0).forEach(fn=>{const cleanup=fn();if(cleanup)cleanups.push(cleanup);});
  render(); flushEffects();
  const cleanupEffects=()=>effectRecords.forEach(record=>{record.cleanup?.();record.cleanup=null;});
  const replayEffects=()=>{cleanupEffects();effectRecords.forEach(record=>{if(record.deps)record.cleanup=record.setup();});};
  return { render, instances, events, stored, scrubbed, flushEffects, replayEffects, cleanupEffects };
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
  assert.equal(find(render(), n => n.type === 'ExecutionStatus').props.stopMessage, 'Stop requested.');
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


test('App forwards status reports only to the current service with that exact bridge', async t => {
  const { render, instances } = await app(t);
  const connecting = wake(render()).props.onClick();
  const service = instances[0]; service.connection.resolve(); await connecting;
  const seen = []; let tracked = true;
  service.observeQueueStatus = report => seen.push(report);
  service.hasTrackedJobs = () => tracked;
  const status = find(render(), n => n.type === 'ExecutionStatus');
  assert.equal(status.props.robotBridge, service.robotBridge);
  assert.equal(status.props.observeControllers(), false, 'tracked voice jobs select the cheap snapshot');
  tracked = false;
  assert.equal(status.props.observeControllers(), true);
  const report = { bridge: service.robotBridge, serverUrl: 'http://x', status: {}, pollStartedAt: 1, receivedAt: 2, generation: 0 };
  status.props.onStatusReport(report);
  status.props.onStatusReport({ ...report, bridge: {} });
  assert.deepEqual(seen, [report]);
  const disconnecting = find(render(), n => n.props.title === 'Sleep / Disconnect').props.onClick();
  service.stop.resolve({ success: true, message: 'zero requested' }); await disconnecting;
  status.props.onStatusReport(report);
  assert.equal(seen.length, 1, 'a disconnected service receives nothing');
  assert.equal(status.props.observeControllers(), true);
  const retry = wake(render()).props.onClick();
  const next = instances[1]; next.connection.resolve(); await retry;
  const nextSeen = []; next.observeQueueStatus = r => nextSeen.push(r);
  status.props.onStatusReport(report);
  assert.deepEqual(nextSeen, [], 'an old bridge report never reaches the reconnected service');
});

function controls(render) {
  find(render(), n => n.props['aria-label'] === 'Robot controls').props.onClick();
  return find(render(), n => n.type === 'DebugPanel');
}

test('header Stop shares a synchronous gate with manual controls and releases on response', async t => {
  const { render, instances } = await app(t);
  const connecting = wake(render()).props.onClick();
  const service = instances[0]; service.connection.resolve(); await connecting;
  const panel = controls(render);
  assert.equal(panel.props.isStopPending(), false);
  const stopping = find(render(), n => n.type === 'ExecutionStatus').props.onStop();
  assert.equal(panel.props.isStopPending(), true, 'stale panel callback sees gate before render');
  assert.equal(find(render(), n => n.type === 'DebugPanel').props.stopPending, true);
  service.stop.resolve({ success: true, message: 'zero requested' }); await stopping;
  assert.equal(panel.props.isStopPending(), false);
  assert.equal(find(render(), n => n.type === 'DebugPanel').props.stopPending, false);
});

test('overlapping automatic/direct Stops keep manual gate until both settle, including a stale session', async t => {
  const { render, instances } = await app(t);
  const connecting = wake(render()).props.onClick();
  const service = instances[0]; service.connection.resolve(); await connecting;
  const panel = controls(render);
  service.callbacks[6](true);
  service.callbacks[5]('automatic Stop pending');
  const disconnecting = find(render(), n => n.props.title === 'Sleep / Disconnect').props.onClick();
  service.stop.resolve({ success: true, message: 'explicit Stop accepted' }); await disconnecting;
  assert.equal(panel.props.isStopPending(), true, 'older automatic request is still pending');
  service.callbacks[5]('automatic Stop response after session disconnected');
  service.callbacks[6](false);
  assert.equal(panel.props.isStopPending(), false, 'stale session still releases its own Stop ownership');
});

test('failed direct Stop releases only the temporary pending gate and reports unknown outcome', async t => {
  const { render, instances } = await app(t);
  const connecting = wake(render()).props.onClick();
  const service = instances[0]; service.connection.resolve(); await connecting;
  const panel = controls(render);
  const stopping = find(render(), n => n.type === 'ExecutionStatus').props.onStop();
  service.stop.reject(new Error('offline')); await stopping;
  assert.equal(panel.props.isStopPending(), false);
  assert.match(find(render(), n => n.type === 'ExecutionStatus').props.stopMessage, /No confirmed response/);
});


test('model Stop typed lifecycle gates manual controls independent of skipped voice receipts', async t => {
  const { render, instances } = await app(t);
  const connecting = wake(render()).props.onClick();
  const service = instances[0]; service.connection.resolve(); await connecting;
  const panel = controls(render);
  service.callbacks[6](true); // model Stop sent
  assert.equal(panel.props.isStopPending(), true);
  service.callbacks[3](); // service closed; final UI dispatch receipt will be skipped
  service.callbacks[6](false); // actual HTTP request settled nonetheless
  assert.equal(panel.props.isStopPending(), false);
});


test('App output mode defaults to combined conversation and cannot be changed through connected Settings',async t=>{
 const {render,instances}=await app(t);
 find(render(),n=>n.props.title==='Settings').props.onClick();
 assert.equal(find(render(),n=>n.type==='Settings').props.config.outputPolicy,'conversation');
 assert.equal(find(render(),n=>n.type==='Settings').props.sessionActive,false);
 const connecting=wake(render()).props.onClick();
 assert.equal(find(render(),n=>n.type==='Settings').props.sessionActive,true);
 instances[0].connection.resolve();await connecting;
 assert.equal(find(render(),n=>n.type==='Settings').props.sessionActive,true);
});


function talk(render){return find(render(),n=>n.props['aria-label']==='Hold to talk');}
function pointer(id=1){return {button:0,isPrimary:true,pointerId:id,preventDefault(){},currentTarget:{setPointerCapture(){}}};}
async function connectedApp(t){const e=await app(t);const pending=wake(e.render()).props.onClick();e.instances[0].connection.resolve();await pending;return {...e,s:e.instances[0]};}

test('App hold-to-talk shows actual sending state and releases pointer without reconnecting',async t=>{
 const {render,s,instances}=await connectedApp(t);assert.equal(talk(render).props['aria-pressed'],false);
 talk(render).props.onPointerDown(pointer());assert.equal(talk(render).props['aria-pressed'],true);assert.equal(s.active,true);
 talk(render).props.onPointerUp(pointer());assert.equal(talk(render).props['aria-pressed'],false);assert.equal(s.active,false);assert.equal(instances.length,1);
});

test('App hold ownership ignores key repeats, other pointers and mismatched releases',async t=>{
 const {render,s}=await connectedApp(t);const key=k=>({key:k,repeat:false,preventDefault(){}});
 talk(render).props.onKeyDown(key(' '));talk(render).props.onKeyDown({...key(' '),repeat:true});talk(render).props.onPointerDown(pointer());
 assert.equal(s.begins,1);talk(render).props.onPointerCancel(pointer());talk(render).props.onKeyUp(key('Enter'));assert.equal(s.active,true);
 talk(render).props.onKeyUp(key(' '));assert.equal(s.active,false);
 talk(render).props.onPointerDown(pointer(3));talk(render).props.onPointerUp(pointer(4));assert.equal(s.active,true);
 talk(render).props.onLostPointerCapture(pointer(3));assert.equal(s.active,false);
});

test('App releases mic on cancellation, blur, hidden page, settings and independent Stop',async t=>{
 const {render,s,events}=await connectedApp(t);
 for(const event of ['cancel','buttonblur','windowblur','hidden']) {
  talk(render).props.onPointerDown(pointer());assert.equal(s.active,true);
  if(event==='cancel')talk(render).props.onPointerCancel(pointer());
  if(event==='buttonblur')talk(render).props.onBlur();
  if(event==='windowblur')events.blur();
  if(event==='hidden'){document.visibilityState='hidden';events.visibilitychange();document.visibilityState='visible';}
  assert.equal(s.active,false,event);assert.equal(talk(render).props['aria-pressed'],false,event);
 }
 talk(render).props.onPointerDown(pointer());find(render(),n=>n.props.title==='Settings').props.onClick();assert.equal(s.active,false);
 find(render(),n=>n.type==='Settings').props.onClose();talk(render).props.onPointerDown(pointer());
 const stopping=find(render(),n=>n.type==='ExecutionStatus').props.onStop();assert.equal(s.active,false);
 s.stop.resolve({success:true});await stopping;
});


test('App exposes permission guidance, cancellation, and failure details without opening Debug',async t=>{
 const {render,instances}=await app(t);
 const connecting=wake(render()).props.onClick();
 const waiting=find(render(),n=>n.props.role==='status'&&n.props.children.some(x=>typeof x==='string'&&x.includes('permission')));
 assert.ok(waiting);assert.ok(find(waiting,n=>n.type==='button'&&n.props.children.includes('Cancel connection')));
 instances[0].connection.reject(new Error('Microphone access timed out after 30 seconds. Allow access in site controls, then reconnect.'));
 await connecting;
 assert.match(find(render(),n=>n.props.role==='alert').props.children.join(''),/Microphone access timed out.*reconnect/);
 assert.ok(wake(render()));
 const retry=wake(render()).props.onClick();assert.equal(find(render(),n=>n.props.role==='alert'),null);
 instances[1].connection.resolve();await retry;
 assert.equal(find(render(),n=>n.props.role==='status'&&n.props.children.some(x=>typeof x==='string'&&x.includes('permission'))),null);
});

function connectVoice(render) { return find(render(),n=>n.type==='button'&&n.props['aria-label']==='Connect voice'); }
test('Robot controls exposes explicit voice connection and reconnect without switching panels or replay',async t=>{
 const {render,instances}=await app(t);controls(render);
 assert.equal(instances.length,0,'render does not connect automatically');
 assert.ok(connectVoice(render));assert.equal(connectVoice(render).props.disabled,false);
 const oldHandler=connectVoice(render).props.onClick;const connecting=oldHandler();
 assert.equal(instances.length,1);assert.equal(connectVoice(render),null,'connecting session owns reconnect');
 await oldHandler();assert.equal(instances.length,1,'rapid clicks do not overlap sessions');
 instances[0].connection.resolve();await connecting;assert.equal(connectVoice(render),null);
 instances[0].callbacks[3]();
 assert.ok(find(render(),n=>n.type==='DebugPanel'),'same controls view remains open');
 assert.ok(connectVoice(render));assert.equal(instances.length,1,'remote close does not automatically reconnect');
 const retry=connectVoice(render).props.onClick();assert.equal(instances.length,2);
 assert.notEqual(instances[0],instances[1],'explicit retry creates fresh conversation');
 instances[1].connection.resolve();await retry;
});
test('visible reconnect obeys the synchronous pending Stop gate in Robot controls',async t=>{
 const {render,instances}=await app(t);controls(render);const staleConnect=connectVoice(render).props.onClick;
 const pending=staleConnect();const s=instances[0];s.connection.resolve();await pending;
 s.callbacks[6](true);s.callbacks[3]();
 assert.equal(connectVoice(render).props.disabled,true);await staleConnect();assert.equal(instances.length,1);
 s.callbacks[6](false);assert.equal(connectVoice(render).props.disabled,false);
 const retry=connectVoice(render).props.onClick();instances[1].connection.resolve();await retry;
});
test('connection failure stays actionable and visible in Robot controls',async t=>{
 const {render,instances}=await app(t);controls(render);
 const pending=connectVoice(render).props.onClick();instances[0].connection.reject(new Error('The voice session could not connect.'));
 await pending;assert.ok(connectVoice(render));
 const alert=find(render(),n=>n.props.role==='alert');assert.match(alert.props.children.join(''),/voice session could not connect/);
 assert.ok(find(render(),n=>n.type==='DebugPanel'));
});


test('transport end is visible before Stop settles and explicit reconnect retains old ownership',async t=>{
 const {render,instances}=await app(t); controls(render);
 const staleConnect=connectVoice(render).props.onClick;
 const connecting=staleConnect(); const s=instances[0];s.connection.resolve();await connecting;
 s.callbacks[6](true);
 s.callbacks[11]({kind:'closed',code:1008,reason:'The operation was aborted.'});
 assert.ok(find(render(),n=>n.type==='span'&&n.props.children.includes('RECOVERING')));
 assert.equal(talk(render).props.disabled,true);
 assert.equal(connectVoice(render).props.disabled,true);
 const notice=find(render(),n=>n.props.role==='status'&&n.props['data-voice-recovery']);
 assert.match(notice.props.children.join(''),/Voice disconnected/);
 assert.match(find(render(),n=>n.props.role==='alert').props.children.join(''),/1008.*operation was aborted/);
 await staleConnect();assert.equal(instances.length,1);
 s.callbacks[5]('I could not confirm that request.');s.callbacks[6](false);
 assert.equal(connectVoice(render).props.disabled,true,'ownership is not released solely by Stop counter');
 s.callbacks[3]();
 assert.equal(connectVoice(render).props.disabled,false);
 assert.equal(find(render(),n=>n.type==='ExecutionStatus').props.stopMessage,'I could not confirm that request.');
 const retry=connectVoice(render).props.onClick();assert.equal(instances.length,2);
 s.callbacks[11]({kind:'closed',code:1008,reason:'late old close'});s.callbacks[3]();
 assert.equal(connectVoice(render),null,'old notifications cannot replace new connecting state');
 instances[1].connection.resolve();await retry;
});

test('a pending connect resolution cannot restore Connected after transport ended',async t=>{
 const {render,instances}=await app(t);const connecting=wake(render()).props.onClick();const s=instances[0];
 s.callbacks[11]({kind:'goaway'});s.connection.resolve();await connecting;
 assert.ok(find(render(),n=>n.type==='span'&&n.props.children.includes('RECOVERING')));
 assert.equal(connectVoice(render).props.disabled,true);
 assert.equal(talk(render).props.disabled,true);
 s.callbacks[3]();assert.equal(connectVoice(render).props.disabled,false);
 assert.match(wake(render()).props.title,/fresh voice conversation/i);
});


const onboardResponse = () => ({schema:'pupper_onboard_bootstrap_v1',apiKey:'MEMORY_ONLY_TEST_KEY',autoConnect:true,
 config:{voiceInputMode:'hands_free',robotApiUrl:'',cameraSource:'server',eyeTrackingSource:'server',enableAudio:true,enableVideo:true,
 modelName:'gemini-robotics-er-2-streaming-preview',ttsModelName:'gemini-3.1-flash-tts-preview',voiceName:'Puck',outputPolicy:'conversation',quickReceipts:true}});
const tick = () => new Promise(resolve=>setImmediate(resolve));
test('explicit onboard bootstrap strips nonce, connects once and never persists its key',async t=>{
 let requests=0;
 const e=await app(t,{allEffects:true,hash:'#onboard='+'n'.repeat(43),fetch:async(url,options)=>{
  requests++;assert.equal(window.location.hash,'');assert.equal(url,'/onboard/bootstrap');
  assert.equal(options.cache,'no-store');return new Response(JSON.stringify(onboardResponse()),{status:200});
 }});
 e.render();e.flushEffects();await tick();
 assert.equal(requests,1);assert.equal(e.instances.length,1);assert.deepEqual(e.scrubbed,['/']);
 const s=e.instances[0];assert.equal(s.connectArguments[0],'MEMORY_ONLY_TEST_KEY');
 assert.equal(s.connectArguments[1].voiceInputMode,'hands_free');
 assert.equal(s.connectArguments[1].modelName,'gemini-robotics-er-2-streaming-preview');
 s.connection.resolve();await tick();e.render();e.flushEffects();
 assert.equal(find(e.render(),n=>n.props['aria-label']==='Hold to talk'),null);
 assert.ok(find(e.render(),n=>n.props['data-hands-free']!==undefined));
 assert.equal(e.stored.some(([key,value])=>key==='pupster_api_key'||value.includes('MEMORY_ONLY_TEST_KEY')),false);
 // A lost provider connection can expose explicit Connect, never replay bootstrap.
 s.callbacks[3]();e.render();e.flushEffects();await tick();
 assert.equal(requests,1);assert.equal(e.instances.length,1);
});
test('Stop while one-use bootstrap is pending prevents delayed automatic Connect',async t=>{
 const response=deferred();let requests=0;
 const e=await app(t,{allEffects:true,hash:'#onboard='+'n'.repeat(43),fetch:async()=>{requests++;return response.promise;}});
 e.render();e.flushEffects();
 // No service exists; use the nonmoving pending-setup cancellation route.
 // Direct Stop uses an independent bridge; intercept only that fake request.
 globalThis.fetch=async()=>new Response(JSON.stringify({success:true}),{status:200});
 await find(e.render(),n=>n.type==='ExecutionStatus').props.onStop();
 response.resolve(new Response(JSON.stringify(onboardResponse()),{status:200}));await tick();e.render();e.flushEffects();
 assert.equal(requests,1);assert.equal(e.instances.length,0);
 assert.equal(e.stored.some(([,v])=>v.includes('MEMORY_ONLY_TEST_KEY')),false);
});
test('failed onboard setup is one-shot and errors never include response key or nonce',async t=>{
 let requests=0;
 const e=await app(t,{allEffects:true,hash:'#onboard='+'n'.repeat(43),fetch:async()=>{requests++;throw Error('MEMORY_ONLY_TEST_KEY '+'n'.repeat(43));}});
 e.render();e.flushEffects();await tick();e.render();e.flushEffects();
 assert.equal(requests,1);assert.equal(e.instances.length,0);
 const alert=find(e.render(),n=>n.props.role==='alert');assert.ok(alert);
 assert.doesNotMatch(alert.props.children.join(''),/MEMORY_ONLY_TEST_KEY|nnnn/);
 assert.equal(wake(e.render()).props.disabled,false);
});
test('saved hands-free selection still requires explicit Connect and exposes no hold control',async t=>{
 const e=await app(t,{savedConfig:{voiceInputMode:'hands_free'}});
 assert.equal(e.instances.length,0);assert.equal(talk(e.render),null);
 const connecting=wake(e.render()).props.onClick();const s=e.instances[0];s.connection.resolve();await connecting;
 s.callbacks[8](true);assert.match(find(e.render(),n=>n.props['data-hands-free']!==undefined).props.children.join(''),/Hands-free listening/);
 e.events.blur();assert.equal(s.active,false);
 assert.ok(find(e.render(),n=>n.props['aria-label']==='Disconnect disarmed voice'));
 assert.equal(e.instances.length,1);
});


test('pending onboard bootstrap survives effect cleanup/replay and video replacement with one grant',async t=>{
 const response=deferred();let requests=0,signal;
 const e=await app(t,{allEffects:true,hash:'#onboard='+'n'.repeat(43),fetch:async(_url,options)=>{requests++;signal=options.signal;return response.promise;}});
 e.render();e.flushEffects();assert.equal(requests,1);
 // Equivalent setup→cleanup→setup ordering used by StrictMode, after the
 // video exists. The repeated video effect also replaces its element.
 e.replayEffects();e.render();e.flushEffects();await tick();
 assert.equal(signal.aborted,false);assert.equal(requests,1);
 response.resolve(new Response(JSON.stringify(onboardResponse()),{status:200}));await tick();
 assert.equal(e.instances.length,1);assert.equal(requests,1);
 e.instances[0].connection.resolve();await tick();e.render();e.flushEffects();
 assert.equal(e.stored.some(([,v])=>v.includes('MEMORY_ONLY_TEST_KEY')),false);
});
test('actual effect unmount aborts pending onboard grant and ignores late response',async t=>{
 const response=deferred();let signal;
 const e=await app(t,{allEffects:true,hash:'#onboard='+'n'.repeat(43),fetch:async(_url,options)=>{signal=options.signal;return response.promise;}});
 e.render();e.flushEffects();e.cleanupEffects();await tick();assert.equal(signal.aborted,true);
 response.resolve(new Response(JSON.stringify(onboardResponse()),{status:200}));await tick();
 assert.equal(e.instances.length,0);assert.equal(e.stored.some(([,v])=>v.includes('MEMORY_ONLY_TEST_KEY')),false);
});
test('opening Settings cancels pending onboard Connect instead of arming behind the dialog',async t=>{
 const response=deferred();let signal;
 const e=await app(t,{allEffects:true,hash:'#onboard='+'n'.repeat(43),fetch:async(_url,options)=>{signal=options.signal;return response.promise;}});
 e.render();e.flushEffects();find(e.render(),n=>n.props.title==='Settings').props.onClick();
 assert.equal(signal.aborted,true);
 response.resolve(new Response(JSON.stringify(onboardResponse()),{status:200}));await tick();
 assert.equal(e.instances.length,0);
});
