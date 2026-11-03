import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { build } from 'esbuild';
import { fileURLToPath } from 'node:url';

async function load(relative, sdk = false) {
  const result = await build({ entryPoints: [fileURLToPath(new URL(relative, import.meta.url))],
    bundle: true, write: false, format: 'esm', platform: 'node', plugins: sdk ? [{name:'inert-sdk',setup(b){
      b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));
      b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI {} export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));
    }}] : [] });
  return import('data:text/javascript;base64,' + Buffer.from(result.outputFiles[0].text).toString('base64'));
}
const { acceptedJobIdentity, matchJobEvidence, parseQueueStatus } = await load('../services/executionStatus.ts');
const { default: RobotBridge } = await load('../services/robotBridge.ts');
const { GeminiLiveService } = await load('../services/geminiLive.ts', true);
// Exact frozen backend sample: fake ROS / virtual clock, never physical evidence.
const sample = JSON.parse(readFileSync(new URL('./fixtures/job-timeline-sample.json', import.meta.url)));
const status = () => ({...structuredClone(sample.status), recent_outcomes:structuredClone(sample.recent_outcomes)});
const url = 'http://sample-server';
const dispatch = (index=0) => ({id:'model-call',name:'wait',status:'accepted',message:'Wait queued.',timestamp:100,
  job:acceptedJobIdentity(sample.accepted[index]),serverUrl:url});
const source = {serverUrl:url,pollStartedAt:101,fresh:true};
const match = (d=dispatch(), s=status(), extra={}) => matchJobEvidence(d,s,{...source,...extra});
const deferred = () => {let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};};

test('only clean actual queued acceptance yields server-owned identity', () => {
  assert.deepEqual(acceptedJobIdentity(sample.accepted[0]), sample.accepted[0].details.submission && {
    job_id:sample.accepted[0].details.submission.job_id,server_instance:sample.accepted[0].details.submission.server_instance});
  for (const patch of [{success:false},{outcome_unknown:true},{superseded:true},{not_dispatched:true},{outcome_unknown:{}},{success:'true'}]) {
    assert.equal(acceptedJobIdentity({...sample.accepted[0],...patch}), undefined);
  }
  for(const submission of [{},{...sample.accepted[0].details.submission,state:'completed'},
    {...sample.accepted[0].details.submission,server_instance:'wrong'},
    {...sample.accepted[0].details.submission,job_id:'name-only'}]) {
    assert.equal(acceptedJobIdentity({success:true,details:{submission}}),undefined);
  }
});

test('identity validation rejects trailing line breaks in receipt, current job and history',()=>{
  for(const suffix of ['\n','\r','\r\n']) {
    const accepted=structuredClone(sample.accepted[0]);accepted.details.submission.job_id+=suffix;
    assert.equal(acceptedJobIdentity(accepted),undefined);
    const malformed=status();malformed.recent_outcomes.outcomes[0].job_id+=suffix;
    assert.equal(match(dispatch(),malformed).state,'unknown');
    const current=status();current.current_action={...dispatch().job,job_id:dispatch().job.job_id+suffix,state:'running',execution_started:true};current.running=true;
    assert.equal(match(dispatch(),current).state,'unknown');
    const context=status();context.session_context.server_instance+=suffix;
    assert.equal(match(dispatch(),context).state,'unknown');
  }
});

test('frozen backend exact IDs distinguish succeeded, cancellation after start and before start', () => {
  assert.equal(match().state,'succeeded');
  assert.match(match(dispatch(1)).text,/cancelled\. Physical/);
  assert.match(match(dispatch(2)).text,/cancelled before execution started/);
  assert.doesNotMatch(match().text,/standing|motion completed|whole|sequence/);
});

test('current_action acquired before invocation is distinct from running', () => {
  const s=status(); s.recent_outcomes.outcomes=[];s.running=true;
  s.current_action={...dispatch().job,name:'same name irrelevant',state:'running',execution_started:false};
  assert.equal(match(dispatch(),s).state,'claimed');
  s.current_action.execution_started=true;assert.equal(match(dispatch(),s).state,'running');
  s.running=false;assert.equal(match(dispatch(),s).state,'unknown');
});

test('no name-based or last_outcome fallback, absent/evicted history stays unknown', () => {
  const s=status();s.recent_outcomes.outcomes=[];s.last_outcome={...sample.recent_outcomes.outcomes[0]};
  assert.equal(match(dispatch(),s).state,'unknown');
  s.recent_outcomes.dropped=65;assert.match(match(dispatch(),s).text,/evicted/);
  s.recent_outcomes.outcomes=[{...sample.recent_outcomes.outcomes[0],job_id:dispatch(1).job.job_id}];
  assert.equal(match(dispatch(),s).state,'unknown');
  assert.equal(match({...dispatch(),status:'unknown'},status()).state,'unknown');
});

test('restart, URL switch, old poll and expired report do not match', () => {
  const s=status();s.session_context.server_instance='a'.repeat(32);
  assert.match(match(dispatch(),s).text,/restarted or changed/);
  assert.equal(match(dispatch(),status(),{serverUrl:'http://other'}).state,'unknown');
  assert.equal(match(dispatch(),status(),{pollStartedAt:99}).state,'unknown');
  assert.equal(match(dispatch(),status(),{fresh:false}).state,'unknown');
});

test('legacy or malformed optional evidence leaves ordinary queue display but no match', () => {
  for(const patch of [{recent_outcomes:undefined},{current_action:undefined},{session_context:{}},
    {recent_outcomes:{capacity:64,dropped:0,outcomes:[{}]}},
    {recent_outcomes:{...sample.recent_outcomes,outcomes:[sample.recent_outcomes.outcomes[0],sample.recent_outcomes.outcomes[0]]}},
    {recent_outcomes:{...sample.recent_outcomes,outcomes:[{...sample.recent_outcomes.outcomes[0],success:false}]}},
    {recent_outcomes:{...sample.recent_outcomes,outcomes:[{...sample.recent_outcomes.outcomes[0],execution_started:false}]}},
    {command_epoch:'other:0'}]) {
    const s={...status(),...patch};assert.ok(parseQueueStatus(s));assert.equal(match(dispatch(),s).state,'unknown');
  }
  assert.equal(parseQueueStatus({...status(),outcome_unknown:true}),null);
});

test('Stop request alone cannot establish terminal cancellation', () => {
  const s=status();s.recent_outcomes.outcomes=[];s.running=true;
  s.current_action={...dispatch(1).job,name:'hold',state:'running',execution_started:true};
  s.stop=sample.stop;assert.equal(match(dispatch(1),s).state,'running');
  s.recent_outcomes.outcomes=[{...sample.recent_outcomes.outcomes[2],success:true,terminal_state:'succeeded'}];
  assert.equal(match(dispatch(1),s).state,'succeeded','suppressed cancellation may succeed; terminal owns truth');
});

test('actual service callback forwards accepted IDs only after response', async () => {
  const entries=[];const s=new GeminiLiveService({modelName:'gemini-3.1-flash-live-preview',robotApiUrl:url,outputPolicy:'receipts'},()=>{},()=>{},()=>{},e=>entries.push(e));
  s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse(){}});
  const response=deferred();s.robotBridge.callTool=()=>response.promise;
  const pending=s.handleMessage({toolCall:{functionCalls:[{id:'fc1',name:'wait',args:{duration:.1}}]}});
  assert.equal(entries.length,1);assert.equal(entries[0].job,undefined);
  response.resolve(structuredClone(sample.accepted[0]));await pending;
  assert.deepEqual(entries.at(-1).job,dispatch().job);assert.equal(entries.at(-1).serverUrl,url);
  assert.equal(entries.at(-1).status,'accepted');
});

test('service discards late accepted IDs after model pause', async () => {
  const entries=[];const s=new GeminiLiveService({modelName:'gemini-3.1-flash-live-preview',robotApiUrl:url,outputPolicy:'receipts'},()=>{},()=>{},()=>{},e=>entries.push(e));
  s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse(){}});
  const response=deferred();s.robotBridge.callTool=()=>response.promise;
  const pending=s.handleMessage({toolCall:{functionCalls:[{id:'fc1',name:'wait',args:{}}]}});
  s.pauseRobotTools();response.resolve(structuredClone(sample.accepted[0]));await pending;
  assert.equal(entries.length,1);assert.equal(entries[0].job,undefined);
});

test('real bridge invalidates delayed status across same-URL Stop without adopting epoch', async t => {
  const old=deferred();let gets=0;
  t.mock.method(globalThis,'fetch',async (address,options)=>({ok:true,json:async()=>{
    if(options.method==='GET'){gets++;return gets===1?old.promise:status();}
    return {success:true,command_epoch_required:true,command_epoch:dispatch().job.server_instance+':1'};
  }}));
  const a=new RobotBridge('http://stop-status-bridge'),b=new RobotBridge('http://stop-status-bridge/');
  const before=a.statusGeneration(),pending=a.getQueueStatus();await Promise.resolve();
  await b.immediateStop();assert.ok(a.statusGeneration()>before);
  old.resolve(status());assert.equal(parseQueueStatus(await pending),null);
  assert.ok(parseQueueStatus(await a.getQueueStatus()));
});

test('status begun during pending Stop remains stale even if Stop settles before its reply', async t => {
  const stopReply=deferred(),statusReply=deferred();let gets=0;
  t.mock.method(globalThis,'fetch',async (_address,options)=>({ok:true,json:async()=>{
    if(options.method==='GET')return ++gets===1?statusReply.promise:status();
    return stopReply.promise;
  }}));
  const a=new RobotBridge('http://status-started-during-stop'),b=new RobotBridge('http://status-started-during-stop/');
  const stopping=b.immediateStop();
  const pending=a.getQueueStatus();
  stopReply.resolve({success:true,command_epoch_required:true,command_epoch:dispatch().job.server_instance+':1'});
  await stopping;
  statusReply.resolve(status());
  assert.equal(parseQueueStatus(await pending),null,'pre-settlement status cannot become fresh merely by arriving late');
  assert.ok(parseQueueStatus(await a.getQueueStatus()),'a subsequent status check is usable');
});

const componentBundle = await build({
  entryPoints:[fileURLToPath(new URL('../components/ExecutionStatus.tsx',import.meta.url))],
  bundle:true,write:false,format:'esm',platform:'node',jsx:'transform',tsconfigRaw:{compilerOptions:{jsx:'react'}},
  plugins:[{name:'component-hooks',setup(b){
    b.onResolve({filter:/^react$/},()=>({path:'react',namespace:'hooks'}));
    b.onLoad({filter:/.*/,namespace:'hooks'},()=>({contents:"export const {useState,useEffect,useRef}=globalThis.__jobHooks; export default {createElement:(type,props,...children)=>({type,props:{...props,children}})};"}));
  }}],
});
const treeText = tree => typeof tree==='string'||typeof tree==='number' ? String(tree)
  : (tree?.props?.children?.flat(Infinity)??[]).map(treeText).join(' ');
const nodes = tree => tree&&typeof tree==='object' ? [tree,...(tree.props?.children?.flat(Infinity)??[]).flatMap(nodes)] : [];
async function flush(){for(let i=0;i<12;i++)await Promise.resolve();}
async function rendered(t, initialStatus=status()) {
  const cells=[],effects=[],restore=[],timers=new Map();let cursor=0,serial=0,time=1000;
  const globals=(name,value)=>{const prior=Object.getOwnPropertyDescriptor(globalThis,name);Object.defineProperty(globalThis,name,{configurable:true,writable:true,value});restore.push(()=>prior?Object.defineProperty(globalThis,name,prior):delete globalThis[name]);};
  globals('__jobHooks',{
    useRef(value){const i=cursor++;if(!(i in cells))cells[i]={current:value};return cells[i];},
    useState(initial){const i=cursor++;if(!(i in cells))cells[i]=typeof initial==='function'?initial():initial;return[cells[i],value=>cells[i]=typeof value==='function'?value(cells[i]):value];},
    useEffect(fn,deps){const i=cursor++,prior=cells[i];if(!prior||deps.some((x,j)=>!Object.is(x,prior.deps[j])))effects.push(()=>{prior?.cleanup?.();cells[i]={deps,cleanup:fn()};});},
  });
  for(const [name,kind] of [['setTimeout','timeout'],['setInterval','interval']])globals(name,(fn,ms)=>{timers.set(++serial,{fn,ms,kind});return serial;});
  for(const name of ['clearTimeout','clearInterval'])globals(name,id=>timers.delete(id));
  t.mock.method(Date,'now',()=>time);
  const {default:ExecutionStatus}=await import('data:text/javascript;base64,'+Buffer.from(componentBundle.outputFiles[0].text).toString('base64')+'#'+Math.random());
  const bridge={baseUrl:url,generation:0,statusGeneration(){return this.generation;},getQueueStatus:async()=>initialStatus};
  let stops=0;
  const props={robotBridge:bridge,lastDispatch:dispatch(),modelActionsPaused:false,onStop:async()=>{stops++;bridge.generation++;},stopMessage:''};
  const render=()=>{cursor=0;const tree=ExecutionStatus(props);effects.splice(0).forEach(fn=>fn());return tree;};
  const unmount=()=>cells.forEach(cell=>cell?.cleanup?.());
  t.after(()=>{unmount();restore.reverse().forEach(fn=>fn());});
  render();await flush();
  return {props,bridge,render,timers,unmount,stops:()=>stops,
    advance(ms){time+=ms;for(const timer of timers.values())if(timer.kind==='interval')timer.fn();},
    async poll(){const entry=[...timers].find(([,v])=>v.kind==='timeout');assert.ok(entry);timers.delete(entry[0]);entry[1].fn();await flush();},
  };
}

test('rendered actual component links frozen accepted ID to server result without exposing UUID',async t=>{
  const p=await rendered(t);let text=treeText(p.render());
  assert.match(text,/Latest Gemini request:.*accepted: Wait queued/);
  assert.match(text,/Latest Gemini job:.*Server job succeeded/);
  assert.doesNotMatch(text,new RegExp(dispatch().job.server_instance));
  p.props.lastDispatch=dispatch(2);text=treeText(p.render());assert.match(text,/cancelled before execution started/);
  p.props.lastDispatch={...dispatch(),status:'unknown',job:undefined};
  assert.match(treeText(p.render()),/No confirmed queued-job identity/);
});

test('rendered status freshness expires, Stop invalidates immediately, direct Stop remains available',async t=>{
  const p=await rendered(t);assert.match(treeText(p.render()),/Server job succeeded/);
  p.advance(3000);assert.match(treeText(p.render()),/unavailable or stale/);
  await p.poll();assert.match(treeText(p.render()),/Server job succeeded/);
  const button=nodes(p.render()).find(n=>n.type==='button');assert.equal(button.props.disabled,undefined);
  button.props.onClick();assert.equal(p.stops(),1);assert.match(treeText(p.render()),/unavailable or stale/);
  p.unmount();assert.equal(p.timers.size,0);
});

test('rendered old status cannot match a newer receipt or changed bridge',async t=>{
  const p=await rendered(t);p.props.lastDispatch={...dispatch(),timestamp:1500};
  assert.match(treeText(p.render()),/Waiting for a status check after this receipt/);
  p.advance(1000);await p.poll();assert.match(treeText(p.render()),/Server job succeeded/);
  const pending=deferred();p.bridge.getQueueStatus=()=>pending.promise;const poll=p.poll();
  p.props.robotBridge={baseUrl:'http://different',statusGeneration:()=>0,getQueueStatus:async()=>status()};
  p.render();await flush();pending.resolve(status());await poll;
  assert.match(treeText(p.render()),/different server URL/);
});
