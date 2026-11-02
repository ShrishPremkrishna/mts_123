import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {build} from 'esbuild';
const b=await build({entryPoints:['services/geminiLive.ts'],bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'sdk',setup(x){x.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));x.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI {} export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {GeminiLiveService}=await import('data:text/javascript;base64,'+Buffer.from(b.outputFiles[0].text).toString('base64'));
const fixture=JSON.parse(readFileSync(new URL('./fixtures/action-failure-http.json',import.meta.url)));
const failed=fixture.responses[0].body,queued=fixture.responses[1].body;
const call=(id,name,args={})=>({id,name,args});
const packet=(...calls)=>({toolCall:{functionCalls:calls}});
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {resolve,promise};};
let seq=0;
function service(){const calls=[],responses=[],logs=[],audio=[],stops=[];
 const s=new GeminiLiveService({modelName:'gemini-3.1-flash-live-preview',voiceName:'Puck',outputPolicy:'conversation',robotApiUrl:'http://action-failure-'+(++seq)},e=>logs.push(e),()=>{},()=>{});
 s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:r=>responses.push(r),sendRealtimeInput(){}});s.playAudioBase64=async x=>audio.push(x);
 s.robotBridge.callTool=async name=>{calls.push(name);return name==='move'?failed:name==='robot_status'||name==='stop'?{success:true}:queued;};
 s.robotBridge.immediateStop=async()=>{stops.push('stop');return {success:true};};return {s,calls,responses,logs,audio,stops};}
test('actual HTTP failed move without epoch flags fences following turn and later packets',async()=>{
 assert.equal(failed.command_epoch_required,undefined);const e=service();
 await e.s.handleMessage(packet(call('m','move',fixture.responses[0].args),call('t','turn',fixture.responses[1].args)));
 assert.deepEqual(e.calls,['move']);assert.equal(e.s.robotToolsPaused,true);assert.equal(e.responses[1].functionResponses.response.result.not_dispatched,true);
 await e.s.handleMessage(packet(call('retry','turn',fixture.responses[1].args)));assert.deepEqual(e.calls,['move']);
 await e.s.handleMessage(packet(call('status','robot_status')));await e.s.handleMessage(packet(call('stop','stop')));assert.deepEqual(e.calls,['move','robot_status','stop']);
 assert.ok(e.logs.some(x=>/paused.*Reconnect/i.test(x.message)));assert.equal(e.stops.length,0,'no other pending action to fence');
});
test('all failed motion/action tools fence; known status and gate refusals do not',async()=>{
 for(const name of ['stand_up','lie_down','move','turn','arc','wait','play_trick','interactive_action','follow']){const e=service();e.s.robotBridge.callTool=async n=>{e.calls.push(n);return {success:false,message:'known refusal'};};await e.s.handleMessage(packet(call('bad',name),call('later','move')));assert.deepEqual(e.calls,[name],name);}
 for(const name of ['robot_status','unlock','lock_tricks']){const e=service();e.s.robotBridge.callTool=async n=>{e.calls.push(n);return n===name?{success:false}:queued;};await e.s.handleMessage(packet(call('benign',name),call('motion','move')));assert.deepEqual(e.calls,[name,'move'],name);assert.equal(e.s.robotToolsPaused,false);}
});
test('current known failure retains exact fixed refusal audio, not a false success',async()=>{
 const e=service();await e.s.handleMessage(packet(call('m','move')));assert.equal(e.s.robotToolsPaused,true);
 const text=e.responses[0].functionResponses.response.operator_acknowledgment?.text;assert.ok(text?.startsWith('Move rejected:'),text);
 await e.s.handleMessage({serverContent:{outputTranscription:{text},modelTurn:{parts:[{inlineData:{data:'fixed-failure'}}]},turnComplete:true}});assert.deepEqual(e.audio,['fixed-failure']);
});
test('failure fences later waiting packets before HTTP without an unnecessary global Stop',async()=>{
 const e=service(),m=deferred();e.s.robotBridge.callTool=name=>{e.calls.push(name);return name==='move'?m.promise:Promise.resolve(queued);};
 const first=e.s.handleMessage(packet(call('m','move')));const second=e.s.handleMessage(packet(call('t','turn'),call('later','arc')));
 m.resolve(failed);await Promise.all([first,second]);assert.equal(e.s.robotToolsPaused,true);assert.deepEqual(e.stops,[]);
 assert.deepEqual(e.calls,['move']);assert.equal(e.responses.at(-1).functionResponses.response.result.not_dispatched,true);
 assert.deepEqual(e.audio,[],'waiting action cannot claim acceptance');
});
test('pending status alone does not trigger global Stop after action refusal',async()=>{
 const e=service(),m=deferred(),status=deferred();e.s.robotBridge.callTool=n=>n==='move'?m.promise:status.promise;
 const action=e.s.handleMessage(packet(call('m','move'))),read=e.s.handleMessage(packet(call('s','robot_status')));m.resolve(failed);await action;assert.deepEqual(e.stops,[]);status.resolve({success:true});await read;
});
test('real RobotBridge never POSTs a later ordinary packet after an earlier refusal',async t=>{
 const e=service(),moveReply=deferred(),moveReached=deferred();const http=[];
 delete e.s.robotBridge.callTool;delete e.s.robotBridge.immediateStop;
 const reply=x=>new Response(JSON.stringify(x),{headers:{'Content-Type':'application/json'}});
 t.mock.method(globalThis,'fetch',async(url,opts)=>{
  const path=new URL(url).pathname;http.push(path);
  if(path==='/health')return reply({command_epoch_required:true,command_epoch:'fixture:0'});
  if(path==='/tools/move'){moveReached.resolve();await moveReply.promise;return reply(failed);}
  throw new Error('Unexpected request '+path);
 });
 const first=e.s.handleMessage(packet(call('m','move',fixture.responses[0].args)));
 const second=e.s.handleMessage(packet(call('t','turn',fixture.responses[1].args),call('later','arc')));
 await moveReached.promise;assert.deepEqual(http,['/health','/tools/move']);moveReply.resolve();await Promise.all([first,second]);
 assert.deepEqual(http,['/health','/tools/move']);assert.equal(e.s.robotToolsPaused,true);
 const turnResult=e.responses.find(x=>x.functionResponses.id==='t').functionResponses.response.result;
 assert.equal(turnResult.not_dispatched,true);assert.deepEqual(e.audio,[]);
});
test('fresh PTT hold does not silently rearm after definitive failure',async()=>{
 const e=service();await e.s.handleMessage(packet(call('m','move')));
 e.s.config.enableAudio=true;e.s.inputAudioContext={state:'running'};e.s.mediaStream={getAudioTracks:()=>[{enabled:false}]};e.s.beginAudioInput();
 await e.s.handleMessage(packet(call('new','turn')));assert.deepEqual(e.calls,['move']);assert.equal(e.s.robotToolsPaused,true);e.s.endAudioInput();
});
test('unknown or malformed action reply fences later waiting packets even without success:false',async()=>{
 for(const result of [{success:true,outcome_unknown:true},{message:'missing success'},{success:true,outcome_unknown:{}}]){
  const e=service(),m=deferred();e.s.robotBridge.callTool=n=>{e.calls.push(n);return n==='move'?m.promise:Promise.resolve(queued)};
  const first=e.s.handleMessage(packet(call('m','move'))),second=e.s.handleMessage(packet(call('t','turn')));m.resolve(result);await Promise.all([first,second]);
  assert.deepEqual(e.calls,['move']);assert.deepEqual(e.stops,[]);assert.deepEqual(e.audio,[]);assert.equal(e.s.robotToolsPaused,true);
 }
});
test('late action failure from disposed or disconnected old service cannot issue another global Stop',async()=>{
 for(const state of ['disposed','disconnected']){
  const e=service(),m=deferred(),turn=deferred();e.s.robotBridge.callTool=n=>n==='move'?m.promise:turn.promise;
  const first=e.s.handleMessage(packet(call('m','move'))),second=e.s.handleMessage(packet(call('t','turn')));
  if(state==='disposed')e.s.disposed=true;else e.s.connected=false;
  m.resolve(failed);await first;assert.deepEqual(e.stops,[],state);turn.resolve(queued);await second;assert.deepEqual(e.audio,[]);
 }
});

// A known refusal must also update the operator-facing paused state.
test('known refusal notifies the UI while preserving its exact receipt',async()=>{
 const e=service();let notified=0;e.s.onRobotToolsPaused=()=>notified++;
 await e.s.handleMessage(packet(call('m','move')));
 assert.equal(notified,1);assert.equal(e.s.robotToolsPaused,true);
 assert.ok(e.responses[0].functionResponses.response.operator_acknowledgment?.text.startsWith('Move rejected:'));
});
test('a paused-state UI callback cannot break refusal handling',async()=>{
 const e=service();e.s.onRobotToolsPaused=()=>{throw new Error('synthetic render failure')};
 await e.s.handleMessage(packet(call('m','move'),call('t','turn')));
 assert.deepEqual(e.calls,['move']);assert.equal(e.s.robotToolsPaused,true);
});
