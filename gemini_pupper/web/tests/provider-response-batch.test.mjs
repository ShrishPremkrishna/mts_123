import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const bundle=await build({entryPoints:[process.env.BATCH_SERVICE || './services/geminiLive.ts'],bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'offline-sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI{};export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'};"}));}}]});
const {GeminiLiveService}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
globalThis.fetch=async()=>{throw new Error('Network forbidden in provider-response batching tests');};
const deferred=()=>{let resolve;return {promise:new Promise(r=>resolve=r),resolve:v=>resolve(v)}};
const tick=()=>new Promise(r=>setImmediate(r));
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
const packet=(...calls)=>({toolCall:{functionCalls:calls.map(([id,name,args={}])=>({id,name,args}))}});
const flat=sends=>sends.flatMap(send=>[send.functionResponses].flat());
let n=0;
function rig({cache=true}={}) {
 const calls=[],sends=[],audio=[],stops=[],traces=[];
 const s=new GeminiLiveService({modelName:'gemini-3.1-flash-live-preview',voiceName:'Puck',outputPolicy:'conversation',robotApiUrl:'http://batch-'+(++n)},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},undefined,()=>{},cache?{lookup:text=>text}:undefined,()=>{},event=>traces.push(event));
 s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:r=>sends.push(r),sendRealtimeInput(){},close(){}});
 s.robotBridge.callTool=async(name,args)=>{calls.push([name,args]);return name==='robot_status'||name==='stop'?{success:true}:queued;};
 s.robotBridge.immediateStop=async()=>{stops.push('stop');return {success:true};};
 s.playAudioBase64=async x=>audio.push(x);s.synthesizeAndPlaySpeech=async()=>{};
 return {s,calls,sends,audio,stops,traces};
}
test('five-call compound packet sends one response array after ordered HTTP admissions; first cue stays immediate',async()=>{
 const e=rig(),wait=deferred();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='wait'?wait.promise:queued;};
 const work=e.s.handleMessage(packet(['sit','play_trick',{name:'sit_stay_shake'}],['wait','wait',{duration:10}],['left','interactive_action',{action:'shake_left'}],['right','interactive_action',{action:'shake_right'}],['stand','stand_up']));
 await tick();assert.deepEqual(e.calls.map(x=>x[0]),['play_trick','wait']);assert.deepEqual(e.sends,[]);assert.deepEqual(e.audio,['Okay.']);
 wait.resolve(queued);await work;await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['play_trick','wait','interactive_action','interactive_action','stand_up']);
 assert.equal(e.sends.length,1);assert.ok(Array.isArray(e.sends[0].functionResponses));assert.deepEqual(flat(e.sends).map(x=>x.id),['sit','wait','left','right','stand']);
 assert.ok(flat(e.sends).every(x=>x.response.result.success));assert.deepEqual(e.audio,['Okay.']);await e.s.disconnect();
});
test('failed compound member returns its refusal and unsent tail in the same response batch',async()=>{
 const e=rig();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='wait'?{success:false,message:'bad duration'}:queued;};
 await e.s.handleMessage(packet(['sit','play_trick'],['wait','wait'],['paw','interactive_action'],['stand','stand_up']));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['play_trick','wait']);assert.equal(e.sends.length,1);const responses=flat(e.sends);assert.equal(responses[1].response.result.success,false);assert.ok(responses.slice(2).every(x=>x.response.result.not_dispatched));await e.s.disconnect();
});
test('separate Stop preempts unresolved older packet and responds before it settles',async()=>{
 const e=rig(),move=deferred();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='move'?move.promise:name==='stop'?{success:true}:queued;};
 const old=e.s.handleMessage(packet(['move','move'],['turn','turn']));await tick();
 await e.s.handleMessage(packet(['stop','stop']));await tick();assert.deepEqual(e.calls.map(x=>x[0]),['move','stop']);assert.deepEqual(flat(e.sends).map(x=>x.id),['stop']);assert.deepEqual(e.audio,['Stop requested.']);
 move.resolve({success:false,superseded:true,outcome_unknown:true});await old;await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['move','stop']);assert.equal(e.sends.length,2);assert.deepEqual(flat(e.sends).map(x=>x.id),['stop','move','turn']);assert.equal(flat(e.sends)[2].response.result.not_dispatched,true);assert.ok(flat(e.sends).slice(1).every(x=>!x.response.operator_acknowledgment));await e.s.disconnect();
});
test('same-packet Stop response does not wait for its following status call',async()=>{
 const e=rig(),status=deferred();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='robot_status'?status.promise:{success:true};};
 const work=e.s.handleMessage(packet(['move','move'],['stop','stop'],['status','robot_status']));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['stop','robot_status']);assert.deepEqual(flat(e.sends).map(x=>x.id),['stop']);
 status.resolve({success:true});await work;await tick();assert.equal(e.sends.length,2);assert.deepEqual(flat(e.sends).map(x=>x.id),['stop','move','status']);assert.equal(flat(e.sends)[1].response.result.not_dispatched,true);await e.s.disconnect();
});
test('cancelled waiting IDs are omitted while other packet replies remain ordered',async()=>{
 const e=rig(),first=deferred();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='move'?first.promise:queued;};
 const work=e.s.handleMessage(packet(['move','move'],['cancel','turn'],['wait','wait']));
 await e.s.handleMessage({toolCallCancellation:{ids:['cancel']}});first.resolve(queued);await work;await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['move','wait']);assert.equal(e.sends.length,1);assert.deepEqual(flat(e.sends).map(x=>x.id),['move','wait']);await e.s.disconnect();
});
test('cancellation after acceptance but before packet flush removes that collected response',async()=>{
 const e=rig(),second=deferred();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='wait'?second.promise:queued;};
 const work=e.s.handleMessage(packet(['move','move'],['wait','wait']));await tick();assert.deepEqual(e.sends,[]);
 await e.s.handleMessage({toolCallCancellation:{ids:['move']}});assert.deepEqual(e.stops,['stop']);
 second.resolve({success:false,superseded:true,outcome_unknown:true});await work;await tick();assert.deepEqual(flat(e.sends).map(x=>x.id),['wait']);await e.s.disconnect();
});
test('disconnected old packet sends no collected responses and does not dispatch its tail',async()=>{
 const e=rig(),second=deferred();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='wait'?second.promise:queued;};
 const work=e.s.handleMessage(packet(['move','move'],['wait','wait'],['turn','turn']));await tick();await e.s.disconnect();second.resolve(queued);await work;await tick();assert.deepEqual(e.calls.map(x=>x[0]),['move','wait']);assert.deepEqual(e.sends,[]);
});
test('receipt authorization is evaluated at SDK flush, not frozen when HTTP completes',async()=>{
 const e=rig({cache:false}),sdk=deferred();e.s.sessionPromise=sdk.promise;
 await e.s.handleMessage(packet(['stand','stand_up']));assert.deepEqual(e.sends,[]);
 e.s.pauseRobotTools();sdk.resolve({sendToolResponse:r=>e.sends.push(r),close(){}});await tick();assert.equal(e.sends.length,1);assert.equal(flat(e.sends)[0].response.operator_acknowledgment,undefined);await e.s.disconnect();
});
test('SDK-session replacement drops a scheduled old batch without response replay',async()=>{
 const e=rig(),sdk=deferred();e.s.sessionPromise=sdk.promise;
 await e.s.handleMessage(packet(['move','move'],['wait','wait']));e.s.sessionPromise=Promise.resolve({sendToolResponse:r=>e.sends.push(r),close(){}});
 sdk.resolve({sendToolResponse:r=>e.sends.push(r),close(){}});await tick();assert.deepEqual(e.sends,[]);await e.s.disconnect();
});
