import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const source=process.env.RELIABILITY_SERVICE || './services/geminiLive.ts';
const bundle=await build({entryPoints:[source],bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'offline-sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI{};export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'};"}));}}]});
const {GeminiLiveService}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const defer=()=>{let resolve;return {promise:new Promise(r=>resolve=r),resolve:v=>resolve(v)}};
const tick=()=>new Promise(r=>setImmediate(r));
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
const packet=(...calls)=>({toolCall:{functionCalls:calls.map(([id,name,args={}])=>({id,name,args}))}});
function rig(){
 const calls=[],responses=[],audio=[],logs=[];
 const config={modelName:'gemini-robotics-er-2-streaming-preview',robotApiUrl:'http://request-order',outputPolicy:'conversation',voiceName:'Puck',ttsModelName:'gemini-3.1-flash-tts-preview'};
 const s=new GeminiLiveService(config,e=>logs.push(e),()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},{lookup:text=>text});
 s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:x=>responses.push(x),close(){}});
 s.robotBridge.callTool=async(name,args)=>{calls.push([name,args]);return name==='robot_status'||name==='stop'?{success:true}:queued};
 s.robotBridge.immediateStop=async()=>({success:true});s.playAudioBase64=async x=>audio.push(x);s.synthesizeAndPlaySpeech=async()=>{};
 return {s,calls,responses,audio,logs};
}
test('ordinary SDK packets retain arrival order across a delayed first HTTP response',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='play_trick'?first.promise:queued};
 const a=e.s.handleMessage(packet(['sit','play_trick',{name:'sit_stay_shake',count:1,count_unit:'routine_playback'}]));
 const b=e.s.handleMessage(packet(['paw','interactive_action',{action:'shake_right'}]));
 await tick();const before=e.calls.map(x=>x[0]);first.resolve(queued);await Promise.all([a,b]);
 assert.deepEqual(before,['play_trick'],'later paw must not overtake delayed Sit acceptance');
 assert.deepEqual(e.calls.map(x=>x[0]),['play_trick','interactive_action']);
 assert.deepEqual(e.audio,['Okay.']);
});
test('all calls in the earlier packet precede a later ordinary packet',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='play_trick'?first.promise:queued};
 const a=e.s.handleMessage(packet(['sit','play_trick'],['wait','wait']));
 const b=e.s.handleMessage(packet(['paw','interactive_action',{action:'shake_right'}]));await tick();
 first.resolve(queued);await Promise.all([a,b]);assert.deepEqual(e.calls.map(x=>x[0]),['play_trick','wait','interactive_action']);
});
test('later Stop bypasses an unresolved request and invalidates earlier waiting actions',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='move'?first.promise:{success:true}};
 const a=e.s.handleMessage(packet(['move','move']));const b=e.s.handleMessage(packet(['turn','turn']));
 await tick();await e.s.handleMessage(packet(['stop','stop']));const before=e.calls.map(x=>x[0]);
 first.resolve(queued);await Promise.all([a,b]);assert.deepEqual(before,['move','stop']);assert.deepEqual(e.calls.map(x=>x[0]),['move','stop']);
});
test('a failed first request fences waiting ordinary calls without sending them',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='move'?first.promise:queued};
 const a=e.s.handleMessage(packet(['move','move']));const b=e.s.handleMessage(packet(['turn','turn']));await tick();first.resolve({success:false});await Promise.all([a,b]);
 assert.deepEqual(e.calls.map(x=>x[0]),['move']);assert.equal(e.s.robotToolsPaused,true);
});
test('same-packet Stop precedes and refuses the action members even behind a busy ordinary lane',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='wait'?first.promise:{success:true}};
 const a=e.s.handleMessage(packet(['wait','wait']));
 await e.s.handleMessage(packet(['later','move'],['stop','stop'],['end','stand_up']));
 assert.deepEqual(e.calls.map(x=>x[0]),['wait','stop']);first.resolve(queued);await a;await tick();
 const refused=e.responses.map(x=>x.functionResponses).filter(x=>['later','end'].includes(x.id));assert.equal(refused.length,2);assert.ok(refused.every(x=>x.response.result.not_dispatched));
});
test('a cancelled waiting ID is never dispatched while unrelated status can follow',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='move'?first.promise:{success:true}};
 const a=e.s.handleMessage(packet(['move','move']));const b=e.s.handleMessage(packet(['turn','turn']));
 await e.s.handleMessage({toolCallCancellation:{ids:['turn']}});first.resolve(queued);await Promise.all([a,b]);
 await e.s.handleMessage(packet(['status','robot_status']));assert.deepEqual(e.calls.map(x=>x[0]),['move','robot_status']);
});
test('disconnect drops waiting old actions and a fresh service has an independent lane',async()=>{
 const old=rig(),first=defer();old.s.robotBridge.callTool=async(name,args)=>{old.calls.push([name,args]);return first.promise};
 const a=old.s.handleMessage(packet(['old-move','move']));const b=old.s.handleMessage(packet(['old-turn','turn']));
 await old.s.disconnect();const fresh=rig();await fresh.s.handleMessage(packet(['new-move','move']));
 assert.deepEqual(fresh.calls.map(x=>x[0]),['move']);first.resolve(queued);await Promise.all([a,b]);assert.deepEqual(old.calls.map(x=>x[0]),['move']);assert.deepEqual(old.audio,[]);
});
test('failed packet tail stays unsent; a fresh reconnect can accept a new request',async()=>{
 const e=rig();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return {success:false}};
 await e.s.handleMessage(packet(['move','move'],['turn','turn']));await e.s.handleMessage(packet(['next','stand_up']));assert.deepEqual(e.calls.map(x=>x[0]),['move']);
 await e.s.disconnect();const fresh=rig();await fresh.s.handleMessage(packet(['new','stand_up']));assert.deepEqual(fresh.calls.map(x=>x[0]),['stand_up']);
});
test('late expired packet cannot interrupt or narrate over the successful Stop cue',async()=>{
 const e=rig(),first=defer(),generation=[];e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='move'?first.promise:{success:true}};
 e.s.playAudioBase64=async text=>{e.audio.push(text);generation.push(e.s.playbackGeneration)};
 const a=e.s.handleMessage(packet(['move','move']));const b=e.s.handleMessage(packet(['turn','turn']));await e.s.handleMessage(packet(['stop','stop']));
 const afterStop=e.s.playbackGeneration;first.resolve(queued);await Promise.all([a,b]);assert.equal(e.s.playbackGeneration,afterStop);assert.deepEqual(e.audio,['Stop requested.']);
});
test('natural conversation following a completed sequence still uses the existing gate',async()=>{
 const e=rig(),spoken=[];e.s.synthesizeAndPlaySpeech=async text=>spoken.push(text);
 await e.s.handleMessage(packet(['move','move'],['turn','turn']));await e.s.handleMessage({serverContent:{generationComplete:true,turnComplete:true}});
 await e.s.handleMessage({serverContent:{outputTranscription:{text:'Hello there!'},generationComplete:true,turnComplete:true}});await tick();assert.deepEqual(spoken,['Hello there!']);assert.deepEqual(e.audio,['Okay.']);
});
