import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const source=process.env.RELIABILITY_SERVICE || './services/geminiLive.ts';
const bundle=await build({entryPoints:[source],bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'offline-sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI{};export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'};"}));}}]});
const {GeminiLiveService}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
globalThis.fetch=async()=>{throw new Error('Network forbidden in request-order tests');};
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

const moveArgs={direction:'forward',speed:.2,duration:.5};
const finishTurn=s=>s.handleMessage({serverContent:{generationComplete:true,turnComplete:true}});
test('same-packet duplicate ID creates one job, response and queued cue',async()=>{
 const e=rig(),server='1'.repeat(32);let jobs=0;
 e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return {...queued,job_id:`${server}:${++jobs}`,server_instance:server}};
 await e.s.handleMessage(packet(['same','move',moveArgs],['same','move',moveArgs]));await tick();
 assert.equal(jobs,1);assert.equal(e.calls.length,1);assert.equal(e.responses.length,1);
 assert.equal(e.responses[0].functionResponses.response.result.job_id,`${server}:1`);
 assert.deepEqual(e.audio,['Okay.']);assert.ok(e.logs.some(x=>/Duplicate tool call ignored/.test(x.message)));
});
test('overlapping duplicate waits for the original response without another dispatch or acknowledgment',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return first.promise};
 const a=e.s.handleMessage(packet(['same','move',moveArgs]));const b=e.s.handleMessage(packet(['same','move',moveArgs]));await tick();
 assert.equal(e.calls.length,1);assert.equal(e.responses.length,0);assert.deepEqual(e.audio,[]);
 first.resolve(queued);await Promise.all([a,b]);await tick();
 assert.equal(e.calls.length,1);assert.equal(e.responses.length,1);assert.deepEqual(e.audio,['Okay.']);
});
test('a call ID stays consumed after turn completion, without a fresh response or cue',async()=>{
 const e=rig();await e.s.handleMessage(packet(['same','move',moveArgs]));await finishTurn(e.s);
 await e.s.handleMessage(packet(['same','move',moveArgs]));await finishTurn(e.s);await tick();
 assert.equal(e.calls.length,1);assert.equal(e.responses.length,1);assert.deepEqual(e.audio,['Okay.']);
});
test('reordered mixed duplicate and fresh IDs dispatch only the fresh work in arrival order',async()=>{
 const e=rig();await e.s.handleMessage(packet(['first','move',moveArgs],['second','wait',{duration:1}]));await finishTurn(e.s);
 await e.s.handleMessage(packet(['second','wait',{duration:1}],['third','turn',{direction:'left',angle:15}],['first','move',moveArgs],['fourth','move',moveArgs]));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['move','wait','turn','move']);
 assert.deepEqual(e.responses.map(x=>x.functionResponses.id),['first','second','third','fourth']);
});
test('a changed tool or arguments cannot reuse an ordinary ID, including a prior status ID',async()=>{
 const e=rig();await e.s.handleMessage(packet(['same','robot_status']));await finishTurn(e.s);
 await e.s.handleMessage(packet(['same','move',moveArgs],['same','turn',{direction:'left',angle:15}]));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['robot_status']);assert.equal(e.responses.length,1);assert.deepEqual(e.audio,[]);
});
test('distinct IDs with identical arguments remain intentional separate actions',async()=>{
 const e=rig();await e.s.handleMessage(packet(['one','move',moveArgs],['two','move',moveArgs]));await tick();
 assert.deepEqual(e.calls,[['move',moveArgs],['move',moveArgs]]);assert.equal(e.responses.length,2);
});
test('Stop may reuse an in-flight action ID once and still bypasses the ordinary lane',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='move'?first.promise:{success:true}};
 const a=e.s.handleMessage(packet(['same','move',moveArgs]));
 await e.s.handleMessage(packet(['same','stop']));assert.deepEqual(e.calls.map(x=>x[0]),['move','stop']);
 first.resolve({...queued,superseded:true});await a;await e.s.handleMessage(packet(['same','stop'],['same','turn']));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['move','stop']);assert.equal(e.responses.length,2);assert.deepEqual(e.audio,['Stop requested.']);
});
test('same-packet Stop wins a colliding action ID regardless of call order',async()=>{
 for(const calls of [[['same','move'],['same','stop']],[['same','stop'],['same','move']]]){
  const e=rig();await e.s.handleMessage(packet(...calls));await tick();assert.deepEqual(e.calls.map(x=>x[0]),['stop']);
  await finishTurn(e.s);await e.s.handleMessage(packet(['same','move']));assert.deepEqual(e.calls.map(x=>x[0]),['stop']);
 }
});
test('repeated Stop sends once but conservatively retains precedence over fresh packet actions',async()=>{
 const e=rig();await e.s.handleMessage(packet(['stop','stop']));await finishTurn(e.s);
 await e.s.handleMessage(packet(['fresh','move'],['stop','stop']));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['stop']);
 assert.equal(e.responses.filter(x=>x.functionResponses.id==='stop').length,1);
 assert.equal(e.responses.find(x=>x.functionResponses.id==='fresh').functionResponses.response.result.not_dispatched,true);
});
test('a refused same-packet action ID cannot be replayed after Stop completes',async()=>{
 const e=rig();await e.s.handleMessage(packet(['refused','move'],['stop','stop']));await finishTurn(e.s);
 await e.s.handleMessage(packet(['refused','move']));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['stop']);assert.equal(e.responses.filter(x=>x.functionResponses.id==='refused').length,1);
});
test('an expired waiting ID is consumed even though it never reached HTTP',async()=>{
 const e=rig(),first=defer();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='wait'?first.promise:{success:true}};
 const a=e.s.handleMessage(packet(['first','wait']));const b=e.s.handleMessage(packet(['expired','move']));
 await e.s.handleMessage(packet(['stop','stop']));first.resolve({...queued,superseded:true});await Promise.all([a,b]);await finishTurn(e.s);
 await e.s.handleMessage(packet(['expired','move']));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['wait','stop']);assert.equal(e.responses.filter(x=>x.functionResponses.id==='expired').length,1);
});
test('cancelled call IDs stay closed across turn boundaries while a fresh ID is usable',async()=>{
 const e=rig();await e.s.handleMessage({toolCallCancellation:{ids:['cancelled']}});
 await e.s.handleMessage(packet(['cancelled','move']));await finishTurn(e.s);
 await e.s.handleMessage(packet(['cancelled','move'],['fresh','move']));await tick();
 assert.deepEqual(e.calls.map(x=>x[0]),['move']);assert.deepEqual(e.responses.map(x=>x.functionResponses.id),['fresh']);
});
for(const [kind,outcome] of [['failure',{success:false,message:'Refused'}],['unknown',{success:false,outcome_unknown:true}]]){
 test(`${kind} action results retain their ID without a duplicate response and keep Stop available`,async()=>{
  const e=rig();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return name==='stop'?{success:true}:outcome};
  await e.s.handleMessage(packet(['same','move']));await finishTurn(e.s);await e.s.handleMessage(packet(['same','move']));await tick();
  assert.deepEqual(e.calls.map(x=>x[0]),['move']);assert.equal(e.responses.length,1);assert.equal(e.s.robotToolsPaused,true);
  assert.deepEqual(e.responses[0].functionResponses.response.result,outcome);assert.ok(!e.audio.includes('Okay.'));
  await e.s.handleMessage(packet(['fresh-stop','stop']));assert.deepEqual(e.calls.map(x=>x[0]),['move','stop']);
 });
 test(`${kind} Stop is not retried by duplicate ID, but a fresh Stop ID remains usable`,async()=>{
  const e=rig();e.s.robotBridge.callTool=async(name,args)=>{e.calls.push([name,args]);return outcome};
  await e.s.handleMessage(packet(['same','stop']));await finishTurn(e.s);
  await e.s.handleMessage(packet(['same','stop']));await e.s.handleMessage(packet(['fresh','stop']));await tick();
  assert.deepEqual(e.calls.map(x=>x[0]),['stop','stop']);assert.deepEqual(e.responses.map(x=>x.functionResponses.id),['same','fresh']);
 });
}
test('a lost SDK response never authorizes duplicate HTTP execution',async()=>{
 const e=rig();let sends=0;e.s.sessionPromise=Promise.resolve({sendToolResponse(){sends++;throw new Error('Closed');},close(){}});
 await e.s.handleMessage(packet(['same','move']));await tick();await finishTurn(e.s);await e.s.handleMessage(packet(['same','move']));await tick();
 assert.equal(e.calls.length,1);assert.equal(sends,1);assert.deepEqual(e.audio,['Okay.']);
});
test('fresh service lifetime permits an ID used by a disconnected service',async()=>{
 const old=rig();await old.s.handleMessage(packet(['same','move',moveArgs]));await old.s.disconnect();
 const fresh=rig();await fresh.s.handleMessage(packet(['same','move',moveArgs]));await tick();
 assert.equal(old.calls.length,1);assert.equal(fresh.calls.length,1);assert.equal(fresh.responses.length,1);assert.deepEqual(fresh.audio,['Okay.']);
});
