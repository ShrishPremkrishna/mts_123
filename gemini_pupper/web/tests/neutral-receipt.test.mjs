import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const bundle=await build({stdin:{contents:"export {GeminiLiveService} from './services/geminiLive';",resolveDir:process.cwd(),loader:'ts'},bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'fake-sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI{};export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'};export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {GeminiLiveService}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
const call=(id,name,args={})=>({toolCall:{functionCalls:[{id,name,args}]}});
const end=()=>({serverContent:{generationComplete:true,turnComplete:true}});
const speech=(text,audio='raw')=>({serverContent:{modelTurn:{parts:[{inlineData:{data:audio}}]},outputTranscription:{text},generationComplete:true,turnComplete:true}});
const defer=()=>{let resolve;return {promise:new Promise(r=>resolve=r),resolve:v=>resolve(v)}};
const tick=()=>new Promise(r=>setImmediate(r));
function rig(model='gemini-robotics-er-2-streaming-preview'){
 const audio=[],tts=[],logs=[],responses=[],calls=[],config={modelName:model,voiceName:'Puck',ttsModelName:'gemini-3.1-flash-tts-preview',robotApiUrl:'http://neutral-tests',outputPolicy:'conversation'};
 const cache={lookup:text=>text}; // explicit unit cache fixture, never an audio asset
 const s=new GeminiLiveService(config,e=>logs.push(e),()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},cache);
 s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:x=>responses.push(x)});
 s.playAudioBase64=async x=>audio.push(x);s.synthesizeAndPlaySpeech=async x=>tts.push(x);
 s.robotBridge.callTool=async name=>{calls.push(name);return ['stop','robot_status'].includes(name)?{success:true}:queued};
 s.robotBridge.immediateStop=async()=>({success:true});
 return {s,audio,tts,logs,responses,calls};
}
for(const model of ['gemini-3.1-flash-live-preview','gemini-robotics-er-2-streaming-preview'])test('cross-packet Sit then paw gets one immediate acceptance cue: '+model,async()=>{
 const e=rig(model),first=defer();e.s.robotBridge.callTool=async name=>{e.calls.push(name);return name==='play_trick'?first.promise:queued};
 const pending=e.s.handleMessage(call('entry','play_trick',{name:'sit_stay_shake'}));assert.deepEqual(e.audio,[]);
 first.resolve(queued);await pending;assert.deepEqual(e.audio,['Okay.']);
 await e.s.handleMessage(call('paw','interactive_action',{action:'shake_right'}));assert.deepEqual(e.audio,['Okay.']);
 assert.equal(e.logs.filter(x=>x.type==='receipt').length,2,'typed per-tool details remain visible');
 await e.s.handleMessage(speech('Trick queued. Right paw command queued.','duplicate'));assert.deepEqual(e.audio,['Okay.']);assert.deepEqual(e.tts,[]);
 await e.s.handleMessage(call('new','move'));assert.deepEqual(e.audio,['Okay.','Okay.'],'new completed turn gets its own cue');
});
test('five-tool batch is not truncated and has one queued acceptance cue',async()=>{
 const e=rig();const calls=[['play_trick',{name:'sit_stay_shake'}],['wait',{}],['interactive_action',{action:'shake_left'}],['interactive_action',{action:'shake_right'}],['stand_up',{}]];
 await e.s.handleMessage({toolCall:{functionCalls:calls.map(([name,args],i)=>({id:String(i),name,args}))}});
 assert.equal(e.calls.length,5);assert.deepEqual(e.audio,['Okay.']);assert.equal(e.logs.filter(x=>x.type==='receipt').length,5);
});
test('later definitive failure and Stop remain audible and invalidate pending ordinary actions',async()=>{
 for(const next of ['failure','stop']){
  const e=rig();await e.s.handleMessage(call('m','move'));
  e.s.robotBridge.callTool=async()=>next==='failure'?{success:false}:{success:true};
  await e.s.handleMessage(call('next',next==='failure'?'turn':'stop'));
  assert.deepEqual(e.audio,['Okay.',next==='failure'?'That request failed. Check the controls for details.':'Stop requested.']);
  if(next==='failure')assert.equal(e.s.robotToolsPaused,true);
 }
});
test('unknown/stale/nonqueued/malformed replies never earn a neutral cue; unknown still pauses and displays',async()=>{
 for(const reply of [{success:true},{success:false,outcome_unknown:true},{...queued,superseded:true},{...queued,not_dispatched:true},{...queued,outcome_unknown:{}},{success:true,details:{submission:{schema:'bad',state:'queued'}}}]){
  const e=rig();e.s.robotBridge.callTool=async()=>reply;await e.s.handleMessage(call('m','move'));assert.ok(!e.audio.includes('Okay.'));
  if(reply.outcome_unknown)assert.equal(e.s.robotToolsPaused,true);
  assert.ok(e.logs.some(x=>x.type==='receipt'));
 }
});
test('new hold, cancellation, Stop and disconnect prevent late prior HTTP from cueing',async()=>{
 for(const mode of ['hold','cancel','stop','disconnect']){
  const e=rig(),r=defer();e.s.robotBridge.callTool=()=>r.promise;const pending=e.s.handleMessage(call('old','move'));
  if(mode==='hold'){e.s.config.enableAudio=true;e.s.inputAudioContext={state:'running'};e.s.mediaStream={getAudioTracks:()=>[{enabled:false}],getTracks:()=>[]};assert.equal(e.s.beginAudioInput(),true)}
  if(mode==='cancel')await e.s.handleMessage({toolCallCancellation:{ids:['old']}});
  if(mode==='stop')e.s.pauseRobotTools();
  if(mode==='disconnect')await e.s.disconnect();
  r.resolve(queued);await pending;assert.ok(!e.audio.includes('Okay.'),mode);
 }
});
test('next same-turn accepted packet does not truncate an already admitted neutral cue',async()=>{
 const e=rig(),resume=defer(),starts=[];e.s.playAudioBase64=GeminiLiveService.prototype.playAudioBase64;
 e.s.receiptAudioCache={lookup:()=>Buffer.from([1,0,2,0]).toString('base64')};e.s.analyser={};
 e.s.audioContext={state:'suspended',currentTime:0,resume:()=>resume.promise,createBuffer:()=>({duration:.01,getChannelData:()=>new Float32Array(2)}),createBufferSource:()=>({connect(){},start(){starts.push(true)},stop(){}})};
 await e.s.handleMessage(call('sit','play_trick',{name:'sit_stay_shake'}));await e.s.handleMessage(call('paw','interactive_action',{action:'shake_right'}));
 resume.resolve();await tick();assert.equal(starts.length,1);
});
test('fourth accepted handshake retains its exact eligibility and speaks only its fixed joke after the cue',async()=>{
 const e=rig();e.s.pawClarificationAvailable=true;
 for(let i=1;i<=4;i++){
  await e.s.handleMessage(call('status'+i,'robot_status'));
  if(i===1)await e.s.handleMessage(call('sit','play_trick',{name:'sit_stay_shake'}));
  await e.s.handleMessage(call('paw'+i,'interactive_action',{action:'shake_right'}));
  await e.s.handleMessage(i===4?speech('Right paw command queued. At this rate, I should charge for pawtographs.','duplicate'):end());
 }
 assert.deepEqual(e.audio,['Okay.','Okay.','Okay.','Okay.']);
 assert.deepEqual(e.tts,['At this rate, I should charge for pawtographs.']);
});
test('status-only typed fallback and ordinary natural conversation are unchanged',async()=>{
 const e=rig();e.s.pawClarificationAvailable=true;
 await e.s.handleMessage(call('status','robot_status'));await e.s.handleMessage(end());assert.deepEqual(e.audio,[]);assert.deepEqual(e.tts,['Status request accepted.']);
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{text:'Hello friend!'}]},generationComplete:true,turnComplete:true}});await tick();assert.equal(e.tts.at(-1),'Hello friend!');
});
test('native fourth joke uses the same accepted-chain proof once; a failed fourth paw never jokes',async()=>{
 for(const failFourth of [false,true]){
  const e=rig('gemini-3.1-flash-live-preview');e.s.pawClarificationAvailable=true;
  for(let i=1;i<=4;i++){
   await e.s.handleMessage(call('s'+i,'robot_status'));
   if(failFourth&&i===4)e.s.robotBridge.callTool=async()=>({success:false});
   await e.s.handleMessage(call('p'+i,'interactive_action',{action:'shake_left'}));
   await e.s.handleMessage({serverContent:{generationComplete:true}});
   await e.s.handleMessage({serverContent:{generationComplete:true}});
   await e.s.handleMessage({serverContent:{turnComplete:true}});
  }
  assert.deepEqual(e.tts,failFourth?[]:['At this rate, I should charge for pawtographs.']);
  assert.equal(e.audio.filter(x=>x==='Okay.').length,failFourth?3:4);
  if(failFourth)assert.equal(e.audio.at(-1),'That request failed. Check the controls for details.');
 }
});
test('new PTT after sealed output retains handshake streak but starts a new acceptance cue',async()=>{
 const e=rig();e.s.pawClarificationAvailable=true;e.s.config.enableAudio=true;
 e.s.inputAudioContext={state:'running'};e.s.mediaStream={getAudioTracks:()=>[{enabled:false}],getTracks:()=>[]};
 for(let i=1;i<=4;i++){
  await e.s.handleMessage(call('s'+i,'robot_status'));await e.s.handleMessage(call('p'+i,'interactive_action',{action:'shake_left'}));
  await e.s.handleMessage({serverContent:{generationComplete:true}});
  e.s.audioInputActive=false;assert.equal(e.s.beginAudioInput(),true);
  await e.s.handleMessage({serverContent:{turnComplete:true}});
 }
 assert.equal(e.audio.filter(x=>x==='Okay.').length,4);
 assert.deepEqual(e.tts,['At this rate, I should charge for pawtographs.']);
});
test('known status failure after acceptance interrupts cue playback without pausing action eligibility',async()=>{
 const e=rig();await e.s.handleMessage(call('m','move'));let stopped=0;e.s.queuedSources=[{stop(){stopped++}}];
 e.s.robotBridge.callTool=async()=>({success:false});await e.s.handleMessage(call('s','robot_status'));
 assert.equal(stopped,1);assert.equal(e.s.robotToolsPaused,false);
 assert.deepEqual(e.audio,['Okay.','That request failed. Check the controls for details.']);
});

test('fourth joke waits for complete accepted chain and is revoked by a later extra action',async()=>{
 const e=rig();e.s.pawClarificationAvailable=true;
 for(let i=1;i<=3;i++){await e.s.handleMessage(call('s'+i,'robot_status'));await e.s.handleMessage(call('p'+i,'interactive_action',{action:'shake_left'}));await e.s.handleMessage(end());}
 await e.s.handleMessage(call('s4','robot_status'));await e.s.handleMessage(call('p4','interactive_action',{action:'shake_left'}));
 assert.deepEqual(e.tts,[],'not before final generation boundary');
 await e.s.handleMessage(call('extra','stand_up'));await e.s.handleMessage(end());
 assert.deepEqual(e.tts,[],'mixed turn is not an accepted handshake-only chain');
 assert.equal(e.audio.filter(x=>x==='Okay.').length,4);
});
