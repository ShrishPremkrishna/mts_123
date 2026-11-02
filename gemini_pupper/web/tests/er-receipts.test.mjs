import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {build} from 'esbuild';
const bundle=await build({stdin:{contents:"export {GeminiLiveService} from './services/geminiLive'; export {ReceiptAudioCache,supportsPuckReceiptConsumer} from './services/receiptAudioCache'; export {supportsQuickReceipts} from './services/receiptAudioPreload';",resolveDir:process.env.ER_SOURCE_ROOT||process.cwd(),loader:'ts'},bundle:true,write:false,platform:'node',format:'esm',plugins:[{name:'offline-sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI{}; export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {GeminiLiveService,ReceiptAudioCache,supportsPuckReceiptConsumer,supportsQuickReceipts}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const manifest=JSON.parse(fs.readFileSync('services/receiptAudioManifest.json'));
const cache=await ReceiptAudioCache.load(manifest,async file=>new Uint8Array(fs.readFileSync('public/receipt-audio/'+file)));
const ER='gemini-robotics-er-2-streaming-preview',TTS='gemini-3.1-flash-tts-preview';
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
const call=(id,name,args={})=>({toolCall:{functionCalls:[{id,name,args}]}});
const end=text=>({serverContent:{outputTranscription:{text},generationComplete:true,turnComplete:true}});
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};};
function setup(ready=cache,patch={}){const audio=[],tts=[],responses=[],calls=[],logs=[],config={modelName:ER,voiceName:'Puck',ttsModelName:TTS,quickReceipts:true,outputPolicy:'conversation',robotApiUrl:'http://er-test',...patch};const s=new GeminiLiveService(config,e=>logs.push(e),()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},ready);s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:r=>responses.push(r),sendRealtimeInput(){}});s.playAudioBase64=async x=>audio.push(x);s.synthesizeAndPlaySpeech=async(text,current=()=>true)=>{tts.push({text,current});};s.robotBridge.callTool=async name=>{calls.push(name);return name==='robot_status'||name==='stop'?{success:true}:queued;};s.robotBridge.immediateStop=async()=>({success:true});return {s,audio,tts,responses,calls,logs,config};}
test('ER consumer uses exact approved asset provenance only with selected Puck and tested TTS',()=>{
 assert.equal(manifest.model,'gemini-3.1-flash-live-preview');
 const config={modelName:ER,voiceName:'Puck',ttsModelName:TTS,quickReceipts:true,outputPolicy:'conversation'};
 assert.equal(supportsQuickReceipts(config),true);assert.equal(supportsPuckReceiptConsumer(ER,'Puck',TTS),true);
 assert.equal(cache.lookup('Move queued.',ER,'Puck',TTS),cache.lookup('Move queued.',manifest.model,'Puck'));
 for(const patch of [{modelName:'gemini-robotics-er-3-preview'},{voiceName:'Kore'},{ttsModelName:'other'},{outputPolicy:'receipts'},{quickReceipts:false}])assert.equal(supportsQuickReceipts({...config,...patch}),false);
 assert.equal(cache.lookup('Move queued.',ER,'Puck','other'),null);
});
test('ER queued cue follows actual reply, mutes model duplicate, and preserves later natural TTS',async()=>{
 const e=setup(),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
 const pending=e.s.handleMessage(call('m','move'));assert.deepEqual(e.audio,[]);d.resolve(queued);await pending;
 assert.deepEqual(e.audio,[cache.lookup('Okay.',ER,'Puck',TTS)]);assert.deepEqual(e.tts,[]);
 await e.s.handleMessage(end('I have completed the move.'));assert.deepEqual(e.tts,[]);
 await e.s.handleMessage(end('Hello, friend!'));assert.equal(e.tts.at(-1).text,'Hello, friend!');await e.s.disconnect();
});
test('ER five-tool chain uses actual typed receipts independently of whole-turn narration',async()=>{
 const e=setup();await e.s.handleMessage({toolCall:{functionCalls:[{id:'sit',name:'play_trick',args:{name:'sit_stay_shake',count:1,count_unit:'routine_playback'}},{id:'wait',name:'wait',args:{duration:10}},{id:'left',name:'interactive_action',args:{action:'shake_left'}},{id:'right',name:'interactive_action',args:{action:'shake_right'}},{id:'stand',name:'stand_up',args:{}}]}});
 assert.equal(e.audio.length,1);assert.equal(e.calls.length,5);await e.s.handleMessage(end('I finished everything.'));assert.deepEqual(e.tts,[]);await e.s.disconnect();
});
test('ER stale, malformed, unknown, superseded and locally unsent outcomes do not speak accepted cues',async()=>{
 for(const result of [{success:true,outcome_unknown:true},{success:true,outcome_unknown:{}},{success:true,superseded:true},{success:false,not_dispatched:true},{success:true,details:{submission:{schema:'other',state:'queued'}}}]){const e=setup();e.s.robotBridge.callTool=async()=>result;await e.s.handleMessage(call('m','move'));assert.deepEqual(e.audio,[]);assert.equal(e.tts.some(x=>x.text==='Move queued.'),false);await e.s.disconnect();}
 for(const event of ['stop','ptt','cancel','disconnect']){const e=setup(),d=deferred();e.s.robotBridge.callTool=()=>d.promise;const pending=e.s.handleMessage(call('old','move'));
 if(event==='stop')e.s.pauseRobotTools();if(event==='cancel')await e.s.handleMessage({toolCallCancellation:{ids:['old']}});if(event==='disconnect')await e.s.disconnect();if(event==='ptt'){e.s.config.enableAudio=true;e.s.inputAudioContext={state:'running',close:async()=>{}};e.s.mediaStream={getAudioTracks:()=>[{enabled:false}],getTracks:()=>[{stop(){}}]};assert.equal(e.s.beginAudioInput(),true);}
 d.resolve(queued);await pending;assert.deepEqual(e.audio,[],event);assert.deepEqual(e.tts,[],event);await e.s.disconnect();}
});
test('ER known failure and confirmed Stop use reviewed fixed phrases',async()=>{
 const e=setup();e.s.robotBridge.callTool=async name=>name==='stop'?{success:true}:{success:false,message:'arbitrary false prose'};
 await e.s.handleMessage(call('m','move'));assert.equal(e.audio[0],cache.lookup('That request failed. Check the controls for details.',ER,'Puck',TTS));
 await e.s.handleMessage(end('ignored'));await e.s.handleMessage(call('s','stop'));assert.equal(e.audio.at(-1),cache.lookup('Stop requested.',ER,'Puck',TTS));assert.deepEqual(e.tts,[]);await e.s.disconnect();
});
test('ER uncached status speaks only typed current facts and fourth handshake preserves composed joke once',async()=>{
 const e=setup();const status={success:true,message:'I am physically standing',details:{running:false,queued:0,current_action:null,observed_controller_mode:'sit_stay_shake',session_context:{schema:'pupper_session_context_v1',scope:'this_server_session',server_instance:'148e99b5fc6646a3a002652f28e8e751',completed_sit_entries:1}}};e.s.robotBridge.callTool=async name=>name==='robot_status'?status:queued;
 for(let i=1;i<=4;i++){
  await e.s.handleMessage(call('s'+i,'robot_status'));const pendingStatus=e.tts.at(-1);assert.equal(pendingStatus.text,'Sit controller active. No command running; 0 queued. 1 Sit-entry playback confirmed this server session.');
  await e.s.handleMessage(call('p'+i,'interactive_action',{action:'shake_left'}));assert.equal(pendingStatus.current(),false);
  assert.equal(e.audio.length,i);
  await e.s.handleMessage(end('I physically shook your hand.'));
  if(i===4)assert.equal(e.tts.at(-1).text,'At this rate, I should charge for pawtographs.');
 }
 assert.equal(e.tts.filter(x=>x.text.includes('pawtographs')).length,1);assert.equal(e.audio.length,4);await e.s.disconnect();
});
test('actual ER receipt TTS discards late chunks after newer same-batch request, PTT, Stop and disconnect',async()=>{
 for(const event of ['newer','ptt','stop','disconnect']){const e=setup(undefined),d=deferred();let yielded=false;
 e.s.synthesizeAndPlaySpeech=GeminiLiveService.prototype.synthesizeAndPlaySpeech.bind(e.s);
 e.s.ai={models:{generateContentStream:async()=>({async *[Symbol.asyncIterator](){await d.promise;yielded=true;yield {candidates:[{content:{parts:[{inlineData:{data:'late'}}]}}]};}})}};
 await e.s.handleMessage(call('status','robot_status'));
 if(event==='newer')await e.s.handleMessage(call('move','move'));if(event==='stop')e.s.pauseRobotTools();if(event==='disconnect')await e.s.disconnect();if(event==='ptt'){e.s.config.enableAudio=true;e.s.inputAudioContext={state:'running',close:async()=>{}};e.s.mediaStream={getAudioTracks:()=>[{enabled:false}],getTracks:()=>[{stop(){}}]};e.s.beginAudioInput();}
 const count=e.audio.length;d.resolve();await new Promise(r=>setImmediate(r));assert.equal(yielded,true);assert.equal(e.audio.length,count,event);await e.s.disconnect();}
});

test('receipt currentness is rechecked after awaited audio decode without a generation change',async()=>{
 const e=setup(),d=deferred(),starts=[];let current=true;
 e.s.audioContext={state:'running',currentTime:0,decodeAudioData:()=>d.promise,createBufferSource:()=>({connect(){},start:t=>starts.push(t)}),close:async()=>{}};e.s.analyser={};
 const before=e.s.playbackGeneration;const pending=GeminiLiveService.prototype.playAudioBase64.call(e.s,'UklGRg==',undefined,()=>current);
 current=false;d.resolve({duration:1});await pending;assert.equal(e.s.playbackGeneration,before);assert.deepEqual(starts,[]);await e.s.disconnect();
});
