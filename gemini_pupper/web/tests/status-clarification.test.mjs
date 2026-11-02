import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const bundle=await build({stdin:{contents:"export {GeminiLiveService,supportsPawClarification} from './services/geminiLive';",resolveDir:process.cwd(),loader:'ts'},bundle:true,write:false,platform:'node',format:'esm',plugins:[{name:'sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI{};export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'};export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {GeminiLiveService,supportsPawClarification}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const QUESTION='Which paw would you like, left or right?';
const STATUS='Sit controller active. No command running; 0 queued. 1 Sit-entry playback confirmed this server session.';
const reply={success:true,message:'INJECTED physical claim',details:{running:false,queued:0,current_action:null,observed_controller_mode:'sit_stay_shake',session_context:{schema:'pupper_session_context_v1',scope:'this_server_session',server_instance:'148e99b5fc6646a3a002652f28e8e751',completed_sit_entries:1}}};
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
const call=(id,name='robot_status',args={})=>({toolCall:{functionCalls:[{id,name,args}]}});
const end=(text,audio)=>({serverContent:{outputTranscription:{text},...(audio?{modelTurn:{parts:[{inlineData:{data:audio}}]}}:{}),generationComplete:true}});
const boundary={serverContent:{turnComplete:true}};
function deferred(){let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};}
function service(native=false,capability=true){const audio=[],tts=[],responses=[],calls=[],s=new GeminiLiveService({modelName:native?'gemini-3.1-flash-live-preview':'gemini-robotics-er-2-streaming-preview',ttsModelName:'gemini-3.1-flash-tts-preview',voiceName:'Puck',outputPolicy:'conversation',robotApiUrl:'http://clarify'},()=>{},()=>{},()=>{});s.connected=true;s.pawClarificationAvailable=capability;s.sessionPromise=Promise.resolve({sendToolResponse:r=>responses.push(r),sendRealtimeInput(){}});s.robotBridge.callTool=async name=>{calls.push(name);return name==='robot_status'?reply:name==='stop'?{success:true}:queued;};s.robotBridge.immediateStop=async()=>({success:true});s.playAudioBase64=async data=>audio.push(data);s.synthesizeAndPlaySpeech=async(text,current=()=>true)=>tts.push({text,current});return {s,audio,tts,responses,calls};}
test('single successful status offers one fixed clarification only after response and speaks it once at generation end',async()=>{
 for(const native of [false,true]){const e=service(native),d=deferred();e.s.robotBridge.callTool=()=>d.promise;const pending=e.s.handleMessage(call('s'));assert.deepEqual(e.audio,[]);assert.deepEqual(e.tts,[]);d.resolve(reply);await pending;
 assert.equal(e.responses[0].functionResponses.response.operator_acknowledgment.optional_clarification.text,QUESTION);assert.deepEqual(e.tts,[]);
 await e.s.handleMessage(end(QUESTION,'question'));assert.deepEqual(native?e.audio:e.tts.map(x=>x.text),[native?'question':QUESTION]);await e.s.handleMessage(end(QUESTION,'duplicate'));await e.s.handleMessage(boundary);assert.equal(native?e.audio.length:e.tts.length,1);await e.s.disconnect();}
});
test('ER valid ordinary status falls back to actual typed facts, never model paraphrase or combined claim',async()=>{
 for(const text of [STATUS,'The count is one.','I am physically sitting. '+QUESTION,'Do you want a paw?',QUESTION+' I have finished moving.','']){const e=service();await e.s.handleMessage(call('s'));assert.deepEqual(e.tts,[]);await e.s.handleMessage(end(text));assert.deepEqual(e.tts.map(x=>x.text),[STATUS]);await e.s.disconnect();}
});
test('native extra claims/paraphrases remain muted and exact ordinary status remains admissible',async()=>{
 for(const text of ['I am physically sitting. '+QUESTION,QUESTION+' I moved.','Which foot?',STATUS]){const e=service(true);await e.s.handleMessage(call('s'));await e.s.handleMessage(end(text,'audio'));assert.deepEqual(e.audio,text===STATUS?['audio']:[]);await e.s.disconnect();}
});
test('no capability or failed/unknown/superseded/not-dispatched status never offers question',async()=>{
 for(const [cap,result] of [[false,reply],[true,{success:false}],[true,{success:true,outcome_unknown:true}],[true,{success:true,outcome_unknown:{}}],[true,{success:true,superseded:true}],[true,{success:false,not_dispatched:true}]]){const e=service(true,cap);e.s.robotBridge.callTool=async()=>result;await e.s.handleMessage(call('s'));assert.equal(e.responses[0]?.functionResponses.response.operator_acknowledgment?.optional_clarification,undefined);await e.s.handleMessage(end(QUESTION,'question'));assert.deepEqual(e.audio,[]);await e.s.disconnect();}
});
test('pre-response output and closure cannot authorize question; typed ER fallback may remain current',async()=>{
 for(const closed of [false,true]){const e=service(),d=deferred();e.s.robotBridge.callTool=()=>d.promise;const pending=e.s.handleMessage(call('s'));await e.s.handleMessage({serverContent:{outputTranscription:{text:QUESTION},...(closed?{generationComplete:true}:{})}});d.resolve(reply);await pending;assert.equal(e.responses[0]?.functionResponses.response.operator_acknowledgment?.optional_clarification,undefined);await e.s.handleMessage(end(''));assert.equal(e.tts.some(x=>x.text===QUESTION),false);await e.s.disconnect();}
});
test('any mixed/duplicate status/action/auth/Stop tool invalidates question without dispatching an invented action',async()=>{
 for(const name of ['move','play_trick','interactive_action','wait','unlock','lock_tricks','stop','robot_status']){const e=service(true);await e.s.handleMessage(call('s'));await e.s.handleMessage(call(name==='robot_status'?'s':'x',name,{name:'sit_stay_shake',action:'shake_left'}));await e.s.handleMessage(end(QUESTION,'question'));assert.deepEqual(e.audio,[],name);await e.s.disconnect();}
});
test('new PTT, Stop, cancel and disconnect invalidate pending status-choice output',async()=>{
 for(const event of ['ptt','stop','cancel','disconnect']){const e=service();await e.s.handleMessage(call('s'));if(event==='stop')e.s.pauseRobotTools();if(event==='cancel')await e.s.handleMessage({toolCallCancellation:{ids:['s']}});if(event==='disconnect')await e.s.disconnect();if(event==='ptt'){e.s.config.enableAudio=true;e.s.inputAudioContext={state:'running',close:async()=>{}};e.s.mediaStream={getAudioTracks:()=>[{enabled:false}],getTracks:()=>[{stop(){}}]};e.s.beginAudioInput();}await e.s.handleMessage(end(QUESTION));assert.deepEqual(e.tts,[],event);await e.s.disconnect();}
});
test('eligible status followed by explicit paw preserves fourth-handshake joke without status-question duplication',async()=>{
 const e=service();for(let i=1;i<=4;i++){await e.s.handleMessage(call('s'+i));assert.equal(e.tts.length,i-1);await e.s.handleMessage(call('p'+i,'interactive_action',{action:'shake_left'}));await e.s.handleMessage(end('ignored'));await e.s.handleMessage(boundary);assert.equal(e.tts.length,i);}assert.equal(e.tts[3].text,'Left paw command queued. At this rate, I should charge for pawtographs.');assert.equal(e.tts.some(x=>x.text===QUESTION||x.text===STATUS),false);await e.s.disconnect();
});

test('clarification capability requires advertised tools, Sit entry and both exact action mappings',()=>{
 const tools=['robot_status','play_trick','interactive_action'].map(name=>({name})), tricks={sit_stay_shake:{}},actions={shake_left:{trick:'sit_stay_shake'},shake_right:{trick:'sit_stay_shake'}};
 assert.equal(supportsPawClarification(tools,tricks,actions),true);
 for(const args of [[tools.slice(1),tricks,actions],[tools,{},actions],[tools,tricks,{shake_left:actions.shake_left}],[tools,tricks,{...actions,shake_right:{trick:'other'}}],[tools,tricks,Object.create(actions)]])assert.equal(supportsPawClarification(...args),false);
});
