import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
const bundle = await build({ entryPoints: ['services/geminiLive.ts'], bundle:true, write:false, format:'esm', platform:'node', plugins:[{name:'offline-sdk',setup(b){ b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'})); b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI {} export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};"})); }}] });
const {GeminiLiveService}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
function deferred(){let resolve; const promise=new Promise(r=>resolve=r);return {promise,resolve};}
function service(policy){const logs=[],dispatches=[],responses=[],audio=[],speech=[],calls=[];const config={modelName:'gemini-3.1-flash-live-preview',robotApiUrl:'http://output-test',outputPolicy:policy};const s=new GeminiLiveService(config,x=>logs.push(x),()=>{},()=>{},x=>dispatches.push(x));s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:r=>responses.push(r)});s.playAudioBase64=async x=>audio.push(x);s.synthesizeAndPlaySpeech=async x=>speech.push(x);s.robotBridge.callTool=async(name)=>{calls.push(name);return {success:true,message:'FAKE PHYSICAL CLAIM'};};s.robotBridge.immediateStop=async()=>({success:true});return {s,config,logs,dispatches,responses,audio,speech,calls};}
const call=(id,name,args={})=>({toolCall:{functionCalls:[{id,name,args}]}});
test('receipts mode suppresses forged no-tool native audio and text',async()=>{for(const p of ['bad','receipts']){const e=service(p);await e.s.handleMessage({serverContent:{modelTurn:{parts:[{text:'I completed the walk'},{inlineData:{data:'forged'}}]},outputTranscription:{text:'I am standing'}}});assert.deepEqual(e.audio,[]);assert.equal(e.logs.some(x=>x.type==='model'||x.message.includes('I completed')),false);}});
test('strict speech aliases never invoke TTS even for text models',async()=>{const e=service('receipts');e.s.config.modelName='gemini-robotics-er-2-streaming-preview';for(const name of ['speak','say','narrate'])await e.s.handleMessage(call(name,name,{text:'I moved already'}));assert.deepEqual(e.speech,[]);assert.equal(e.logs.some(x=>x.message.includes('I moved already')),false);});
test('receipt feedback waits for actual matching response without trusting prose',async()=>{const e=service('receipts'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;const pending=e.s.handleMessage(call('walk','move'));assert.equal(e.logs.some(x=>x.type==='receipt'),false);d.resolve({success:true,message:'I walked and finished'});await pending;assert.equal(e.logs.filter(x=>x.type==='receipt').length,1);assert.equal(e.logs.find(x=>x.type==='receipt').message,'Server accepted the move request. Completion is unconfirmed.');});

const feedbackBundle=await build({entryPoints:['services/operatorFeedback.ts'],bundle:true,write:false,format:'esm',platform:'node'});
const {operatorFeedback,normalizeOutputPolicy,ReceiptSpeaker}=await import('data:text/javascript;base64,'+Buffer.from(feedbackBundle.outputFiles[0].text).toString('base64'));
function replace(t,name,value){const old=Object.getOwnPropertyDescriptor(globalThis,name);Object.defineProperty(globalThis,name,{value,writable:true,configurable:true});t.after(()=>old?Object.defineProperty(globalThis,name,old):delete globalThis[name]);}
function speech(t){const spoken=[],cancelled=[];replace(t,'window',{speechSynthesis:{speak:u=>spoken.push(u),cancel:()=>cancelled.push(true)}});replace(t,'SpeechSynthesisUtterance',class{constructor(text){this.text=text;}});return {spoken,cancelled};}
test('formatter permits only typed queued evidence and never raw or prototype names',()=>{
 const typed={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}},message:'completed physically'};
 assert.equal(operatorFeedback('move',typed),'Server queued the move request. Completion is unconfirmed.');
 for(const result of [{success:true,message:'queued'}, {success:true,details:{submission:{schema:'other',state:'queued'}}}, {success:true,details:{submission:{schema:'pupper_submission_v1',state:'completed'}}}]) assert.equal(operatorFeedback('move',result),'Server accepted the move request. Completion is unconfirmed.');
 for(const name of ['toString','__proto__','I moved safely']) assert.equal(operatorFeedback(name,typed),'Server accepted the robot request. Completion is unconfirmed.');
 assert.equal(operatorFeedback('stop',typed),'Server accepted the Stop request. Completion is unconfirmed. Physical stopping is unverified.');
 assert.equal(operatorFeedback('move',{success:false}),'Server reported the move request failed. Earlier effects are not ruled out.');
 assert.equal(operatorFeedback('move',{success:false,not_dispatched:true}),'The move request was not dispatched.');
 assert.equal(operatorFeedback('move',{success:false,outcome_unknown:true,not_dispatched:true}),'The move response is unknown. Check server status and the robot.');
 assert.equal(operatorFeedback('move',{success:true,superseded:true}),'This request was superseded. Earlier execution is possible.');
 assert.equal(normalizeOutputPolicy(undefined),'conversation');
 for(const value of [null,'bad',true]) assert.equal(normalizeOutputPolicy(value),'receipts');
});
test('late older receipt cannot speak after newer request, Stop, cancellation or disconnect',async t=>{
 const voice=speech(t);
 for(const event of ['newer','stop','cancel','disconnect']){
  const e=service('receipts'),d=deferred();e.s.robotBridge.callTool=name=>name==='move'?d.promise:Promise.resolve({success:true});
  const pending=e.s.handleMessage(call('old','move'));
  if(event==='newer') await e.s.handleMessage(call('new','robot_status'));
  if(event==='stop') e.s.pauseRobotTools();
  if(event==='cancel') await e.s.handleMessage({toolCallCancellation:{ids:['old']}});
  if(event==='disconnect') await e.s.disconnect();
  const count=voice.spoken.length;d.resolve({success:true,message:'old done'});await pending;
  assert.equal(voice.spoken.length,count,event);assert.equal(e.logs.some(x=>x.type==='receipt'&&x.message.includes('move')),false,event);
  await e.s.disconnect();
 }
});
test('bounded receipt speech coalesces pending receipts and stale callbacks cannot restart output',t=>{
 const voice=speech(t),speaker=new ReceiptSpeaker();speaker.say('first');const first=voice.spoken[0];
 for(let i=0;i<50;i++)speaker.say('receipt '+i);
 assert.equal(voice.spoken.length,1);first.onend();assert.equal(voice.spoken.length,2);assert.equal(voice.spoken[1].text,'receipt 49');
 const last=voice.spoken[1];speaker.say('discard');speaker.cancel();last.onend();assert.equal(voice.spoken.length,2);
});
test('Stop dispatch does not await a stuck receipt utterance and clears old speech',async t=>{
 const voice=speech(t),e=service('receipts');await e.s.handleMessage(call('move','move'));assert.equal(voice.spoken.length,1);
 await e.s.handleMessage(call('stop','stop'));assert.deepEqual(e.calls,['move','stop']);assert.ok(voice.cancelled.length>0);
 assert.equal(voice.spoken.at(-1).text,'Server accepted the Stop request. Completion is unconfirmed. Physical stopping is unverified.');await e.s.disconnect();
});
test('strict lower-level output sinks cannot be bypassed',async t=>{
 const voice=speech(t),e=service('receipts');e.s.ai={models:{generateContent(){throw Error('must not call TTS');},generateContentStream(){throw Error('must not call TTS');}}};
 e.s.audioContext={state:'running',decodeAudioData(){throw Error('must not decode');}};e.s.analyser={};
 await GeminiLiveService.prototype.playAudioBase64.call(e.s,'forged');
 await GeminiLiveService.prototype.synthesizeAndPlaySpeech.call(e.s,'forged');
 e.s.speakWithWebSpeech('forged');assert.deepEqual(voice.spoken,[]);
});
test('conversation native text is visibly labeled and cannot alter frozen output policy',async()=>{
 const e=service('conversation');e.config.outputPolicy='receipts';await e.s.handleMessage({serverContent:{modelTurn:{parts:[{text:'I see a triangle'},{inlineData:{data:'native'}}]},outputTranscription:{text:'I see a triangle'},turnComplete:true}});
 assert.deepEqual(e.audio,['native']);assert.match(e.logs.find(x=>x.type==='model').message,/^UNVERIFIED CONVERSATION:/);assert.equal(e.s.outputPolicy,'conversation');
});


test('optional speech constructor/speak/cancel failures cannot lose receipts or block independent Stop',async t=>{
 for(const failure of ['constructor','speak','cancel']) await t.test(failure,async t=>{
  const voice=speech(t),e=service('receipts');
  if(failure==='constructor') replace(t,'SpeechSynthesisUtterance',class {constructor(){throw Error('speech constructor unavailable');}});
  if(failure==='speak') window.speechSynthesis.speak=()=>{throw Error('speech blocked');};
  if(failure==='cancel') window.speechSynthesis.cancel=()=>{throw Error('speech cancellation failed');};
  await e.s.handleMessage(call('move','move'));await Promise.resolve();
  assert.equal(e.responses.at(-1).functionResponses.response.result.success,true,'optional output cannot replace actual server reply');
  let stopped=0;e.s.robotBridge.immediateStop=async()=>{stopped++;return {success:true};};
  const exec=await build({entryPoints:['services/executionStatus.ts'],bundle:true,write:false,format:'esm',platform:'node'});
  const {requestIndependentStop}=await import('data:text/javascript;base64,'+Buffer.from(exec.outputFiles[0].text).toString('base64'));
  await requestIndependentStop(e.s,e.s.robotBridge);assert.equal(stopped,1);await e.s.disconnect();
 });
});


test('old status response after direct Stop cannot publish accepted dispatch card',async()=>{
 const e=service('receipts'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
 const pending=e.s.handleMessage(call('status','robot_status'));e.s.pauseRobotTools();
 d.resolve({success:true});await pending;assert.deepEqual(e.dispatches.map(x=>x.status),['sending']);
});
test('current unknown response remains visible after it pauses later model actions',async()=>{
 const e=service('receipts');e.s.robotBridge.callTool=async()=>({success:false,outcome_unknown:true});
 await e.s.handleMessage(call('unknown','move'));
 assert.equal(e.s.robotToolsPaused,true);assert.equal(e.logs.find(x=>x.type==='receipt')?.message,'The move response is unknown. Check server status and the robot.');
 assert.equal(e.dispatches.at(-1).status,'unknown');
});
test('reused model call ID cannot turn an older response into a current acknowledgment',async()=>{
 const e=service('receipts'),old=deferred();e.s.robotBridge.callTool=name=>name==='move'?old.promise:Promise.resolve({success:true});
 const pending=e.s.handleMessage(call('same-id','move'));await e.s.handleMessage(call('same-id','robot_status'));
 const before=e.dispatches.length;old.resolve({success:true});await pending;assert.equal(e.dispatches.length,before);
 assert.equal(e.logs.filter(x=>x.type==='receipt').length,1);
});


test('malformed outcome flags cannot manufacture acceptance',()=>{
 for(const flag of ['outcome_unknown','superseded','not_dispatched']) for(const value of [{},[],0,1,'',null,'false']) {
  assert.equal(operatorFeedback('move',{success:true,[flag]:value}),'The move response is unknown. Check server status and the robot.');
 }
});
test('throwing bridge publishes current unknown feedback without replacing the real outcome',async()=>{
 const e=service('receipts');e.s.robotBridge.callTool=async()=>{throw Error('lost reply');};
 await e.s.handleMessage(call('unknown-error','move'));await Promise.resolve();
 assert.equal(e.dispatches.at(-1).status,'unknown');assert.match(e.logs.find(x=>x.type==='receipt').message,/response is unknown/);
 assert.equal(e.responses.at(-1).functionResponses.response.result.outcome_unknown,true);
});


test('combined mode delays native output until boundary and suppresses an eventual tool turn',async()=>{
 const e=service('conversation');
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'early'}}]},outputTranscription:{text:'I have started walking.'}}});
 assert.deepEqual(e.audio,[]);
 await e.s.handleMessage(call('actual-move','move'));
 await e.s.handleMessage({serverContent:{turnComplete:true}});
 assert.deepEqual(e.calls,['move']);assert.deepEqual(e.audio,[]);
});
test('combined mode can dispatch movement and later have a natural conversation in same session',async()=>{
 const e=service('conversation');await e.s.handleMessage(call('move','move'));
 await e.s.handleMessage({serverContent:{turnComplete:true}});
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'hello'}}]},outputTranscription:{text:'Hello! How are you?'},turnComplete:true}});
 await new Promise(r=>setImmediate(r));
 assert.deepEqual(e.calls,['move']);assert.deepEqual(e.audio,['hello']);
});
test('native no-tool output without transcript is not released at boundary',async()=>{
 const e=service('conversation');await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'no-transcript'}}]},turnComplete:true}});
 assert.deepEqual(e.audio,[]);
});


test('Stop before a late first packet quarantines output until boundary then conversation recovers',async()=>{
 const e=service('conversation');e.s.pauseRobotTools();
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'old'}}]},outputTranscription:{text:'Hello old friend'},turnComplete:true}});
 assert.deepEqual(e.audio,[]);
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'new'}}]},outputTranscription:{text:'How is your day?'},turnComplete:true}});
 await new Promise(r=>setImmediate(r));assert.deepEqual(e.audio,['new']);
});
test('interrupted or cancelled first packet cannot release its own content',async()=>{
 for(const event of [{serverContent:{interrupted:true}},{toolCallCancellation:{ids:['old']}}]){
  const e=service('conversation');await e.s.handleMessage({...event,serverContent:{...event.serverContent,
   modelTurn:{parts:[{inlineData:{data:'old'}}]},outputTranscription:{text:'Hello there'},turnComplete:true}});
  assert.deepEqual(e.audio,[]);
 }
});
test('split text tokens are screened exactly as displayed',async()=>{
 const e=service('conversation');e.s.config.modelName='gemini-robotics-er-2-streaming-preview';
 for(const text of ['I am sta','nding.'])await e.s.handleMessage({serverContent:{modelTurn:{parts:[{text}]}}});
 await e.s.handleMessage({serverContent:{turnComplete:true}});
 assert.equal(e.logs.some(x=>x.type==='model'),false);assert.ok(e.logs.some(x=>x.message==='No confirmed robot action in this reply; check server status.'));
});
test('completion arriving before tool HTTP reply cannot release tool-turn narration',async()=>{
 const e=service('conversation'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
 const pending=e.s.handleMessage(call('move','move'));
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'early-ack'}}]},outputTranscription:{text:'Okay'},turnComplete:true}});
 assert.deepEqual(e.audio,[]);assert.equal(e.logs.some(x=>x.type==='receipt'),false);
 d.resolve({success:true});await pending;assert.deepEqual(e.audio,[]);assert.equal(e.logs.filter(x=>x.type==='receipt').length,1);
});
test('non-native speak is buffered, never awaited by tools, and discarded for tool turns',async()=>{
 const e=service('conversation');e.s.config.modelName='gemini-robotics-er-2-streaming-preview';
 await e.s.handleMessage(call('say','speak',{text:'Hello there'}));assert.deepEqual(e.speech,[]);
 await e.s.handleMessage(call('move','move'));await e.s.handleMessage({serverContent:{turnComplete:true}});
 assert.deepEqual(e.speech,[]);assert.deepEqual(e.calls,['move']);
 await e.s.handleMessage(call('next','speak',{text:'How is your day?'}));await e.s.handleMessage({serverContent:{turnComplete:true}});
 await new Promise(r=>setImmediate(r));assert.deepEqual(e.speech,['How is your day?']);
});


test('Stop and boundary in the same packet do not discard the next genuine chat turn',async()=>{
 const e=service('conversation');await e.s.handleMessage({...call('stop','stop'),serverContent:{turnComplete:true}});
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'new'}}]},outputTranscription:{text:'How is your day?'},turnComplete:true}});
 await new Promise(r=>setImmediate(r));assert.deepEqual(e.audio,['new']);
});
test('screened no-tool claim uses fixed local feedback rather than native audio',async t=>{
 const voice=speech(t),e=service('conversation');
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'claim'}}]},outputTranscription:{text:'I am standing now.'},turnComplete:true}});
 assert.deepEqual(e.audio,[]);assert.equal(voice.spoken[0].text,'No confirmed robot action in this reply; check server status.');await e.s.disconnect();
});
