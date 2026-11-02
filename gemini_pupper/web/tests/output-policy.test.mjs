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
test('receipt feedback waits for actual matching response without trusting prose',async()=>{const e=service('receipts'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;const pending=e.s.handleMessage(call('walk','move'));assert.equal(e.logs.some(x=>x.type==='receipt'),false);d.resolve({success:true,message:'I walked and finished'});await pending;assert.equal(e.logs.filter(x=>x.type==='receipt').length,1);assert.equal(e.logs.find(x=>x.type==='receipt').message,'Move request accepted.');});

const feedbackBundle=await build({entryPoints:['services/operatorFeedback.ts'],bundle:true,write:false,format:'esm',platform:'node'});
const {operatorFeedback,nativeReceiptFeedback,isOperatorReceiptNarration,normalizeOutputPolicy,ReceiptSpeaker}=await import('data:text/javascript;base64,'+Buffer.from(feedbackBundle.outputFiles[0].text).toString('base64'));
function replace(t,name,value){const old=Object.getOwnPropertyDescriptor(globalThis,name);Object.defineProperty(globalThis,name,{value,writable:true,configurable:true});t.after(()=>old?Object.defineProperty(globalThis,name,old):delete globalThis[name]);}
function speech(t){const spoken=[],cancelled=[];replace(t,'window',{speechSynthesis:{speak:u=>spoken.push(u),cancel:()=>cancelled.push(true)}});replace(t,'SpeechSynthesisUtterance',class{constructor(text){this.text=text;}});return {spoken,cancelled};}
test('formatter permits only typed queued evidence and never raw or prototype names',()=>{
 const typed={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}},message:'completed physically'};
 assert.equal(operatorFeedback('move',typed),'Move queued.');
 for(const result of [{success:true,message:'queued'}, {success:true,details:{submission:{schema:'other',state:'queued'}}}, {success:true,details:{submission:{schema:'pupper_submission_v1',state:'completed'}}}]) assert.equal(operatorFeedback('move',result),'Move request accepted.');
 for(const name of ['toString','__proto__','I moved safely']) assert.equal(operatorFeedback(name,typed),'Robot request accepted.');
 assert.equal(operatorFeedback('stop',typed),'Stop requested.');
 assert.equal(operatorFeedback('move',{success:false}),'That request failed. Check the controls for details.');
 assert.equal(operatorFeedback('move',{success:false,not_dispatched:true}),'Move request not sent.');
 assert.equal(operatorFeedback('move',{success:false,outcome_unknown:true,not_dispatched:true}),'I could not confirm that request.');
 assert.equal(operatorFeedback('move',{success:true,superseded:true}),'That request was superseded.');
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
test('Stop and action receipts remain visible without synthesizing a second voice',async t=>{
 const voice=speech(t),e=service('receipts');await e.s.handleMessage(call('move','move'));assert.deepEqual(voice.spoken,[]);
 assert.equal(e.logs.find(x=>x.type==='receipt').message,'Move request accepted.');
 await e.s.handleMessage(call('stop','stop'));assert.deepEqual(e.calls,['move','stop']);assert.ok(voice.cancelled.length>0);
 assert.deepEqual(voice.spoken,[]);assert.equal(e.logs.filter(x=>x.type==='receipt').at(-1).message,'Stop requested.');await e.s.disconnect();
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
 assert.equal(e.s.robotToolsPaused,true);assert.equal(e.logs.find(x=>x.type==='receipt')?.message,'I could not confirm that request.');
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
  assert.equal(operatorFeedback('move',{success:true,[flag]:value}),'I could not confirm that request.');
 }
});
test('throwing bridge publishes current unknown feedback without replacing the real outcome',async()=>{
 const e=service('receipts');e.s.robotBridge.callTool=async()=>{throw Error('lost reply');};
 await e.s.handleMessage(call('unknown-error','move'));await Promise.resolve();
 assert.equal(e.dispatches.at(-1).status,'unknown');assert.match(e.logs.find(x=>x.type==='receipt').message,/could not confirm/);
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
test('non-native model speak is buffered and discarded for tool turns while code receipts remain',async()=>{
 const e=service('conversation');e.s.config.modelName='gemini-robotics-er-2-streaming-preview';
 await e.s.handleMessage(call('say','speak',{text:'Hello there'}));assert.deepEqual(e.speech,[]);
 await e.s.handleMessage(call('move','move'));await e.s.handleMessage({serverContent:{turnComplete:true}});
 assert.deepEqual(e.speech,['Move request accepted.']);assert.deepEqual(e.calls,['move']);
 await e.s.handleMessage(call('next','speak',{text:'How is your day?'}));await e.s.handleMessage({serverContent:{turnComplete:true}});
 await new Promise(r=>setImmediate(r));assert.deepEqual(e.speech,['Move request accepted.','How is your day?']);
});


test('Stop and boundary in the same packet do not discard the next genuine chat turn',async()=>{
 const e=service('conversation');await e.s.handleMessage({...call('stop','stop'),serverContent:{turnComplete:true}});
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'new'}}]},outputTranscription:{text:'How is your day?'},turnComplete:true}});
 await new Promise(r=>setImmediate(r));assert.deepEqual(e.audio,['new']);
});
test('screened no-tool claim and genuine receipt stay visible without OS speech',async t=>{
 const voice=speech(t),e=service('conversation');
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'claim'}}]},outputTranscription:{text:'I am standing now.'},turnComplete:true}});
 assert.deepEqual(e.audio,[]);assert.deepEqual(voice.spoken,[]);
 assert.ok(e.logs.some(x=>x.type==='info'&&x.message==='No confirmed robot action in this reply; check server status.'));
 assert.equal(e.logs.some(x=>x.type==='model'||x.type==='receipt'),false);
 await e.s.handleMessage(call('move','move'));
 assert.deepEqual(voice.spoken,[]);assert.equal(e.logs.find(x=>x.type==='receipt').message,'Move request accepted.');
 await e.s.disconnect();
});


test('both modes show matching actual server receipts only after response and never synthesize them',async t=>{
 const voice=speech(t);
 for(const mode of ['receipts','conversation']) {
  const e=service(mode),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
  const pending=e.s.handleMessage(call('move','move'));
  assert.equal(e.logs.some(x=>x.type==='receipt'),false);assert.deepEqual(voice.spoken,[]);
  d.resolve({success:true});await pending;
  assert.equal(e.logs.find(x=>x.type==='receipt').message,'Move request accepted.');assert.deepEqual(voice.spoken,[]);
  await e.s.disconnect();
 }
});


test('ER transcript-only output reaches TTS at the complete generation screen without duplicate turn playback',async()=>{
 const e=service('conversation');e.s.config.modelName='gemini-robotics-er-2-streaming-preview';
 await e.s.handleMessage({serverContent:{outputTranscription:{text:'Hello friend!'}}});assert.deepEqual(e.speech,[]);
 await e.s.handleMessage({serverContent:{generationComplete:true}});await new Promise(r=>setImmediate(r));assert.deepEqual(e.speech,['Hello friend!']);
 await e.s.handleMessage({serverContent:{turnComplete:true}});await new Promise(r=>setImmediate(r));assert.deepEqual(e.speech,['Hello friend!']);
});
test('ER transcript-only claims, mixed tool narration and interrupted tails never reach TTS',async()=>{
 for(const kind of ['claim','tool','interrupted']) {
  const e=service('conversation');e.s.config.modelName='gemini-robotics-er-2-streaming-preview';
  if(kind==='tool')await e.s.handleMessage(call('move','move'));
  if(kind==='interrupted')await e.s.handleMessage({serverContent:{interrupted:true}});
  await e.s.handleMessage({serverContent:{outputTranscription:{text:kind==='claim'?'I have finished moving.':'Hello friend!'},turnComplete:true}});
  await new Promise(r=>setImmediate(r));assert.deepEqual(e.speech,kind==='tool'?['Move request accepted.']:[],kind);
 }
});
test('ER text and speak aliases do not create duplicate TTS or use native transcription as a second voice',async()=>{
 const e=service('conversation');e.s.config.modelName='gemini-robotics-er-2-streaming-preview';
 await e.s.handleMessage({...call('speak','speak',{text:'Hello friend!'}),serverContent:{modelTurn:{parts:[{text:'Hello friend!'}]},outputTranscription:{text:'Hello friend!'},turnComplete:true}});
 await new Promise(r=>setImmediate(r));assert.deepEqual(e.speech,['Hello friend!']);
 const native=service('conversation');await native.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'native'}}]},outputTranscription:{text:'Hello friend!'},turnComplete:true}});
 await new Promise(r=>setImmediate(r));assert.deepEqual(native.speech,[]);assert.deepEqual(native.audio,['native']);
});
test('TTS empty/error never falls back to OS voice and cannot block a later Stop',async t=>{
 const voice=speech(t);
 for(const bad of ['empty','error']) {
  const e=service('conversation');e.s.config.modelName='gemini-robotics-er-2-streaming-preview';
  e.s.ai={models:{generateContentStream:async()=>{if(bad==='error')throw Error('no TTS');return (async function*(){yield {candidates:[]};})();}}};
  await GeminiLiveService.prototype.synthesizeAndPlaySpeech.call(e.s,'Hello friend!');
  assert.deepEqual(voice.spoken,[]);assert.ok(e.logs.some(x=>x.type==='error'));
  await e.s.handleMessage(call('stop','stop'));assert.deepEqual(e.calls,['stop']);
 }
});

test('Stop between audio parts in one TTS chunk cannot start its remaining stale audio',async()=>{
 for(const mode of ['stream','unary']) {
  const e=service('conversation'),first=deferred();e.s.config.ttsModelName=mode==='stream'?'gemini-3.1-flash-tts-preview':'gemini-2.5-flash-preview-tts';
  const packet={candidates:[{content:{parts:[{inlineData:{data:'old-first'}},{inlineData:{data:'old-second'}}]}}]};
  e.s.ai={models:{generateContentStream:async()=> (async function*(){yield packet;})(),generateContent:async()=>packet}};
  const played=[];e.s.playAudioBase64=async data=>{played.push(data);if(data==='old-first')await first.promise;};
  const pending=GeminiLiveService.prototype.synthesizeAndPlaySpeech.call(e.s,'Old response');
  await new Promise(r=>setImmediate(r));assert.deepEqual(played,['old-first']);e.s.pauseRobotTools();first.resolve();await pending;
  assert.deepEqual(played,['old-first'],mode);
 }
});


test('TTS quota failure is explicit without leaking provider error text or changing robot tools',async()=>{
 const e=service('conversation');e.s.ai={models:{generateContentStream:async()=>{throw Object.assign(new Error('PRIVATE_PROVIDER_DETAILS'),{status:429});}}};
 await GeminiLiveService.prototype.synthesizeAndPlaySpeech.call(e.s,'Hello friend!');
 assert.ok(e.logs.some(x=>/quota reached/.test(x.message)));assert.equal(e.logs.some(x=>x.message.includes('PRIVATE_PROVIDER_DETAILS')),false);
 assert.equal(e.s.robotToolsPaused,false);await e.s.handleMessage(call('stop','stop'));assert.deepEqual(e.calls,['stop']);
});


const nativeReply=(text='Move request accepted.',data='receipt-audio',complete=true)=>({serverContent:{modelTurn:{parts:[{inlineData:{data}}]},outputTranscription:{text},turnComplete:complete}});
test('native receipt needs actual current reply and exact complete transcript, then preserves ordinary conversation',async()=>{
 const e=service('conversation'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
 const pending=e.s.handleMessage(call('m','move'));assert.equal(e.responses.length,0);
 d.resolve({success:true});await pending;await Promise.resolve();
 assert.equal(e.responses[0].functionResponses.response.operator_acknowledgment.text,'Move request accepted.');
 await e.s.handleMessage(nativeReply('Move request accepted.','a',false));assert.deepEqual(e.audio,[]);
 await e.s.handleMessage({serverContent:{turnComplete:true}});assert.deepEqual(e.audio,['a']);
 await e.s.handleMessage(nativeReply('Hello there!','chat'));assert.deepEqual(e.audio,['a','chat']);
});

test('pre-response or pre-tool native output cannot be retrospectively authorized',async()=>{
 for(const phase of ['before-response','before-tool']) {
  const e=service('conversation'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
  if(phase==='before-tool')await e.s.handleMessage(nativeReply('Move request accepted.','pre',false));
  const pending=e.s.handleMessage(call('m','move'));
  if(phase==='before-response')await e.s.handleMessage(nativeReply('Move request accepted.','pre',false));
  d.resolve({success:true});await pending;
  await e.s.handleMessage(nativeReply('Move request accepted.','late'));assert.deepEqual(e.audio,[]);
  assert.equal(e.responses[0].functionResponses.response.operator_acknowledgment,undefined);
 }
});

test('native receipt rejects added claims, missing transcript and mismatched text even after server success',async()=>{
 for(const packet of [nativeReply('Move request accepted. I am standing.'),{serverContent:{modelTurn:{parts:[{inlineData:{data:'x'}}]},turnComplete:true}},{serverContent:{modelTurn:{parts:[{text:'I am standing.'},{inlineData:{data:'x'}}]},outputTranscription:{text:'Move request accepted.'},turnComplete:true}}]) {
  const e=service('conversation');await e.s.handleMessage(call('m','move'));await e.s.handleMessage(packet);assert.deepEqual(e.audio,[]);
 }
});

test('no-tool forged receipt has no authorization and receipts-only mode remains silent',async()=>{
 for(const mode of ['conversation','receipts']) {const e=service(mode);await e.s.handleMessage(nativeReply('Move queued.'));assert.deepEqual(e.audio,[]);}
 const e=service('receipts');await e.s.handleMessage(call('m','move'));await e.s.handleMessage(nativeReply());assert.deepEqual(e.audio,[]);
});

test('multi-tool native output remains withheld even if it matches the final actual receipt',async()=>{
 for(const split of [false,true]) {
  const e=service('conversation');
  if(split){await e.s.handleMessage(call('m','move'));await e.s.handleMessage(call('s','robot_status'));}
  else await e.s.handleMessage({toolCall:{functionCalls:[{id:'m',name:'move'},{id:'s',name:'robot_status'}]}});
  await e.s.handleMessage(nativeReply('Status request accepted.'));assert.deepEqual(e.audio,[]);
  assert.equal(e.responses.at(-1).functionResponses.response.operator_acknowledgment,undefined);
 }
});

test('Stop, interruption, cancellation, disconnect and a premature boundary invalidate native receipt audio',async()=>{
 for(const kind of ['stop','interrupt','cancel','disconnect','boundary']) {
  const e=service('conversation'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
  const pending=e.s.handleMessage(call('same','move'));
  if(kind==='stop')e.s.pauseRobotTools();
  if(kind==='interrupt')await e.s.handleMessage({serverContent:{interrupted:true}});
  if(kind==='cancel')await e.s.handleMessage({toolCallCancellation:{ids:['same']}});
  if(kind==='disconnect')await e.s.disconnect();
  if(kind==='boundary')await e.s.handleMessage({serverContent:{turnComplete:true}});
  d.resolve({success:true});await pending;await e.s.handleMessage(nativeReply());assert.deepEqual(e.audio,[],kind);
 }
});

test('model Stop itself can acknowledge after its actual reply; Stop during receipt decode suppresses later parts',async()=>{
 const e=service('conversation');await e.s.handleMessage(call('stop','stop'));
 assert.equal(e.responses.at(-1).functionResponses.response.operator_acknowledgment.text,'Stop requested.');
 const decoded=deferred();e.s.playAudioBase64=async data=>{if(data==='first')await decoded.promise;else e.audio.push(data);};
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'first'}},{inlineData:{data:'second'}}]},outputTranscription:{text:'Stop requested.'},turnComplete:true}});
 e.s.pauseRobotTools();decoded.resolve();await new Promise(r=>setImmediate(r));assert.deepEqual(e.audio,[]);
});

test('unknown, superseded, local refusal and malformed outcomes never authorize native receipt',async()=>{
 for(const result of [{success:true,outcome_unknown:true},{success:true,superseded:true},{success:false,not_dispatched:true},{success:true,outcome_unknown:null}]){
  const e=service('conversation');e.s.robotBridge.callTool=async()=>result;
  await e.s.handleMessage(call('m','move'));assert.equal(e.responses[0].functionResponses.response.operator_acknowledgment,undefined);
  await e.s.handleMessage(nativeReply(operatorFeedback('move',result)));assert.deepEqual(e.audio,[]);
 }
});

test('status receipt includes only validated authoritative server-session Sit-entry count',()=>{
 const details={running:false,queued:0,current_action:null,observed_controller_mode:'sit_stay_shake',session_context:{schema:'pupper_session_context_v1',scope:'this_server_session',server_instance:'a'.repeat(32),completed_sit_entries:2}};
 assert.match(operatorFeedback('robot_status',{success:true,details}),/Confirmed Sit-entry playbacks in this server session: 2\./);
 for(const patch of [{schema:'wrong'},{scope:'all_day'},{server_instance:'unknown'},{server_instance:'a'.repeat(32)+'\n'},{completed_sit_entries:-1},{completed_sit_entries:1.5},{completed_sit_entries:'2'},{completed_sit_entries:1000001}])assert.doesNotMatch(operatorFeedback('robot_status',{success:true,details:{...details,session_context:{...details.session_context,...patch}}}),/Confirmed Sit-entry/);
});


test('older status and Stop receipts cannot re-enter ordinary conversation after a newer receipt',async()=>{
 for(const oldTool of ['robot_status','stop']) {
  const e=service('conversation'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
  const old=e.s.handleMessage(call('old',oldTool));await e.s.handleMessage({serverContent:{turnComplete:true}});
  const state=mode=>({success:true,details:{running:false,queued:0,current_action:null,observed_controller_mode:mode}});
  d.resolve(state('locomotion'));await old;const older=e.logs.filter(x=>x.type==='receipt').at(-1).message;
  e.s.robotBridge.callTool=async()=>state('none');await e.s.handleMessage(call('new','robot_status'));
  const newer=e.responses.at(-1).functionResponses.response.operator_acknowledgment.text;
  await e.s.handleMessage(nativeReply(newer,'current-authorized'));
  await e.s.handleMessage(nativeReply(older,'stale-unbound'));
  await e.s.handleMessage(nativeReply('Hello again!','chat'));
  assert.deepEqual(e.audio,['current-authorized','chat']);
 }
});

test('all fixed receipt families are reserved for authorized output, not free-form narration',()=>{
 for(const name of ['move','turn','arc','wait','stand_up','lie_down','stop','robot_status','play_trick','interactive_action','follow','unlock','lock_tricks','unknown']) {
  for(const value of [{success:true},{success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}},{success:false},{success:false,not_dispatched:true},{outcome_unknown:true},{superseded:true}]) {
   const text=operatorFeedback(name,value);assert.equal(isOperatorReceiptNarration(text),true,text);
  }
 }
 assert.equal(isOperatorReceiptNarration('Hello there!'),false);
 assert.equal(isOperatorReceiptNarration('The controller is a component of the robot.'),false);
 assert.equal(isOperatorReceiptNarration('Controller design is an interesting subject.'),false);
});


test('native status stays concise while visible details retain full scoped context',async()=>{
 const e=service('conversation');const details={running:false,queued:0,current_action:null,observed_controller_mode:'locomotion',following:false,session_context:{schema:'pupper_session_context_v1',scope:'this_server_session',server_instance:'a'.repeat(32),completed_sit_entries:1}};
 e.s.robotBridge.callTool=async()=>({success:true,details});await e.s.handleMessage(call('status','robot_status'));
 const visible=e.logs.find(x=>x.type==='receipt').message,spoken=e.responses[0].functionResponses.response.operator_acknowledgment.text;
 assert.match(visible,/Controller: locomotion.*Following off.*Confirmed Sit-entry playbacks in this server session: 1/);
 assert.equal(spoken,'Walking controller active. No command running; 0 queued. 1 Sit-entry playback confirmed this server session.');
 assert.ok(spoken.split(/\s+/).length<=25);await e.s.handleMessage(nativeReply(spoken,'short-status'));assert.deepEqual(e.audio,['short-status']);
});

test('concise native status does not turn cache, malformed counts or faults into physical claims',()=>{
 const base={success:true,details:{running:false,queued:0,current_action:null,session_context:{schema:'pupper_session_context_v1',scope:'this_server_session',server_instance:'a'.repeat(32),completed_sit_entries:2}}};
 assert.match(nativeReceiptFeedback('robot_status',base),/^Controller state unverified/);
 for(const mode of ['locomotion','none','unknown','sit_stay_shake','secret_dance','other_skill','walking_skill','locked_skill','INJECTED']) {
  const text=nativeReceiptFeedback('robot_status',{...base,details:{...base.details,observed_controller_mode:mode}});
  assert.equal(isOperatorReceiptNarration(text),true,text);assert.doesNotMatch(text,/INJECTED|physically|ready|sitting|standing|today/);
 }
 const invalid=nativeReceiptFeedback('robot_status',{...base,details:{...base.details,running:true,queued:3,following:true,controller_fault:'INJECTED',session_context:{...base.details.session_context,completed_sit_entries:'2'}}});
 assert.match(invalid,/A command is running; 3 queued.*Following enabled.*Controller fault reported/);
 assert.doesNotMatch(invalid,/Sit-entry|INJECTED/);
 assert.equal(nativeReceiptFeedback('robot_status',{success:false}),operatorFeedback('robot_status',{success:false}));
});


const queuedHandshake={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
function handshakes(e){let number=0;e.s.robotBridge.callTool=async name=>name==='robot_status'?{success:true,details:{running:false,queued:0,current_action:null,observed_controller_mode:'sit_stay_shake'}}:queuedHandshake;return async({entry=false,paw='shake_left',preamble=false,emit=true}={})=>{
 const id=++number;
 if(preamble)await e.s.handleMessage(nativeReply('Let me check.','pre',false));
 await e.s.handleMessage(call('status'+id,'robot_status'));
 const calls=[...(entry?[{id:'entry'+id,name:'play_trick',args:{name:'sit_stay_shake'}}]:[]),{id:'paw'+id,name:'interactive_action',args:{action:paw}}];
 await e.s.handleMessage({toolCall:{functionCalls:calls}});
 const text=e.responses.at(-1)?.functionResponses.response.operator_acknowledgment?.text;
 if(emit)await e.s.handleMessage(nativeReply(text??'Unapproved narration.','handshake'+id));
 return text;
};}

test('whitelisted status/optional-entry/paw chain has one post-server acknowledgment and fourth-request joke',async()=>{
 const e=service('conversation'),shake=handshakes(e);
 for(let i=1;i<=4;i++){
  const text=await shake({entry:i===1});assert.equal(text,i===4?'Left paw command queued. At this rate, I should charge for pawtographs.':'Left paw command queued.');
 }
 assert.deepEqual(e.audio,['handshake1','handshake2','handshake3','handshake4']);
 assert.equal(await shake(),'Left paw command queued.','joke once per consecutive streak');
 await e.s.handleMessage(nativeReply('Two plus two is four.','unrelated'));
 for(let i=1;i<=4;i++){const text=await shake({paw:'shake_right'});assert.equal(text.includes('pawtographs'),i===4);}
});

test('accepted handshake streak survives ordinary between-turn input playback cancellation but resets on Stop',async()=>{
 const e=service('conversation'),shake=handshakes(e);
 for(let i=0;i<3;i++){await shake();e.s.stopPlayback();}
 assert.match(await shake(),/pawtographs/);
 e.s.pauseRobotTools();e.s.robotToolsPaused=false;
 assert.equal(await shake(),'Left paw command queued.');
});

test('pre-response chain speech is entirely muted while accepted request bookkeeping stays nonphysical',async()=>{
 const e=service('conversation'),shake=handshakes(e);
 for(let i=0;i<3;i++)assert.equal(await shake({preamble:true}),undefined);
 assert.deepEqual(e.audio,[]);
 assert.match(await shake(),/pawtographs/);assert.deepEqual(e.audio,['handshake4']);
});

test('chain permit waits for the final actual reply and rejects failures, unknowns, wrong actions and duplicate IDs',async()=>{
 for(const bad of ['pending','failed','unknown','wrong-action','duplicate','extra-move']){
  const e=service('conversation'),d=deferred();e.s.robotBridge.callTool=async name=>name==='robot_status'?{success:true}:name==='interactive_action'?(bad==='pending'?d.promise:bad==='failed'?{success:false}:bad==='unknown'?{success:false,outcome_unknown:true}:queuedHandshake):queuedHandshake;
  await e.s.handleMessage(call('s','robot_status'));
  const actions=[{id:bad==='duplicate'?'s':'p',name:'interactive_action',args:{action:bad==='wrong-action'?'stand_up':'shake_left'}},...(bad==='extra-move'?[{id:'m',name:'move',args:{}}]:[])];
  const pending=e.s.handleMessage({toolCall:{functionCalls:actions}});
  if(bad==='pending'){await e.s.handleMessage(nativeReply('Left paw command queued.','pre-reply'));d.resolve(queuedHandshake);}
  await pending;await e.s.handleMessage(nativeReply('Left paw command queued.','unapproved'));
  assert.deepEqual(e.audio,[],bad);assert.equal(e.responses.at(-1).functionResponses.response.operator_acknowledgment,undefined,bad);
 }
});

test('Stop/cancel invalidates a final pending chain response and cannot replay the fourth joke later',async()=>{
 for(const action of ['stop','cancel']){
  const e=service('conversation'),shake=handshakes(e);for(let i=0;i<3;i++)await shake();
  const d=deferred();e.s.robotBridge.callTool=name=>name==='robot_status'?Promise.resolve({success:true}):d.promise;
  await e.s.handleMessage(call('status-final','robot_status'));const pending=e.s.handleMessage(call('paw-final','interactive_action',{action:'shake_left'}));
  if(action==='stop')e.s.pauseRobotTools();else await e.s.handleMessage({toolCallCancellation:{ids:['paw-final']}});
  d.resolve(queuedHandshake);await pending;await e.s.handleMessage(nativeReply('Left paw command queued. At this rate, I should charge for pawtographs.','stale'));
  assert.deepEqual(e.audio,['handshake1','handshake2','handshake3']);
 }
});

test('actual service admits complete screened generation before turn end, without replaying its tail',async()=>{
 const e=service('conversation');
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'hello'}}]},outputTranscription:{text:'Hello there'},generationComplete:true}});
 await new Promise(r=>setImmediate(r));assert.deepEqual(e.audio,['hello']);
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'late'}}]},outputTranscription:{text:'late'},generationComplete:true}});
 await e.s.handleMessage({serverContent:{turnComplete:true}});assert.deepEqual(e.audio,['hello']);
 await e.s.disconnect();
});
test('actual service pending HTTP cannot authorize native output after generation has sealed',async()=>{
 const e=service('conversation'),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
 const pending=e.s.handleMessage(call('pending','move'));
 await e.s.handleMessage({serverContent:{generationComplete:true}});
 d.resolve({success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}});await pending;
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'late-receipt'}}]},outputTranscription:{text:'Move queued.'},turnComplete:true}});
 assert.deepEqual(e.audio,[]);assert.equal(e.logs.filter(x=>x.type==='receipt').length,1,'actual server receipt remains visible');
 await e.s.disconnect();
});
test('Stop during generation-sealed playback invalidates deferred later chunks and preserves dispatch',async()=>{
 const e=service('conversation'),decoded=deferred();let stopped=0;
 e.s.playAudioBase64=async data=>{if(data==='first')await decoded.promise;else e.audio.push(data);};
 await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'first'}},{inlineData:{data:'must-not-play'}}]},outputTranscription:{text:'Hello there'},generationComplete:true}});
 e.s.robotBridge.callTool=async()=>{stopped++;return {success:true};};await e.s.handleMessage(call('stop','stop'));assert.equal(stopped,1);
 decoded.resolve();await new Promise(r=>setImmediate(r));assert.deepEqual(e.audio,[]);
 await e.s.handleMessage({serverContent:{turnComplete:true}});await e.s.disconnect();
});

test('new PTT after generation seal preserves the accepted-handshake streak while Stop still resets it',async()=>{
 const e=service('conversation');e.s.config.enableAudio=true;e.s.inputAudioContext={state:'running'};
 e.s.mediaStream={getAudioTracks:()=>[{enabled:false}]};e.s.sessionPromise=Promise.resolve({sendToolResponse:r=>e.responses.push(r),sendRealtimeInput(){}});
 e.s.robotBridge.callTool=async name=>name==='robot_status'?{success:true}:queuedHandshake;
 async function shake(i){
  await e.s.handleMessage(call('s'+i,'robot_status'));
  await e.s.handleMessage(call('p'+i,'interactive_action',{action:'shake_left'}));
  const ack=e.responses.at(-1).functionResponses.response.operator_acknowledgment.text;
  await e.s.handleMessage({serverContent:{modelTurn:{parts:[{inlineData:{data:'paw'+i}}]},outputTranscription:{text:ack},generationComplete:true}});
  assert.equal(e.s.beginAudioInput(),true);e.s.endAudioInput();
  await e.s.handleMessage({serverContent:{turnComplete:true}});return ack;
 }
 for(let i=1;i<=4;i++)assert.equal((await shake(i)).includes('pawtographs'),i===4);
 e.s.pauseRobotTools();e.s.robotToolsPaused=false;
 assert.equal((await shake(5)).includes('pawtographs'),false);
});
