import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { build } from 'esbuild';
const result=await build({entryPoints:['services/conversationOutput.ts'],bundle:true,write:false,format:'esm',platform:'node'});
const {ConversationOutputGate,NativeReceiptOutputGate,normalizeReceiptTranscript,screenConversation,OUTPUT_LIMITS,BLOCKED_CONVERSATION}=await import('data:text/javascript;base64,'+Buffer.from(result.outputFiles[0].text).toString('base64'));
const fixture=JSON.parse(readFileSync('../conversation_guard.json','utf8'));
const packet=(text='Hello there',audio='native',complete=true)=>({serverContent:{modelTurn:{parts:[{inlineData:{data:audio}}]},outputTranscription:{text},turnComplete:complete}});
function setup(t){const released=[],notices=[];const gate=new ConversationOutputGate(x=>released.push(x),x=>notices.push(x));t.after(()=>gate.close());return {gate,released,notices};}
test('all shared English examples agree with the best-effort screen',()=>{for(const {text,blocked} of fixture.examples)assert.equal(screenConversation(text),!blocked,text);});
test('normalization handles compatibility characters, curly quotes and whitespace',()=>{
 assert.equal(screenConversation('Ｉ’ve   just stopped.'),false);
 assert.equal(screenConversation('Hello!\n How is your day?'),true);
});
test('generationComplete and transcript.finished never release before turnComplete',t=>{
 const e=setup(t);e.gate.observe({...packet('Hello','data',false),serverContent:{...packet('Hello','data',false).serverContent,generationComplete:true,outputTranscription:{text:'Hello',finished:true}}},false,false);
 assert.equal(e.released.length,0);e.gate.observe({serverContent:{turnComplete:true}},false,false);assert.equal(e.released.length,1);
});
test('byte, item and text bounds drop the entire turn and its continuation',async t=>{
 for(const kind of ['bytes','items','text'])await t.test(kind,t=>{
  const e=setup(t);
  if(kind==='bytes')e.gate.observe(packet('Hello','x'.repeat(OUTPUT_LIMITS.bytes+1),false),false,false);
  if(kind==='items')for(let i=0;i<=OUTPUT_LIMITS.items;i++)e.gate.observe({serverContent:{modelTurn:{parts:[{inlineData:{data:'x'}}]}}},false,false);
  if(kind==='text')e.gate.observe({serverContent:{modelTurn:{parts:[{text:'x'.repeat(OUTPUT_LIMITS.text+1)}]}}},false,true);
  e.gate.observe(packet('Benign tail','tail'),false,false);assert.equal(e.released.length,0);assert.equal(e.notices.length,1);
  e.gate.observe(packet(),false,false);assert.equal(e.released.length,1);
 });
});
test('missing boundary timeout quarantines the tail and permits the next turn',t=>{
 t.mock.timers.enable({apis:['setTimeout']});const e=setup(t);e.gate.observe(packet('Hello','first',false),false,false);
 t.mock.timers.tick(OUTPUT_LIMITS.timeoutMs);e.gate.observe(packet('Tail','tail'),false,false);assert.equal(e.released.length,0);assert.match(e.notices[0],/boundary/);
 e.gate.observe(packet(),false,false);assert.equal(e.released.length,1);
});
test('signal-only interruption or cancellation creates a tombstone before any content',t=>{
 for(const signal of [{serverContent:{interrupted:true}},{toolCallCancellation:{ids:['old']}}]){
  const e=setup(t);e.gate.observe(signal,false,false);e.gate.observe(packet(),false,false);assert.equal(e.released.length,0);
  e.gate.observe(packet(),false,false);assert.equal(e.released.length,1);
 }
});
test('unresolved tool contaminates a newly arriving output turn until its boundary',t=>{
 const e=setup(t);e.gate.observe(packet(),true,false);assert.equal(e.released.length,0);e.gate.observe(packet(),false,false);assert.equal(e.released.length,1);
});
test('split claim text and missing native transcript are refused, and thought text is never displayed',t=>{
 const e=setup(t);for(const text of ['I have que','ued that.'])e.gate.observe({serverContent:{modelTurn:{parts:[{text}]}}},false,true);
 e.gate.observe({serverContent:{turnComplete:true}},false,true);assert.equal(e.released.length,0);assert.equal(e.notices[0],BLOCKED_CONVERSATION);
 e.gate.observe(packet('','data'),false,false);assert.equal(e.released.length,0);
 e.gate.observe({serverContent:{modelTurn:{parts:[{text:'private reasoning',thought:true}]},turnComplete:true}},false,true);assert.equal(e.released.length,0);
});


test('native receipt equality permits boundary punctuation but preserves decimal and sign meaning',()=>{
 assert.equal(normalizeReceiptTranscript('Move queued!'),normalizeReceiptTranscript('move queued.'));
 assert.notEqual(normalizeReceiptTranscript('Speed0.2'),normalizeReceiptTranscript('Speed02'));
 assert.notEqual(normalizeReceiptTranscript('Speed -0.2'),normalizeReceiptTranscript('Speed0.2'));
});

test('native receipt timeout and size limits fail closed, then a fresh single-tool turn can succeed',async t=>{
 for(const limit of ['time','bytes','items','text'])await t.test(limit,t=>{
  if(limit==='time')t.mock.timers.enable({apis:['setTimeout']});
  const emitted=[],gate=new NativeReceiptOutputGate(a=>emitted.push(a));t.after(()=>gate.close());
  const tool={toolCall:{functionCalls:[{id:'a',name:'move'}]}};
  const turn=gate.observe(tool);assert.equal(gate.authorize(turn,'Move queued.'),true);
  if(limit==='time')t.mock.timers.tick(OUTPUT_LIMITS.timeoutMs);
  if(limit==='bytes')gate.observe(packet('','a'.repeat(OUTPUT_LIMITS.bytes+1),false));
  if(limit==='items')for(let i=0;i<=OUTPUT_LIMITS.items;i++)gate.observe(packet('','x',false));
  if(limit==='text')gate.observe({serverContent:{outputTranscription:{text:'x'.repeat(OUTPUT_LIMITS.text+1)}}});
  gate.observe(packet('Move queued.','bad'));assert.deepEqual(emitted,[]);
  const next=gate.observe(tool);assert.equal(gate.authorize(turn,'Move queued.'),false);
  assert.equal(gate.authorize(next,'Move queued.'),true);gate.observe(packet('Move queued.','good'));assert.deepEqual(emitted,[['good']]);
 });
});
