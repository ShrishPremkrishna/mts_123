import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const built=await build({entryPoints:['services/conversationOutput.ts'],bundle:true,write:false,format:'esm',platform:'node'});
const {ConversationOutputGate,NativeReceiptOutputGate}=await import('data:text/javascript;base64,'+Buffer.from(built.outputFiles[0].text).toString('base64'));
const audio=(text='Hello there',data='voice',extra={})=>({serverContent:{modelTurn:{parts:[{inlineData:{data}}]},outputTranscription:{text},...extra}});
const gen=()=>({serverContent:{generationComplete:true}}),end=()=>({serverContent:{turnComplete:true}});
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
function conversation(t){const emitted=[],notices=[],g=new ConversationOutputGate(x=>emitted.push({audio:[...x.audio],text:[...x.text],transcript:x.transcript}),x=>notices.push(x));t.after(()=>g.close());return {g,emitted,notices};}
function receipts(t){const emitted=[],g=new NativeReceiptOutputGate(x=>emitted.push([...x]));t.after(()=>g.close());return {g,emitted};}
function begin(g,id='m',name='move',args={}){const c={id,name,args};return {c,turn:g.observe({toolCall:{functionCalls:[c]}})};}
test('complete generation releases full screened conversation once, retaining a tombstone through turn end',t=>{
 const e=conversation(t);e.g.observe(audio(),false,false);assert.equal(e.emitted.length,0);e.g.observe(gen(),false,false);assert.equal(e.emitted.length,1);
 e.g.observe(audio('Late tail','late',{generationComplete:true}),false,false);e.g.observe(gen(),false,false);assert.equal(e.emitted.length,1);
 e.g.observe(end(),false,false);e.g.observe(end(),false,false);assert.equal(e.emitted.length,1);
 e.g.observe(audio('A new greeting','fresh',{generationComplete:true,turnComplete:true}),false,false);assert.deepEqual(e.emitted.map(x=>x.audio),[['voice'],['fresh']]);
});
test('same packet appends final content before generation seal; finished alone is insufficient',t=>{
 const e=conversation(t);e.g.observe({serverContent:{outputTranscription:{text:'Hello ',finished:true}}},false,false);assert.equal(e.emitted.length,0);
 e.g.observe(audio('there','voice',{generationComplete:true}),false,false);assert.equal(e.emitted[0].transcript,'Hello there');e.g.observe(end(),false,false);assert.equal(e.emitted.length,1);
});
test('pending tools, false claims and missing transcript remain closed at generation boundary',t=>{
 for(const kind of ['pending','claim','missing']){const e=conversation(t);e.g.observe(audio(kind==='claim'?'I have moved.':kind==='missing'?'':'Hello','voice',{generationComplete:true}),kind==='pending',false);assert.equal(e.emitted.length,0,kind);e.g.observe(audio('Safe late tail','late',{turnComplete:true}),false,false);assert.equal(e.emitted.length,0,kind);}
});
test('empty generation and pre-output interruption quarantine continuation tails until turn end',t=>{
 for(const signal of [gen(),{serverContent:{interrupted:true}},{toolCallCancellation:{ids:['old']}}]){const e=conversation(t);e.g.observe(signal,false,false);e.g.observe(audio('Tail','late',{generationComplete:true}),false,false);assert.equal(e.emitted.length,0);e.g.observe(end(),false,false);e.g.observe(audio('Fresh','fresh',{turnComplete:true}),false,false);assert.equal(e.emitted.length,1);}
});
test('native exact receipt seals after settled reply, ignores late content and keeps normal fallback',t=>{
 const e=receipts(t);const a=begin(e.g);assert.equal(e.g.authorize(a.turn,'Move queued.',a.c,queued),'Move queued.');e.g.observe(audio('Move queued.','first',{generationComplete:true}));assert.deepEqual(e.emitted,[['first']]);
 assert.equal(e.g.authorize(a.turn,'Move queued.',a.c,queued),null);e.g.observe(audio('Move queued.','tail',{generationComplete:true}));e.g.observe(end());e.g.observe(end());assert.deepEqual(e.emitted,[['first']]);
 const b=begin(e.g,'next');e.g.authorize(b.turn,'Move queued.',b.c,queued);e.g.observe(audio('Move queued.','fallback',{turnComplete:true}));assert.deepEqual(e.emitted,[['first'],['fallback']]);
});
test('native pending reply at generation closure never authorizes its later tail',t=>{
 const e=receipts(t);const a=begin(e.g);e.g.observe(gen());assert.equal(e.g.authorize(a.turn,'Move queued.',a.c,queued),null);e.g.observe(audio('Move queued.','late',{turnComplete:true}));assert.deepEqual(e.emitted,[]);
 const b=begin(e.g,'fresh');e.g.authorize(b.turn,'Move queued.',b.c,queued);e.g.observe(audio('Move queued.','good',{generationComplete:true}));assert.deepEqual(e.emitted,[['good']]);
});
test('native pre-response, cancellation, malformed and mismatched output still cannot speak',t=>{
 for(const kind of ['pre','cancel','malformed','mismatch']){const e=receipts(t),a=begin(e.g);if(kind==='pre')e.g.observe(audio('Move queued.','early'));if(kind==='cancel')e.g.observe({toolCallCancellation:{ids:['m']}});e.g.authorize(a.turn,'Move queued.',a.c,kind==='malformed'?{success:true,outcome_unknown:{}}:queued);e.g.observe(audio(kind==='mismatch'?'Move completed.':'Move queued.','bad',{generationComplete:true}));e.g.observe(end());assert.deepEqual(e.emitted,[],kind);}
});
test('generation plus final turn boundary counts accepted handshake exactly once',t=>{
 const e=receipts(t);
 for(let i=0;i<4;i++){
  const s=begin(e.g,'status'+i,'robot_status');e.g.authorize(s.turn,'Status.',s.c,{success:true});
  const p=begin(e.g,'paw'+i,'interactive_action',{action:'shake_left'});const ack=e.g.authorize(p.turn,'Left paw command queued.',p.c,queued);
  assert.equal(ack.includes('pawtographs'),i===3,'joke only on fourth accepted request');
  e.g.observe(audio(ack,'paw'+i,{generationComplete:true}));e.g.observe(gen());e.g.observe(end());e.g.observe(end());
 }
 assert.equal(e.emitted.length,4);
});
test('Stop or new-input discard after seal keeps following genuine turn usable',t=>{
 const e=conversation(t);e.g.observe(audio('First','one',{generationComplete:true}),false,false);e.g.discardPending();e.g.observe(audio('Late','late',{generationComplete:true}),false,false);e.g.observe(end(),false,false);e.g.observe(audio('Next','two',{generationComplete:true}),false,false);assert.deepEqual(e.emitted.map(x=>x.audio),[['one'],['two']]);
});
