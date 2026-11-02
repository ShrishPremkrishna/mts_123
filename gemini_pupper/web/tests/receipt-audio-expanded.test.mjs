import test from 'node:test';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { build } from 'esbuild';
const result=await build({entryPoints:['services/receiptAudioCache.ts'],bundle:true,write:false,format:'esm',platform:'node'});
const {ReceiptAudioCache,serverReceiptAudioText}=await import('data:text/javascript;base64,'+Buffer.from(result.outputFiles[0].text).toString('base64'));
const phrases=['Move queued.','Stop requested.','Left paw command queued.','Right paw command queued.','Stand queued.','Turn queued.','Arc queued.','Wait queued.','Trick queued.','Interactive action queued.','That request failed. Check the controls for details.','Exact physical repetition counts are not supported.'];
const pcm=Uint8Array.from([1,0,2,0]),sha=createHash('sha256').update(pcm).digest('hex');
// Synthetic waveform fixtures exercise review flags; they are not reviewed speech.
const manifest=()=>({schema:'pupper_receipt_audio_v1',model:'gemini-3.1-flash-live-preview',voice:'Puck',encoding:'pcm_s16le',sample_rate_hz:24000,channels:1,clips:phrases.map((text,i)=>({text,file:'clip-'+i+'.pcm',sha256:sha,bytes:pcm.length,duration_s:pcm.length/48000,transcript:text,review_status:i<4?'approved_human':'pending_human'}))});
test('twelve closed phrases load but the eight new pending clips cannot play',async()=>{
 const cache=await ReceiptAudioCache.load(manifest(),async()=>pcm);
 for(let i=0;i<phrases.length;i++)assert.equal(cache.lookup(phrases[i],'gemini-3.1-flash-live-preview','Puck'),i<4?Buffer.from(pcm).toString('base64'):null);
});
test('approved synthetic flags permit only the exact twelve phrases in the exact voice/model',async()=>{
 const m=manifest();for(const c of m.clips)c.review_status='approved_human';const cache=await ReceiptAudioCache.load(m,async()=>pcm);
 for(const text of phrases){assert.equal(cache.lookup(text,'gemini-3.1-flash-live-preview','Puck'),Buffer.from(pcm).toString('base64'));assert.equal(cache.lookup(text,'gemini-3.1-flash-live-preview','Kore'),null);}
 assert.equal(cache.lookup('Stand completed.','gemini-3.1-flash-live-preview','Puck'),null);
});
test('unknown phrases, duplicate entries and more than twelve clips are rejected',async()=>{
 for(const mutate of [m=>m.clips[4].text='Standing physically still.',m=>m.clips[4]=m.clips[0],m=>m.clips.push({...m.clips[0],file:'extra.pcm'})]){const m=manifest();mutate(m);await assert.rejects(ReceiptAudioCache.load(m,async()=>pcm));}
});


test('only known current-result shapes select fixed accepted, queued or failure phrases',()=>{
 const q={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
 for(const [name,text] of Object.entries({stand_up:'Stand queued.',move:'Move queued.',turn:'Turn queued.',arc:'Arc queued.',wait:'Wait queued.',play_trick:'Trick queued.',interactive_action:'Interactive action queued.'}))assert.equal(serverReceiptAudioText(name,{},q),text);
 assert.equal(serverReceiptAudioText('interactive_action',{action:'shake_left'},q),'Left paw command queued.');
 assert.equal(serverReceiptAudioText('interactive_action',{action:'shake_right'},q),'Right paw command queued.');
 assert.equal(serverReceiptAudioText('stop',{}, {success:true}),'Stop requested.');
 for(const x of [{success:true},{success:true,details:{submission:{schema:'other',state:'queued'}}},{success:true,details:{submission:{schema:'pupper_submission_v1',state:'completed'}}}])assert.equal(serverReceiptAudioText('move',{},x),null);
 for(const name of ['robot_status','unlock','lie_down','__proto__','constructor'])assert.equal(serverReceiptAudioText(name,{},q),null);
 assert.equal(serverReceiptAudioText('move',{}, {success:false,message:'arbitrary prose'}),'That request failed. Check the controls for details.');
 const refusal={success:false,details:{refusal:{schema:'pupper_trick_quantity_refusal_v1',code:'physical_count_unsupported'}}};
 assert.equal(serverReceiptAudioText('play_trick',{},refusal),'Exact physical repetition counts are not supported.');
 assert.equal(serverReceiptAudioText('move',{},refusal),'That request failed. Check the controls for details.');
});
test('unknown, malformed, superseded and unsent results never become known failure clips',()=>{
 for(const flag of ['outcome_unknown','superseded','not_dispatched'])for(const value of [true,{},[],1,'true',null])for(const success of [true,false])assert.equal(serverReceiptAudioText('move',{}, {success,[flag]:value}),null);
 for(const result of [null,{},[],{success:1},{success:'false'}])assert.equal(serverReceiptAudioText('move',{},result),null);
});
