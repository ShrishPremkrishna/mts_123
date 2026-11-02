import test from 'node:test';import assert from 'node:assert/strict';import {readFileSync} from 'node:fs';import {build} from 'esbuild';
const b=await build({stdin:{contents:"export {GeminiLiveService} from './services/geminiLive';export {ReceiptAudioCache,serverReceiptAudioText} from './services/receiptAudioCache';",resolveDir:process.cwd(),loader:'ts'},bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'sdk',setup(x){x.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));x.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI {} export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {GeminiLiveService,ReceiptAudioCache}=await import('data:text/javascript;base64,'+Buffer.from(b.outputFiles[0].text).toString('base64'));
const manifest=JSON.parse(readFileSync(new URL('../services/receiptAudioManifest.json',import.meta.url)));
const ready=await ReceiptAudioCache.load(manifest,async file=>new Uint8Array(readFileSync(new URL('../public/receipt-audio/'+file,import.meta.url))));
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {resolve,promise};};
function setup(){const audio=[],responses=[],calls=[];const s=new GeminiLiveService({modelName:'gemini-3.1-flash-live-preview',voiceName:'Puck',outputPolicy:'conversation',robotApiUrl:'http://approved-receipts'},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},ready);s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:r=>responses.push(r),sendRealtimeInput(){}});s.playAudioBase64=async x=>audio.push(x);s.robotBridge.callTool=async n=>{calls.push(n);return queued;};s.robotBridge.immediateStop=async()=>({success:true});return {s,audio,responses,calls};}
const packet=(...calls)=>({toolCall:{functionCalls:calls.map((c,i)=>({id:'id'+i,...c}))}});
const expected=text=>ready.lookup(text,'gemini-3.1-flash-live-preview','Puck');
test('approved12 assets: each full five-tool batch cue follows its own actual server reply',async()=>{
 const e=setup(),pending=[];e.s.robotBridge.callTool=name=>{e.calls.push(name);const d=deferred();pending.push(d);return d.promise;};
 const done=e.s.handleMessage(packet({name:'play_trick',args:{name:'sit_stay_shake',count:1,count_unit:'routine_playback'}},{name:'wait',args:{duration:10}},{name:'interactive_action',args:{action:'shake_left'}},{name:'interactive_action',args:{action:'shake_right'}},{name:'stand_up'}));
 const texts=['Trick queued.','Wait queued.','Left paw command queued.','Right paw command queued.','Stand queued.'];
 for(let i=0;i<5;i++){assert.equal(e.audio.length,i,'no cue before its reply');pending[i].resolve(queued);for(let n=0;n<5;n++)await Promise.resolve();assert.equal(e.audio[i],expected(texts[i]));}
 await done;assert.equal(e.responses.length,5);
 await e.s.handleMessage({serverContent:{outputTranscription:{text:'I completed everything.'},modelTurn:{parts:[{inlineData:{data:'false-narration'}}]},turnComplete:true}});assert.equal(e.audio.length,5,'whole-turn model prose remains muted');
});
test('known failures use approved fixed audio only after response; exact quantity overrides generic',async()=>{
 for(const [result,text] of [[{success:false,details:{refusal:{schema:'pupper_motion_refusal_v1',code:'minimum_speed',limits:{forward_min:.2,sideways_min:.2,turn_min:20}}}},'That request failed. Check the controls for details.'],[{success:false,details:{refusal:{schema:'pupper_trick_quantity_refusal_v1',code:'physical_count_unsupported'}}},'Exact physical repetition counts are not supported.']]){
  const e=setup(),d=deferred();e.s.robotBridge.callTool=()=>d.promise;const p=e.s.handleMessage(packet({name:text.startsWith('Exact')?'play_trick':'move',args:{name:'push_up'}}));assert.deepEqual(e.audio,[]);d.resolve(result);await p;assert.deepEqual(e.audio,[expected(text)]);assert.equal(e.s.robotToolsPaused,true);
 }
});
test('unknown, stale, malformed or unsent failure never uses generic known-failure cache',async()=>{
 for(const result of [{success:false,outcome_unknown:true},{success:false,superseded:true},{success:false,not_dispatched:true},{success:false,outcome_unknown:{}},{message:'failed'}]){const e=setup();e.s.robotBridge.callTool=async()=>result;await e.s.handleMessage(packet({name:'move'}));assert.deepEqual(e.audio,[]);}
});
