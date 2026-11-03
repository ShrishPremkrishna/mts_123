import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { build } from 'esbuild';
const bundle=await build({stdin:{contents:"export { preloadReceiptAudio, supportsQuickReceipts, RECEIPT_PRELOAD_TIMEOUT_MS } from './services/receiptAudioPreload'; export { GeminiLiveService } from './services/geminiLive'; export { operatorFeedback,isOperatorReceiptNarration } from './services/operatorFeedback';",resolveDir:process.cwd(),loader:'ts'},bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'offline-sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI { constructor(){this.live={connect:x=>globalThis.__cacheSdkConnect(x)}} } export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {preloadReceiptAudio,supportsQuickReceipts,RECEIPT_PRELOAD_TIMEOUT_MS,GeminiLiveService,operatorFeedback,isOperatorReceiptNarration}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const manifest=JSON.parse(fs.readFileSync('services/receiptAudioManifest.json','utf8'));
const config={quickReceipts:true,outputPolicy:'conversation',modelName:manifest.model,voiceName:manifest.voice,enableAudio:false,enableVideo:false,extraInstructions:'',robotApiUrl:'http://preload-tests'};
const asset=file=>new Uint8Array(fs.readFileSync('public/receipt-audio/'+file));
const response=data=>new Response(data,{headers:{'content-length':String(data.byteLength)}});
const fetchAssets=async url=>response(asset(decodeURIComponent(String(url).split('/').at(-1))));
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};};
function replace(t,key,value){const old=Object.getOwnPropertyDescriptor(globalThis,key);Object.defineProperty(globalThis,key,{configurable:true,writable:true,value});t.after(()=>old?Object.defineProperty(globalThis,key,old):delete globalThis[key]);}

test('approved bundled manifest preloads exactly fourteen pinned local PCM files',async()=>{
  const requests=[],cache=await preloadReceiptAudio(config,undefined,async(url,options)=>{requests.push(url);assert.ok(options.signal);return fetchAssets(url);});
  assert.ok(cache);assert.equal(requests.length,14);assert.ok(requests.every(x=>x.startsWith('/receipt-audio/')&&x.endsWith('.pcm')));
  for(const clip of manifest.clips){assert.equal(clip.review_status,'approved_human');assert.equal(cache.lookup(clip.text,config.modelName,'Puck'),Buffer.from(asset(clip.file)).toString('base64'));}
});
test('disabled, mismatched model/voice and receipt-only settings perform no preload',async()=>{
  for(const patch of [{quickReceipts:false},{quickReceipts:undefined},{quickReceipts:'true'},{voiceName:'Kore'},{modelName:'gemini-2.5-flash-native-audio-preview'},{outputPolicy:'receipts'}]) {
    let calls=0;const c={...config,...patch};assert.equal(supportsQuickReceipts(c),false);assert.equal(await preloadReceiptAudio(c,undefined,async()=>{calls++;throw Error('must not fetch');}),null);assert.equal(calls,0);
  }
});
test('missing, corrupt, truncated and oversized assets cleanly select native fallback',async()=>{
  for(const kind of ['missing','corrupt','truncated','oversize','wrong-length']) {
    let cancelled=false;
    const loaded=await preloadReceiptAudio(config,undefined,async()=>{
      if(kind==='missing')return new Response('',{status:404});
      const bytes=asset(manifest.clips[0].file);
      if(kind==='corrupt'){bytes[0]^=1;return response(bytes);}
      if(kind==='truncated')return new Response(bytes.subarray(0,100));
      if(kind==='wrong-length')return new Response(bytes,{headers:{'content-length':'2'}});
      return new Response(new ReadableStream({start(c){c.enqueue(new Uint8Array(bytes.length+1));},cancel(){cancelled=true;}}));
    });assert.equal(loaded,null,kind);if(kind==='oversize')assert.equal(cancelled,true);
  }
});
test('one connect preload deadline and external cancellation abort even a nonsettling fetch',async t=>{
  t.mock.timers.enable({apis:['setTimeout']});
  let signal;const load=preloadReceiptAudio(config,undefined,async(_,options)=>{signal=options.signal;return new Promise(()=>{});});
  t.mock.timers.tick(RECEIPT_PRELOAD_TIMEOUT_MS);assert.equal(await load,null);assert.equal(signal.aborted,true);
  const controller=new AbortController();const again=preloadReceiptAudio(config,controller.signal,async()=>new Promise(()=>{}));controller.abort();assert.equal(await again,null);
});

function environment(t,fetcher) {
  const calls=[],logs=[],audio=[],responses=[];let connects=0;
  class Context {constructor(){this.state='running';this.currentTime=0;}createAnalyser(){return {connect(){},getByteTimeDomainData(x){x.fill(128);}};}async close(){this.state='closed';}}
  replace(t,'window',{AudioContext:Context,setInterval,location:{origin:'http://preload-tests'}});replace(t,'fetch',fetcher);
  replace(t,'__cacheSdkConnect',async setup=>{connects++;setup.callbacks.onopen();return {sendToolResponse:x=>responses.push(x),close(){}};});
  const s=new GeminiLiveService(config,x=>logs.push(x),()=>{},()=>{});s.robotBridge.getTools=async()=>({prompt:'Test',tools:[{name:'move'},{name:'stop'},{name:'play_trick'}]});
  s.robotBridge.callTool=async(name,args)=>{calls.push({name,args});return {success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};};s.robotBridge.immediateStop=async()=>({success:true});s.playAudioBase64=async x=>audio.push(x);
  t.after(()=>s.disconnect());return {s,calls,logs,audio,responses,get connects(){return connects;}};
}
test('real service connect preloads once; cache dispatch does not fetch or wait for model audio',async t=>{
  let downloads=0;const e=environment(t,async(...args)=>{downloads++;return fetchAssets(...args);});await e.s.connect('UNIT_TEST',config,{});assert.equal(downloads,14);assert.equal(e.connects,1);
  await e.s.handleMessage({toolCall:{functionCalls:[{id:'m',name:'move',args:{}}]}});assert.equal(e.audio.length,1);assert.equal(downloads,14);assert.equal(e.responses[0].functionResponses.response.operator_acknowledgment,undefined);
  assert.ok(e.logs.some(x=>x.message==='Quick Puck acknowledgments ready.'));
});
test('connect fallback preserves normal native receipt and disconnect cancels preload before SDK opens',async t=>{
  const e=environment(t,async()=>new Response('',{status:404}));await e.s.connect('UNIT_TEST',config,{});await e.s.handleMessage({toolCall:{functionCalls:[{id:'m',name:'move',args:{}}]}});
  assert.deepEqual(e.audio,[]);assert.equal(e.responses[0].functionResponses.response.operator_acknowledgment.text,'Move queued.');assert.ok(e.logs.some(x=>x.message.includes('normal model reply')));
  await e.s.disconnect();
  const pending=environment(t,async()=>new Promise(()=>{}));const connect=pending.s.connect('UNIT_TEST',config,{});await pending.s.disconnect();await assert.rejects(connect);assert.equal(pending.connects,0);
});
test('typed quantity failure remains closed wording and fences later model motion until reconnect',async t=>{
  const e=environment(t,fetchAssets);await e.s.connect('UNIT_TEST',config,{});let dispatches=0;
  const refused={success:false,command_epoch_required:true,details:{refusal:{schema:'pupper_trick_quantity_refusal_v1',code:'physical_count_unsupported'}}};
  e.s.robotBridge.callTool=async name=>{dispatches++;return name==='play_trick'?refused:{success:true};};
  const tool=(id,name,args={})=>({toolCall:{functionCalls:[{id,name,args}]}});
  await e.s.handleMessage(tool('ten','play_trick',{name:'push_up',count:10,count_unit:'physical_repetition'}));
  assert.equal(e.s.robotToolsPaused,true);assert.equal(e.logs.filter(x=>x.type==='receipt').at(-1).message,'Exact physical repetition counts are not supported.');
  await e.s.handleMessage(tool('retry','play_trick',{name:'push_up',count:1,count_unit:'routine_playback'}));assert.equal(dispatches,1);assert.equal(e.responses.at(-1).functionResponses.response.result.not_dispatched,true);
  await e.s.handleMessage(tool('status','robot_status'));await e.s.handleMessage(tool('stop','stop'));assert.equal(dispatches,3);
  assert.equal(operatorFeedback('play_trick',{...refused,outcome_unknown:true}),'I could not confirm that request.');assert.equal(isOperatorReceiptNarration(operatorFeedback('play_trick',refused)),true);
});
