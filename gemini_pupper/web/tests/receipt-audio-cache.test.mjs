import test from 'node:test';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { build } from 'esbuild';

const bundle = await build({ stdin: { contents: "export { GeminiLiveService } from './services/geminiLive'; export { ReceiptAudioCache } from './services/receiptAudioCache';", resolveDir: process.cwd(), loader: 'ts' }, bundle: true, write: false, format: 'esm', platform: 'node', plugins: [{ name: 'offline-sdk', setup(b) {
  b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));
  b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI {} export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));
} }] });
const {GeminiLiveService,ReceiptAudioCache}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const phrases=['Move queued.','Stop requested.','Left paw command queued.','Right paw command queued.'];
const pcm=Uint8Array.from([1,0,2,0]),pcm64=Buffer.from(pcm).toString('base64');
const digest=createHash('sha256').update(pcm).digest('hex');
// These tiny synthetic unit fixtures test flags and lifecycle, not waveform review.
function manifest(review_status='approved_human') { return {schema:'pupper_receipt_audio_v1',model:'gemini-3.1-flash-live-preview',voice:'Puck',encoding:'pcm_s16le',sample_rate_hz:24000,channels:1,clips:phrases.map((text,i)=>({text,file:`clip-${i}.pcm`,sha256:digest,bytes:4,duration_s:4/48000,transcript:text,review_status}))}; }
const cache=()=>ReceiptAudioCache.load(manifest(),async()=>pcm);
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
const call=(id,name,args={})=>({toolCall:{functionCalls:[{id,name,args}]}});
const speech=(text,data='model-audio',complete=true)=>({serverContent:{modelTurn:{parts:[{inlineData:{data}}]},outputTranscription:{text},turnComplete:complete}});
function deferred(){let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};}
function service(audioCache,patch={}) {
  const logs=[],responses=[],audio=[],calls=[];
  const config={modelName:'gemini-3.1-flash-live-preview',voiceName:'Puck',robotApiUrl:'http://cache-tests',outputPolicy:'conversation',...patch};
  const s=new GeminiLiveService(config,x=>logs.push(x),()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},audioCache);
  s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:r=>responses.push(r)});
  s.playAudioBase64=async data=>audio.push(data);
  s.robotBridge.callTool=async name=>{calls.push(name);return name==='stop'||name==='robot_status'?{success:true}:queued;};
  s.robotBridge.immediateStop=async()=>({success:true});
  return {s,logs,responses,audio,calls};
}

test('cache preload verifies bounded exact PCM, transcript, identity and hashes',async()=>{
  const ready=await cache();assert.equal(ready.lookup('Move queued.','gemini-3.1-flash-live-preview','Puck'),pcm64);
  assert.equal(ready.lookup('Move queued.','models/gemini-3.1-flash-live-preview','Puck'),pcm64);
  for(const bad of [m=>m.model='other',m=>m.voice='Kore',m=>m.encoding='wav',m=>m.sample_rate_hz=16000,m=>m.channels=2,m=>m.clips.push(m.clips[0]),m=>m.clips[0].file='../secret.pcm',m=>m.clips[0].text='I physically stopped.',m=>m.clips[0].transcript='I completed the move.',m=>m.clips[0].sha256+='\n',m=>m.clips[0].bytes=480002,m=>m.clips[0].bytes=3,m=>m.clips[0].duration_s=10,m=>m.clips[0].review_status='claimed_review']) {
    const m=manifest();bad(m);await assert.rejects(ReceiptAudioCache.load(m,async()=>pcm));
  }
  await assert.rejects(ReceiptAudioCache.load(manifest(),async()=>Uint8Array.from([9,0,2,0])),/hash/);
  await assert.rejects(ReceiptAudioCache.load(manifest(),async()=>Uint8Array.from([1,0])),/size/);
});

test('cache defaults absent; pending waveform review, voice/model mismatch and missing phrase use original path',async()=>{
  const pending=await ReceiptAudioCache.load(manifest('pending_human'),async()=>pcm);
  for(const [ready,config] of [[undefined,{}],[pending,{}],[await cache(),{voiceName:'Kore'}],[await cache(),{modelName:'gemini-2.5-flash-native-audio-preview'}]]) {
    const e=service(ready,config);await e.s.handleMessage(call('m','move'));assert.deepEqual(e.audio,[]);
    assert.equal(e.responses[0].functionResponses.response.operator_acknowledgment.text,'Move queued.');
    await e.s.handleMessage(speech('Move queued.'));assert.deepEqual(e.audio,['model-audio']);
  }
  const e=service(await cache());await e.s.handleMessage(call('status','robot_status'));assert.deepEqual(e.audio,[]);
  await e.s.handleMessage(speech('Status request accepted.'));assert.deepEqual(e.audio,['model-audio']);
});

test('preload snapshots review metadata and PCM rather than retaining mutable caller ownership',async()=>{
  const m=manifest('pending_human'),owned=new Uint8Array(pcm);
  const loaded=await ReceiptAudioCache.load(m,async()=>{for(const item of m.clips)item.review_status='approved_human';return owned;});
  assert.equal(loaded.lookup('Move queued.','gemini-3.1-flash-live-preview','Puck'),null);
  const ready=await ReceiptAudioCache.load(manifest(),async()=>owned);owned.fill(99);
  assert.equal(ready.lookup('Move queued.','gemini-3.1-flash-live-preview','Puck'),pcm64);
});

test('cached receipt waits for actual accepted typed queue reply, then suppresses duplicate native and preserves chat',async()=>{
  const e=service(await cache()),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
  const pending=e.s.handleMessage(call('m','move'));assert.deepEqual(e.audio,[]);assert.equal(e.logs.some(x=>x.type==='receipt'),false);
  d.resolve(queued);await pending;assert.deepEqual(e.audio,[pcm64]);assert.equal(e.logs.find(x=>x.type==='receipt').message,'Move queued.');
  assert.equal(e.responses[0].functionResponses.response.operator_acknowledgment,undefined);
  await e.s.handleMessage(speech('Move queued.','duplicate'));assert.deepEqual(e.audio,[pcm64]);
  await e.s.handleMessage(speech('Hello friend!','chat'));assert.deepEqual(e.audio,[pcm64,'chat']);
});

test('failed, unknown, superseded, locally refused and merely accepted requests never use queued cache',async()=>{
  for(const result of [{success:false},{success:false,outcome_unknown:true},{success:true,superseded:true},{success:false,not_dispatched:true},{success:true},{...queued,outcome_unknown:{}},{success:true,details:{submission:{schema:'other',state:'queued'}}}]) {
    const e=service(await cache());e.s.robotBridge.callTool=async()=>result;await e.s.handleMessage(call('m','move'));assert.deepEqual(e.audio,[]);
  }
  const e=service(await cache(),{outputPolicy:'receipts'});await e.s.handleMessage(call('m','move'));assert.deepEqual(e.audio,[]);
});

test('Stop cache needs successful current Stop reply and never means physical stopping',async()=>{
  const e=service(await cache()),d=deferred();e.s.robotBridge.callTool=()=>d.promise;
  const pending=e.s.handleMessage(call('stop','stop'));assert.deepEqual(e.audio,[]);d.resolve({success:true});await pending;
  assert.deepEqual(e.audio,[pcm64]);assert.equal(e.logs.find(x=>x.type==='receipt').message,'Stop requested.');
  await e.s.handleMessage(speech('Stop requested.','duplicate'));assert.deepEqual(e.audio,[pcm64]);
});

test('pre-response audio, same-packet output, extra tools and stale response cannot enable cache',async()=>{
  for(const scenario of ['pre-response','same-packet','extra-tool','stop','cancel','disconnect','newer','boundary']) {
    const e=service(await cache()),d=deferred();e.s.robotBridge.callTool=name=>name==='move'?d.promise:Promise.resolve({success:true});
    let packet=call('old','move');if(scenario==='same-packet')packet={...packet,...speech('Move queued.','pre',false)};
    if(scenario==='extra-tool')packet={toolCall:{functionCalls:[{id:'old',name:'move'},{id:'extra',name:'robot_status'}]}};
    const pending=e.s.handleMessage(packet);
    if(scenario==='pre-response')await e.s.handleMessage(speech('Move queued.','pre',false));
    if(scenario==='stop')e.s.pauseRobotTools();
    if(scenario==='cancel')await e.s.handleMessage({toolCallCancellation:{ids:['old']}});
    if(scenario==='disconnect')await e.s.disconnect();
    if(scenario==='newer')await e.s.handleMessage(call('new','robot_status'));
    if(scenario==='boundary')await e.s.handleMessage({serverContent:{turnComplete:true}});
    d.resolve(queued);await pending;assert.deepEqual(e.audio,[],scenario);
  }
});

test('cache uses complete current status/entry/paw authorization and leaves fourth joke on exact native path',async()=>{
  const e=service(await cache());
  for(let n=1;n<=4;n++) {
    await e.s.handleMessage(call('s'+n,'robot_status'));
    if(n===1)await e.s.handleMessage(call('entry','play_trick',{name:'sit_stay_shake'}));
    assert.equal(e.audio.length,n-1);
    await e.s.handleMessage(call('p'+n,'interactive_action',{action:'shake_left'}));
    const result=e.responses.at(-1).functionResponses.response;
    if(n<4){assert.equal(e.audio.length,n);assert.equal(result.operator_acknowledgment,undefined);await e.s.handleMessage(speech('Left paw command queued.','duplicate'));}
    else {assert.equal(e.audio.length,3);assert.equal(result.operator_acknowledgment.text,'Left paw command queued. At this rate, I should charge for pawtographs.');await e.s.handleMessage(speech(result.operator_acknowledgment.text,'joke'));}
  }
  assert.deepEqual(e.audio,[pcm64,pcm64,pcm64,'joke']);
});

test('pending cached playback never delays Stop and cancelled decode cannot later start audio',async()=>{
  const e=service(await cache()),resume=deferred(),started=[];
  e.s.playAudioBase64=GeminiLiveService.prototype.playAudioBase64;
  e.s.analyser={};e.s.audioContext={state:'suspended',currentTime:0,resume:()=>resume.promise,createBuffer:()=>({duration:0.01,getChannelData:()=>new Float32Array(2)}),createBufferSource:()=>({connect(){},start(){started.push(true);},stop(){}}),close:async()=>{}};
  await e.s.handleMessage(call('m','move'));
  assert.equal(e.responses.length,1,'HTTP result not delayed by cached audio');
  await e.s.handleMessage(call('stop','stop'));
  assert.deepEqual(e.calls,['move','stop']);assert.equal(e.responses.length,2,'Stop dispatch/reply independent of pending cached audio');
  e.s.pauseRobotTools();resume.resolve();await new Promise(r=>setImmediate(r));assert.deepEqual(started,[]);
});

function prepareHold(s) {
  const track={enabled:false};s.config.enableAudio=true;s.inputAudioContext={state:'running'};
  s.mediaStream={getAudioTracks:()=>[track],getTracks:()=>[track]};return track;
}
test('new deliberate hold invalidates a pending cached decode and late old HTTP without stopping robot action',async()=>{
  for(const phase of ['decode','http']) {
    const e=service(await cache()),held=deferred(),started=[];prepareHold(e.s);
    if(phase==='http')e.s.robotBridge.callTool=()=>held.promise;
    else {
      e.s.playAudioBase64=GeminiLiveService.prototype.playAudioBase64;e.s.analyser={};
      e.s.audioContext={state:'suspended',currentTime:0,resume:()=>held.promise,createBuffer:()=>({duration:0.01,getChannelData:()=>new Float32Array(2)}),createBufferSource:()=>({connect(){},start(){started.push(true);},stop(){}})};
    }
    const pending=e.s.handleMessage(call('m','move'));if(phase==='decode')await pending;
    assert.equal(e.s.beginAudioInput(),true);held.resolve(queued);await pending;await new Promise(r=>setImmediate(r));
    assert.deepEqual(started,[],phase);assert.deepEqual(e.audio,[],phase);assert.equal(e.calls.includes('stop'),false);
  }
});
test('fresh hold cancels existing output only; repeated hold and first genuine conversation remain usable',async()=>{
  const e=service(await cache());prepareHold(e.s);
  assert.equal(e.s.beginAudioInput(),true);const generation=e.s.playbackGeneration;
  assert.equal(e.s.beginAudioInput(),true);assert.equal(e.s.playbackGeneration,generation);
  await e.s.handleMessage(speech('Hello friend!','first-genuine'));assert.deepEqual(e.audio,['first-genuine']);
  e.s.audioInputActive=false;
  await e.s.handleMessage(speech('An older thought.','old-buffer',false));e.s.beginAudioInput();
  await e.s.handleMessage({serverContent:{turnComplete:true}});assert.deepEqual(e.audio,['first-genuine']);
  await e.s.handleMessage(speech('A new thought.','new-genuine'));assert.deepEqual(e.audio,['first-genuine','new-genuine']);
});
