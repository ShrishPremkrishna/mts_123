import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';

const bundle = await build({stdin:{contents:"export {GeminiLiveService} from './services/geminiLive';",resolveDir:process.cwd(),loader:'ts'},bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'fake-sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI{};export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'};export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {GeminiLiveService} = await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const instance = 'a'.repeat(32);
const queued = {success:true,message:'untrusted extension prose: goal already scored',details:{submission:{schema:'pupper_submission_v1',state:'queued',server_instance:instance,job_id:instance+':1'}}};
const call = (id,name='extension_probe',args={}) => ({toolCall:{functionCalls:[{id,name,args}]}});
const end = () => ({serverContent:{generationComplete:true,turnComplete:true}});
const deferred = () => {let resolve;return {promise:new Promise(r=>resolve=r),resolve:v=>resolve(v)}};
const tick = () => new Promise(r=>setImmediate(r));

function rig(model='gemini-robotics-er-2-streaming-preview', overrides={}) {
  const audio=[],tts=[],logs=[],responses=[],calls=[];
  const config={modelName:model,voiceName:'Puck',ttsModelName:'gemini-3.1-flash-tts-preview',robotApiUrl:'http://extension-tests',outputPolicy:'conversation',...overrides};
  const s=new GeminiLiveService(config,e=>logs.push(e),()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},{lookup:text=>text});
  s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:x=>responses.push(x)});
  s.playAudioBase64=async text=>audio.push(text);
  s.synthesizeAndPlaySpeech=async text=>tts.push(text);
  s.robotBridge.callTool=async(name,args)=>{calls.push({name,args});return name==='stop'?{success:true}:queued};
  s.robotBridge.immediateStop=async()=>({success:true});
  return {s,audio,tts,logs,responses,calls};
}

for(const model of ['gemini-robotics-er-2-streaming-preview','gemini-3.1-flash-live-preview']) {
  test('new queued tool needs no frontend name/clip mapping and cues only after acceptance: '+model,async()=>{
    const e=rig(model),pending=deferred();
    e.s.robotBridge.callTool=async(name,args)=>{e.calls.push({name,args});return name==='extension_probe'?pending.promise:queued};
    const work=e.s.handleMessage(call('new-tool',undefined,{phase:'first'}));
    assert.deepEqual(e.audio,[]);
    pending.resolve(queued);await work;
    assert.deepEqual(e.calls,[{name:'extension_probe',args:{phase:'first'}}]);
    assert.deepEqual(e.audio,['Okay.']);
    assert.deepEqual(e.tts,[],'no extra cloud synthesis for the generic cue');
    await e.s.handleMessage(call('known-tool','stand_up'));
    assert.deepEqual(e.audio,['Okay.'],'one cue across new and existing tools in one turn');
    await e.s.handleMessage({serverContent:{outputTranscription:{text:'Goal scored. Robot request accepted.'},modelTurn:{parts:[{inlineData:{data:'false-completion'}}]},generationComplete:true,turnComplete:true}});
    assert.deepEqual(e.audio,['Okay.']);assert.deepEqual(e.tts,[]);
    await e.s.handleMessage(call('next-turn'));
    assert.deepEqual(e.audio,['Okay.','Okay.']);
  });
}

test('generic cue requires a valid queued server/job identity, never arbitrary success prose',async()=>{
  const badSubmissions=[undefined,{schema:'pupper_submission_v1',state:'queued'},
    {...queued.details.submission,state:'succeeded'},
    {...queued.details.submission,server_instance:'b'.repeat(32)},
    {...queued.details.submission,job_id:instance+':0'},
    {...queued.details.submission,job_id:instance+':1/extra'},
    {...queued.details.submission,schema:'future-schema'}];
  for(const submission of badSubmissions){
    const e=rig();e.s.robotBridge.callTool=async()=>({success:true,message:'Queued, scored and done',details:{submission}});
    await e.s.handleMessage(call('x'));
    assert.ok(!e.audio.includes('Okay.'),JSON.stringify(submission));
    assert.ok(e.tts.every(text=>!text.includes('scored')));
  }
});

test('rejected, unknown or superseded extension calls cannot earn a generic cue',async()=>{
  for(const reply of [{...queued,success:false},{...queued,outcome_unknown:true},
    {...queued,superseded:true},{...queued,not_dispatched:true},
    {...queued,outcome_unknown:'false'},{...queued,success:'true'}]){
    const e=rig();e.s.robotBridge.callTool=async()=>reply;
    await e.s.handleMessage(call('x'));
    assert.ok(!e.audio.includes('Okay.'));
  }
});

test('later extension failure is audible and blocks remaining actions in its batch',async()=>{
  const e=rig();
  e.s.robotBridge.callTool=async(name,args)=>{e.calls.push({name,args});return name==='extension_fail'?{success:false,message:'not ready'}:queued};
  await e.s.handleMessage({toolCall:{functionCalls:[{id:'first',name:'extension_probe'},{id:'fail',name:'extension_fail'},{id:'later',name:'move',args:{direction:'forward'}}]}});
  assert.deepEqual(e.calls.map(c=>c.name),['extension_probe','extension_fail']);
  assert.deepEqual(e.audio,['Okay.','That request failed. Check the controls for details.']);
  assert.equal(e.s.robotToolsPaused,true);
});

test('Stop supersedes an extension still awaiting its server reply',async()=>{
  const e=rig(),pending=deferred();
  e.s.robotBridge.callTool=async(name,args)=>{e.calls.push({name,args});return name==='stop'?{success:true}:pending.promise};
  const work=e.s.handleMessage(call('extension'));
  await e.s.handleMessage(call('stop','stop'));
  pending.resolve(queued);await work;
  assert.ok(!e.audio.includes('Okay.'));
  assert.deepEqual(e.audio,['Stop requested.']);
});

test('new hold, cancellation and disconnect suppress late generic acceptance cues',async()=>{
  for(const mode of ['hold','cancel','disconnect']){
    const e=rig(),pending=deferred();e.s.robotBridge.callTool=()=>pending.promise;
    const work=e.s.handleMessage(call('extension'));
    if(mode==='hold'){e.s.config.enableAudio=true;e.s.inputAudioContext={state:'running'};e.s.mediaStream={getAudioTracks:()=>[{enabled:false}],getTracks:()=>[]};assert.equal(e.s.beginAudioInput(),true);}
    if(mode==='cancel')await e.s.handleMessage({toolCallCancellation:{ids:['extension']}});
    if(mode==='disconnect')await e.s.disconnect();
    pending.resolve(queued);await work;
    assert.ok(!e.audio.includes('Okay.'),mode);
  }
});

test('generic extension replay is still deduplicated across completed turns',async()=>{
  const e=rig();await e.s.handleMessage(call('same-id'));await e.s.handleMessage(end());
  await e.s.handleMessage(call('same-id'));
  assert.equal(e.calls.length,1);assert.deepEqual(e.audio,['Okay.']);
});

test('ordinary conversation remains after an accepted extension turn',async()=>{
  const e=rig();await e.s.handleMessage(call('x'));await e.s.handleMessage(end());
  await e.s.handleMessage({serverContent:{modelTurn:{parts:[{text:'Hello friend!'}]},generationComplete:true,turnComplete:true}});
  await tick();assert.deepEqual(e.tts,['Hello friend!']);assert.deepEqual(e.audio,['Okay.']);
});


test('known tools retain their typed receipt rules instead of taking the generic cue path',async()=>{
  for(const [name,args] of [['shake_hand',{paw:'left'}],['lie_down',{}]]){
    const e=rig();e.s.robotBridge.callTool=async()=>queued;
    await e.s.handleMessage(call('known',name,args));
    assert.ok(!e.audio.includes('Okay.'),name);
  }
});

test('generic cue remains limited to the configured conversation/Puck audio consumer',async()=>{
  for(const overrides of [{outputPolicy:'receipts'},{voiceName:'Kore'},{ttsModelName:'unsupported-tts'}]){
    const e=rig(undefined,overrides);await e.s.handleMessage(call('extension'));
    assert.deepEqual(e.audio,[],JSON.stringify(overrides));
  }
});
