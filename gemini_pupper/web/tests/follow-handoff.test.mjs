import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const built=await build({stdin:{contents:"export {GeminiLiveService} from './services/geminiLive'; export {default as RobotBridge} from './services/robotBridge'; export * from './services/followHandoff';",resolveDir:process.cwd(),loader:'ts'},bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'no-provider',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'test'}));b.onLoad({filter:/.*/,namespace:'test'},()=>({contents:"export class GoogleGenAI{};export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'};export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {GeminiLiveService,RobotBridge,FOLLOW_HANDOFF_CUE,followHandoffTools,confirmedFollowHandoff}=await import('data:text/javascript;base64,'+Buffer.from(built.outputFiles[0].text).toString('base64'));
const id='a'.repeat(32),epoch=id+':0';
const queued={success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued',server_instance:id,job_id:id+':1'}}};
const confirmed=(changed=true)=>({success:true,details:{following:{schema:'pupper_follow_state_v1',enabled:false,changed,server_instance:id,command_epoch:epoch}}});
const call=(id,name='move',args={})=>({toolCall:{functionCalls:[{id,name,args}]}});
const tick=()=>new Promise(resolve=>setImmediate(resolve));
const deferred=()=>{let resolve;return {promise:new Promise(r=>resolve=r),resolve:v=>resolve(v)}};
function rig(){
 const calls=[],sources=[],audio=[],responses=[];let preparations=0,serial=0;
 const config={modelName:'gemini-robotics-er-2-streaming-preview',ttsModelName:'gemini-3.1-flash-tts-preview',voiceName:'Puck',robotApiUrl:'http://handoff-test',outputPolicy:'conversation'};
 const cache={lookup:text=>text===FOLLOW_HANDOFF_CUE?Buffer.alloc(8).toString('base64'):text};
 const s=new GeminiLiveService(config,()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},undefined,undefined,cache);
 s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:r=>responses.push(r),close(){}});
 s.followHandoffToolNames=followHandoffTools({schema:'pupper_follow_handoff_v1',prepare_if_active:true,tools:['move','new_pose']},[{name:'move'},{name:'new_pose'}]);
 const ctx={state:'running',currentTime:0,close:async()=>{},createBuffer:()=>({duration:1.56,getChannelData:()=>new Float32Array(4)}),createBufferSource(){const source={connect(){},disconnect(){},start(at){this.at=at;},stop(){this.stopped=true;},buffer:null};sources.push(source);return source;}};
 s.audioContext=ctx;s.analyser={};s.playAudioBase64=async text=>audio.push(text);
 s.robotBridge.prepareFollowHandoff=async()=>confirmed(++preparations===1);
 s.robotBridge.statusGeneration=()=>serial;
 s.robotBridge.callTool=async(name,args,pinnedEpoch)=>{calls.push({name,pinnedEpoch});return name==='stop'?{success:true}:queued;};
 s.robotBridge.immediateStop=async()=>({success:true});
 return {s,calls,sources,audio,responses,ctx,get preparations(){return preparations},bumpStop(){serial++;},end(){const source=sources.at(-1);ctx.currentTime=source.at+source.buffer.duration;source.onended?.();}};
}
test('handoff waits for actual source completion, pins epoch and replaces Okay once',async()=>{
 const e=rig(),pending=e.s.handleMessage(call('held'));await tick();
 assert.deepEqual(e.calls,[],'motion must not dispatch before the handoff audio ends');assert.equal(e.preparations,1);assert.equal(e.sources.length,1);
 assert.deepEqual(e.audio,[]);e.end();await pending;
 assert.deepEqual(e.calls,[{name:'move',pinnedEpoch:epoch}]);assert.deepEqual(e.audio,[]);
});
test('parallel ordinary packet waits behind handoff and does not replay its cue',async()=>{
 const e=rig(),first=e.s.handleMessage(call('a')),second=e.s.handleMessage(call('b','new_pose'));await tick();
 assert.equal(e.preparations,1);assert.deepEqual(e.calls,[]);e.end();await Promise.all([first,second]);
 assert.deepEqual(e.calls.map(x=>x.name),['move','new_pose']);assert.equal(e.sources.length,1);assert.deepEqual(e.audio,[]);
});
test('already-off response leaves ordinary acceptance cue and motion behavior unchanged',async()=>{
 const e=rig();e.s.robotBridge.prepareFollowHandoff=async()=>confirmed(false);
 await e.s.handleMessage(call('a'));assert.equal(e.sources.length,0);assert.deepEqual(e.audio,['Okay.']);assert.equal(e.calls.length,1);
});
for(const mode of ['stop','interruption','disconnect','cancel','bridge-stop','early-ended','suspended','decode-failure','missing-cue','timeout'])test('held motion cancelled on '+mode,async t=>{
 const e=rig();if(mode==='suspended')e.ctx.state='suspended';if(mode==='decode-failure')e.ctx.createBuffer=()=>{throw Error('bad audio')};if(mode==='missing-cue')e.s.receiptAudioCache={lookup:()=>null};
 if(mode==='timeout')t.mock.timers.enable({apis:['setTimeout']});
 const pending=e.s.handleMessage(call('held'));await tick();
 if(mode==='stop')await e.s.handleMessage(call('stop','stop'));
 if(mode==='interruption')await e.s.handleMessage({serverContent:{interrupted:true}});
 if(mode==='disconnect')await e.s.disconnect();
 if(mode==='cancel')await e.s.handleMessage({toolCallCancellation:{ids:['held']}});
 if(mode==='bridge-stop'){e.bumpStop();e.end();}
 if(mode==='early-ended')e.sources[0].onended();
 if(mode==='timeout')t.mock.timers.tick(10000);
 await pending;assert.equal(e.calls.some(x=>x.name==='move'),false);assert.equal(e.audio.includes('Okay.'),false);
 if(mode==='stop')assert.deepEqual(e.calls.map(x=>x.name),['stop']);
});
test('Stop during preparation prevents a late confirmed reply from playing or dispatching',async()=>{
 const e=rig(),reply=deferred();e.s.robotBridge.prepareFollowHandoff=()=>reply.promise;
 const pending=e.s.handleMessage(call('held'));await e.s.handleMessage(call('stop','stop'));reply.resolve(confirmed());await pending;
 assert.equal(e.sources.length,0);assert.deepEqual(e.calls.map(x=>x.name),['stop']);
});
test('failure or malformed following proof never dispatches held motion',async()=>{
 for(const response of [{success:false}, {...confirmed(),superseded:true}, {...confirmed(),outcome_unknown:true}, {success:true,details:{following:{...confirmed().details.following,enabled:true}}}, {success:true,details:{following:{...confirmed().details.following,command_epoch:'other:0'}}}]){
  const e=rig();e.s.robotBridge.prepareFollowHandoff=async()=>response;await e.s.handleMessage(call('held'));
  assert.equal(e.calls.length,0);assert.equal(e.sources.length,0);
 }
});
test('catalog opt-in supports a new tool without exposing transport metadata to model parameters',()=>{
 assert.deepEqual([...followHandoffTools({schema:'pupper_follow_handoff_v1',prepare_if_active:true,tools:['future']},[{name:'future'}])],['future']);
 assert.equal(followHandoffTools(undefined,[]).size,0);
 assert.throws(()=>followHandoffTools({schema:'wrong',tools:['move']},[{name:'move'}]));
 assert.throws(()=>followHandoffTools({schema:'pupper_follow_handoff_v1',prepare_if_active:true,tools:['invented']},[]));
 assert.equal(confirmedFollowHandoff(confirmed()).changed,true);
});
test('pinned epoch cannot be refreshed after another client stopped or restarted server',async t=>{
 const requests=[];const bridge=new RobotBridge('http://pinned-handoff-test');
 t.mock.method(globalThis,'fetch',async(url,options)=>{requests.push({url,options});return {ok:false,status:409,json:async()=>({success:false,command_epoch_required:true,command_epoch:id+':1'})};});
 const result=await bridge.callTool('move',{},epoch);
 assert.equal(requests.length,1);assert.match(requests[0].url,/tools\/move$/);
 assert.equal(requests[0].options.headers['X-Pupper-Command-Epoch'],epoch);assert.equal(result.not_dispatched,true);
});


test('explicit follow(false) in a model batch also finishes the cue before later motion',async()=>{
 const e=rig();e.s.robotBridge.callTool=async(name,args,pinnedEpoch)=>{e.calls.push({name,pinnedEpoch});return name==='follow'?confirmed():queued;};
 e.s.robotBridge.prepareFollowHandoff=async()=>confirmed(false);
 const pending=e.s.handleMessage({toolCall:{functionCalls:[{id:'off',name:'follow',args:{enabled:false}},{id:'motion',name:'move',args:{}}]}});
 await tick();assert.deepEqual(e.calls.map(x=>x.name),['follow']);assert.equal(e.sources.length,1);
 e.end();await pending;assert.deepEqual(e.calls.map(x=>x.name),['follow','move']);assert.deepEqual(e.audio,[]);
});


test('interruption while preparation is pending also cancels the future handoff audio and held motion',async()=>{
 const e=rig(),reply=deferred();e.s.robotBridge.prepareFollowHandoff=()=>reply.promise;
 const pending=e.s.handleMessage(call('held'));
 await e.s.handleMessage({serverContent:{interrupted:true}});
 reply.resolve(confirmed());await pending;
 assert.equal(e.sources.length,0);assert.equal(e.calls.length,0);
});
