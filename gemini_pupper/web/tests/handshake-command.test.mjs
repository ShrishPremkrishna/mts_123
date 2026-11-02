import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const bundle=await build({stdin:{contents:"export {GeminiLiveService} from './services/geminiLive'; export {queuedHandshakePaw,operatorFeedback} from './services/operatorFeedback'; export {serverReceiptAudioText} from './services/receiptAudioCache';",resolveDir:process.cwd(),loader:'ts'},bundle:true,write:false,format:'esm',platform:'node',plugins:[{name:'fake-sdk',setup(b){b.onResolve({filter:/^@google\/genai$/},()=>({path:'sdk',namespace:'stub'}));b.onLoad({filter:/.*/,namespace:'stub'},()=>({contents:"export class GoogleGenAI{};export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'};export const Type={OBJECT:'OBJECT',STRING:'STRING'};"}));}}]});
const {GeminiLiveService,queuedHandshakePaw,operatorFeedback,serverReceiptAudioText}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
const receipt=(paw='left')=>({success:true,details:{submission:{schema:'pupper_submission_v1',state:'queued',server_instance:'a'.repeat(32),job_id:'a'.repeat(32)+':1'},handshake:{schema:'pupper_handshake_submission_v1',paw,action:'shake_'+paw}}});
const call=(id,paw)=>({toolCall:{functionCalls:[{id,name:'shake_hand',args:{paw}}]}});
const end=()=>({serverContent:{generationComplete:true,turnComplete:true}});
function rig(t,model){
 const audio=[],tts=[],responses=[],calls=[],config={modelName:model,voiceName:'Puck',ttsModelName:'gemini-3.1-flash-tts-preview',robotApiUrl:'http://handshake-tests',outputPolicy:'conversation'};
 const s=new GeminiLiveService(config,()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},{lookup:text=>text});
 s.connected=true;s.sessionPromise=Promise.resolve({sendToolResponse:x=>responses.push(x)});s.playAudioBase64=async x=>audio.push(x);s.synthesizeAndPlaySpeech=async x=>tts.push(x);
 s.robotBridge.callTool=async(name,args)=>{calls.push([name,args]);return receipt(args.paw)};s.robotBridge.immediateStop=async()=>({success:true});t.after(()=>{s.connected=false;s.nativeReceiptOutput.close();s.conversationOutput.close()});return {s,audio,tts,responses,calls};
}
for(const model of ['gemini-3.1-flash-live-preview','gemini-robotics-er-2-streaming-preview'])test('one combined handshake call gets one cue after its actual reply: '+model,async t=>{
 const e=rig(t,model);let resolve;e.s.robotBridge.callTool=()=>new Promise(r=>resolve=r);const pending=e.s.handleMessage(call('left','left'));assert.deepEqual(e.audio,[]);resolve(receipt('left'));await pending;assert.deepEqual(e.audio,['Okay.']);assert.equal(e.responses.length,1);
 await e.s.handleMessage({serverContent:{outputTranscription:{text:'Left paw command queued.'},generationComplete:true,turnComplete:true}});assert.deepEqual(e.audio,['Okay.']);assert.deepEqual(e.tts,[]);
});
test('four complete combined handshake requests retain the one fixed joke',async t=>{const e=rig(t,'gemini-robotics-er-2-streaming-preview');for(let i=0;i<4;i++){await e.s.handleMessage(call(String(i),i%2?'right':'left'));await e.s.handleMessage(end())}assert.equal(e.calls.length,4);assert.deepEqual(e.audio,['Okay.','Okay.','Okay.','Okay.']);assert.deepEqual(e.tts,['At this rate, I should charge for pawtographs.']);});
test('typed handshake provenance and requested paw must agree before queued cue',async t=>{
 for(const alter of [x=>delete x.details.submission.job_id,x=>delete x.details.submission.server_instance,x=>x.details.submission.job_id='b'.repeat(32)+':1',x=>x.details.submission.job_id='a'.repeat(32)+':0',x=>delete x.details.handshake,x=>x.details.handshake.schema='wrong',x=>x.details.handshake.action='shake_right',x=>x.details.submission.state='done',x=>x.superseded=true,x=>x.outcome_unknown=true]){const value=receipt();alter(value);assert.equal(queuedHandshakePaw(value),null);assert.equal(serverReceiptAudioText('shake_hand',{paw:'left'},value),null);assert.ok(!operatorFeedback('shake_hand',value).includes('paw command queued'))}
 assert.equal(serverReceiptAudioText('shake_hand',{paw:'right'},receipt('left')),null);
 const e=rig(t,'gemini-robotics-er-2-streaming-preview');e.s.robotBridge.callTool=async()=>receipt('right');await e.s.handleMessage(call('wrong','left'));assert.deepEqual(e.audio,[]);assert.deepEqual(e.tts,[]);
});
test('Stop while combined handshake admission is pending prevents a late Okay',async t=>{const e=rig(t,'gemini-robotics-er-2-streaming-preview');let resolve;e.s.robotBridge.callTool=async name=>name==='stop'?{success:true}:new Promise(r=>resolve=r);const pending=e.s.handleMessage(call('pending','left'));await e.s.handleMessage({toolCall:{functionCalls:[{id:'stop',name:'stop',args:{}}]}});resolve(receipt());await pending;assert.deepEqual(e.audio,['Stop requested.']);});
