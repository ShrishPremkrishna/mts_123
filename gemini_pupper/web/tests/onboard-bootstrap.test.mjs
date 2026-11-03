import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const built=await build({entryPoints:['services/onboardBootstrap.ts'],bundle:true,write:false,format:'esm',platform:'node'});
const {takeOnboardBootstrap,consumeOnboardBootstrap}=await import('data:text/javascript;base64,'+Buffer.from(built.outputFiles[0].text).toString('base64'));
function replace(t,name,value){const old=Object.getOwnPropertyDescriptor(globalThis,name);Object.defineProperty(globalThis,name,{value,configurable:true,writable:true});t.after(()=>old?Object.defineProperty(globalThis,name,old):delete globalThis[name]);}
const nonce='n'.repeat(43);
const body=()=>({schema:'pupper_onboard_bootstrap_v1',autoConnect:true,apiKey:'TEST_MEMORY_KEY',config:{voiceInputMode:'hands_free',robotApiUrl:'',cameraSource:'server',eyeTrackingSource:'server',enableAudio:true,enableVideo:true,modelName:'gemini-robotics-er-2-streaming-preview',ttsModelName:'gemini-3.1-flash-tts-preview',voiceName:'Puck',outputPolicy:'conversation',quickReceipts:true}});
for(const [name,host,hash,expected] of [['normal','localhost','#onboard='+nonce,nonce],['duplicate','localhost','#onboard='+nonce+'&onboard='+nonce,null],['short','localhost','#onboard=short',null],['nonloopback','example.test','#onboard='+nonce,null]]){
 test(`onboard fragment is removed before use: ${name}`,t=>{
  const paths=[];replace(t,'window',{location:{hostname:host,hash,pathname:'/',search:'?v=1'},history:{replaceState:(_s,_t,path)=>paths.push(path)}});
  assert.deepEqual(takeOnboardBootstrap(),{present:true,nonce:expected});assert.deepEqual(paths,['/?v=1']);
 });
}
test('ordinary page neither consumes bootstrap nor rewrites navigation',t=>{
 replace(t,'window',{location:{hash:'#ordinary'},history:{replaceState(){throw Error('must not run')}}});
 assert.deepEqual(takeOnboardBootstrap(),{present:false,nonce:null});
});
test('bootstrap accepts only schema-bound same-origin model settings and whitelists config',async t=>{
 let request;const value=body();value.config.apiKey='MUST_NOT_PERSIST';value.config.extraInstructions='MUST_NOT_IMPORT';
 replace(t,'fetch',async(url,options)=>{request={url,...options};return new Response(JSON.stringify(value),{status:200});});
 const result=await consumeOnboardBootstrap(nonce,new AbortController().signal);
 assert.equal(request.url,'/onboard/bootstrap');assert.equal(request.credentials,'same-origin');assert.equal(request.cache,'no-store');
 assert.deepEqual(JSON.parse(request.body),{nonce});assert.equal(result.apiKey,'TEST_MEMORY_KEY');
 assert.equal(result.config.apiKey,undefined);assert.equal(result.config.extraInstructions,undefined);
 assert.equal(result.config.modelName,'gemini-robotics-er-2-streaming-preview');
});
for(const [field,value] of [['robotApiUrl','http://other'],['voiceInputMode','hold_to_talk'],['modelName','unknown'],['ttsModelName','unknown'],['voiceName','unknown'],['cameraSource','browser'],['enableAudio',false]]){
 test(`bootstrap rejects mismatched ${field}`,async t=>{
  const result=body();result.config[field]=value;replace(t,'fetch',async()=>new Response(JSON.stringify(result),{status:200}));
  await assert.rejects(consumeOnboardBootstrap(nonce,new AbortController().signal),/Invalid onboard setup/);
 });
}
for(const ending of ['caller','timeout'])test(`pending bootstrap cancels on ${ending} without retry`,async t=>{
 let expire,requests=0;const external=new AbortController();
 if(ending==='timeout'){replace(t,'setTimeout',(fn,ms)=>{assert.equal(ms,10000);expire=fn;return 123;});replace(t,'clearTimeout',()=>{});}
 replace(t,'fetch',async(_url,{signal})=>{requests++;return await new Promise((_resolve,reject)=>{signal.addEventListener('abort',()=>reject(new Error('aborted')),{once:true});});});
 const pending=consumeOnboardBootstrap(nonce,external.signal);const rejected=assert.rejects(pending,/aborted/);
 if(ending==='caller')external.abort();else expire();await rejected;assert.equal(requests,1);
});
