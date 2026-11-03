import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
const bundle=await build({entryPoints:['services/operatorFeedback.ts'],bundle:true,write:false,format:'esm',platform:'node'});
const {operatorFeedback,nativeReceiptFeedback,isOperatorReceiptNarration}=await import('data:text/javascript;base64,'+Buffer.from(bundle.outputFiles[0].text).toString('base64'));
test('unsupported playback quantity does not claim all routines allow only one',()=>{
 const refused={success:false,message:'untrusted extra prose',details:{refusal:{schema:'pupper_trick_quantity_refusal_v1',code:'playback_count_unsupported'}}};
 assert.equal(operatorFeedback('play_trick',refused),'Requested routine playback count is unsupported.');
 assert.equal(isOperatorReceiptNarration(operatorFeedback('play_trick',refused)),true);
 assert.equal(isOperatorReceiptNarration('Only one routine playback is supported per request.'),true);
 assert.equal(operatorFeedback('play_trick',{...refused,outcome_unknown:true}),'I could not confirm that request.');
});
test('two-swim acceptance remains one queued acknowledgment and not completion',()=>{
 const accepted={success:true,message:'controller already swam twice',details:{submission:{schema:'pupper_submission_v1',state:'queued'}}};
 assert.equal(operatorFeedback('play_trick',accepted),'Trick queued.');
});
