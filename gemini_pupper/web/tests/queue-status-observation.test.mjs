import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
import {fileURLToPath} from 'node:url';
const compiled=await build({entryPoints:[fileURLToPath(new URL('../services/executionStatus.ts',import.meta.url))],bundle:true,write:false,format:'esm',platform:'node'});
const {parseQueueStatus,queueSummary}=await import('data:text/javascript;base64,'+Buffer.from(compiled.outputFiles[0].text).toString('base64'));
const idle={queued:0,running:false,following:false,state:'motors idle',state_after_queue:'motors idle',controller_fault:null,last_outcome:null};

test('observed stock locomotion does not display stale idle API plan',()=>{
 const report={...idle,observed_controller_mode:'locomotion',state_basis:'cached_queue_plan'};
 assert.deepEqual(parseQueueStatus(report),report);
 const text=queueSummary(report);
 assert.match(text,/Controller: locomotion/);
 assert.doesNotMatch(text,/cached|API plan|motors idle|Next:/);
 assert.doesNotMatch(text,/standing|physically|motors off|relaxed/);
});
test('missing or unfamiliar observation stays unverified rather than promoting cached mode',()=>{
 for(const observed of [undefined,'unknown','new_future_mode']){
  const text=queueSummary({...idle,state:'locomotion',observed_controller_mode:observed});
  assert.match(text,/Controller: unverified/);
  assert.doesNotMatch(text,/cached|API plan|Next:/);
  assert.doesNotMatch(text,/Controller: locomotion/);
 }
});
test('observed inactive inventory is controller evidence, not a physical stop claim',()=>{
 const text=queueSummary({...idle,observed_controller_mode:'none'});
 assert.match(text,/no active command controller/);
 assert.doesNotMatch(text,/stopped|relaxed|motors off/);
});
test('observed skill shows a human-readable different next controller only with pending work',()=>{
 const text=queueSummary({...idle,state:'sit_stay_shake',state_after_queue:'locomotion',observed_controller_mode:'sit_stay_shake',running:true,queued:2});
 assert.match(text,/Controller: Sit/);assert.match(text,/Next: locomotion/);assert.match(text,/action running · 2 queued/);
});
test('optional observation and projected state fields reject malformed types',()=>{
 for(const value of [{...idle,observed_controller_mode:42},{...idle,state_after_queue:{}},{...idle,state_basis:[]}] )assert.equal(parseQueueStatus(value),null);
 assert.deepEqual(parseQueueStatus(idle),idle);
});

test('no next-mode hint for idle queues, identical destination, or unknown future label',()=>{
 for (const report of [
  {...idle,observed_controller_mode:'locomotion',state_after_queue:'motors idle'},
  {...idle,observed_controller_mode:'locomotion',state_after_queue:'locomotion',running:true},
  {...idle,observed_controller_mode:'locomotion',state_after_queue:'unmapped_future_skill',queued:1},
 ]) assert.doesNotMatch(queueSummary(report),/Next:/);
 const changing=queueSummary({...idle,observed_controller_mode:'sit_stay_shake',state_after_queue:'motors idle',queued:1});
 assert.match(changing,/Next: no active command controller/);
 assert.doesNotMatch(changing,/motors idle|relaxed|motors off/);
});
