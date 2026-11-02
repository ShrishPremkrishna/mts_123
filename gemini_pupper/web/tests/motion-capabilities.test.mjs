import test from 'node:test';
import assert from 'node:assert/strict';
import {build} from 'esbuild';
import {fileURLToPath} from 'node:url';
const compiled = await build({entryPoints: [fileURLToPath(new URL('../services/executionStatus.ts', import.meta.url))], bundle: true, write: false, format: 'esm', platform: 'node'});
const {motionCapabilitySummary, parseQueueStatus} = await import('data:text/javascript;base64,' + Buffer.from(compiled.outputFiles[0].text).toString('base64'));
const status = {queued: 0, running: false, following: false, state: 'locomotion', controller_fault: null, last_outcome: null};
const caps = {schema: 'pupper_motion_capabilities_v1', autonomous_visual_navigation: false, collision_avoidance: false, person_following: false};

test('configured following does not imply visual navigation or obstacle avoidance', () => {
  const report = parseQueueStatus({...status, capabilities: {...caps, person_following: true}});
  assert.ok(report);
  const text = motionCapabilitySummary(report, true);
  assert.match(text, /Visual navigation: unavailable/);
  assert.match(text, /Obstacle avoidance: unavailable/);
  assert.match(text, /Follower service: configured/);
});

test('old, missing, malformed and stale capability reports cannot establish availability', () => {
  for (const capabilities of [undefined, null, [], {}, {...caps, schema: 'future_v2'},
    {...caps, collision_avoidance: 'false'}, {...caps, person_following: 1},
    {...caps, autonomous_visual_navigation: null}]) {
    const report = parseQueueStatus({...status, capabilities});
    assert.ok(report, 'unusable capability metadata does not discard independent queue evidence');
    assert.equal(motionCapabilitySummary(report, true), 'Navigation and obstacle avoidance: unverified');
  }
  assert.equal(motionCapabilitySummary(null, true), 'Navigation and obstacle avoidance: unverified');
  assert.equal(motionCapabilitySummary({...status, capabilities: caps}, false), 'Navigation and obstacle avoidance: unverified');
});

test('future positive metadata remains a server report rather than physical validation', () => {
  const text = motionCapabilitySummary({...status, capabilities: {...caps, autonomous_visual_navigation: true, collision_avoidance: true}}, true);
  assert.match(text, /Visual navigation: reported available/);
  assert.match(text, /Obstacle avoidance: reported available/);
  assert.doesNotMatch(text, /safe|validated|collision.free/);
});
