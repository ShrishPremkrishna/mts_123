import assert from 'node:assert/strict';
import test from 'node:test';
import {kickPresentation} from '../static/kick_view.mjs';

test('a live runner edge is visible without replay telemetry', () => {
  const view = kickPresentation(null, {attempt_id: 'native-17', epoch: 'game-1', state: 'completed', outcome: 'walker_restored'}, 'game-1');
  assert.match(view.heading, /Kick native-17: completed.*walker restored/);
  assert.equal(view.epoch, 'game-1');
  assert.deepEqual(view.receipts, [{text: 'Robot runner receipt · no physical contact observed', synthetic: false}]);
});

test('a new kickoff dates the previous attempt instead of presenting it as current', () => {
  const edge = {attempt_id: 'old', epoch: 'game-1', state: 'completed'};
  assert.match(kickPresentation(null, edge, 'game-2').heading, /^Previous match/);
  assert.match(kickPresentation(null, edge, 'game-1', true).heading, /^Previous match/);
  assert.doesNotMatch(kickPresentation(null, {...edge, epoch: 'game-2'}, 'game-2').heading, /^Previous match/);
});

test('a current live edge wins over old rich telemetry', () => {
  const view = kickPresentation({attempt_id: 'old', epoch: 'game-1', state: 'completed'},
    {attempt_id: 'new', epoch: 'game-2', state: 'engaged'}, 'game-2');
  assert.match(view.heading, /Kick new: engaged/);
  assert.match(view.receipts[0].text, /^Kick requested/);
});

test('synthetic receipt labels stay distinct and empty evidence stays empty', () => {
  assert.equal(kickPresentation(null, null, 'game-1'), null);
  const view = kickPresentation({attempt_id: 'sim', epoch: 'game-1', state: 'completed', synthetic: true,
    receipts: [{phase: 'engaged', source: 'runner'}, {phase: 'completed', source: 'synthetic native receipt'}]}, null, 'game-1');
  assert.equal(view.receipts[0].synthetic, false);
  assert.equal(view.receipts[1].synthetic, true);
  assert.match(view.receipts[1].text, /simulated robot receipt/);
});
