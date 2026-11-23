/* One display adapter for live runner edges and richer synthetic rehearsal reports. */
const words = value => String(value || 'unknown').replaceAll('_', ' ');
const SOURCES = {executor: 'planner request', runner: 'runner receipt',
  'synthetic native receipt': 'simulated robot receipt'};

export function kickPresentation(kick, edge, currentEpoch, starting = false) {
  let row = kick?.attempt_id ? kick : edge;
  if (edge?.attempt_id && edge.epoch === currentEpoch && row?.epoch !== currentEpoch) row = edge;
  if (!row?.attempt_id) return null;
  const epoch = row.epoch || (edge?.attempt_id === row.attempt_id ? edge.epoch : null);
  const previous = starting || Boolean(epoch && currentEpoch && epoch !== currentEpoch);
  const synthetic = row.synthetic === true || (edge?.attempt_id === row.attempt_id && edge.synthetic === true);
  const outcome = row.outcome ? ' · outcome: ' + words(row.outcome) : '';
  const impulse = row.impulse == null ? '' : row.impulse ? ' · simulated ball impulse applied' : ' · ball outside simulated reach';
  const receipts = row.receipts?.length ? row.receipts.map(receipt => ({
    text: words(receipt.phase) + ' — ' + (SOURCES[receipt.source] || words(receipt.source)),
    synthetic: receipt.source === 'synthetic native receipt',
  })) : [{text: (row.state === 'engaged' ? 'Kick requested by the robot runner' :
    synthetic ? 'Simulated robot receipt' : 'Robot runner receipt') + ' · no physical contact observed', synthetic}];
  return {heading: (previous ? 'Previous match · ' : '') + 'Kick ' + row.attempt_id + ': ' + words(row.state) + outcome + impulse,
    epoch, receipts};
}
