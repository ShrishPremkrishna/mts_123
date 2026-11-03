import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { build } from 'esbuild';

const root = fileURLToPath(new URL('..', import.meta.url));
const bundled = await build({ entryPoints: [`${root}services/timingCapture.ts`], bundle: true, write: false, format: 'esm', platform: 'node' });
const { TimingCapture } = await import(`data:text/javascript;base64,${Buffer.from(bundled.outputFiles[0].text).toString('base64')}`);

const SECRET = 'SECRET-sk-https://evil.example/?key=AIza';
const clock = () => { let t = 1000; const now = () => t; now.set = value => { t = value; }; return now; };
const parse = capture => JSON.parse(capture.snapshot());
const records = capture => parse(capture).connections.flatMap(c => c.records);
const timingLog = summary => ({ id: SECRET, timestamp: 1, type: 'info', message: `Timing ${JSON.stringify(summary)}` });
const fullSummary = {
  turn: 3, basis: SECRET, reason: 'complete', latest_audio_release_sent_at_ms: 1200,
  latest_audio_release_to_first_callback_ms: 40, input_transcript_first_ms: 5, input_transcript_end_ms: null,
  first_model_output_ms: 300, first_native_chunk_ms: 310, tool_entry_ms: 120, tool_reply_ms: 450, receipt_ms: 460,
  tool_count: 1, native_playback_expected: true, generation_complete_ms: 500, turn_complete_ms: 520,
  native_playback_scheduled_ms: 530, observed_input_end_to_model_ms: null, native_chunk_to_turn_complete_ms: 210,
  generation_complete_to_playback_ms: 30, turn_complete_to_playback_ms: 10, note: SECRET, [SECRET]: SECRET, injected: SECRET,
};

test('injected secrets in phases, metadata, IDs, tool names, logs and dispatch never appear', () => {
  const now = clock();
  const capture = new TimingCapture({ now });
  const scope = capture.beginConnection();
  scope.trace({ phase: 'tool_call_start', atMs: 1010, metadata: { request_id: SECRET, tool: SECRET, message_sequence: 2, [SECRET]: SECRET, status: SECRET } });
  scope.trace({ phase: SECRET, atMs: 1011, metadata: {} });
  scope.trace({ phase: 'audio_capture', atMs: 1012, metadata: { audio_sequence: 1 } });
  scope.log({ id: SECRET, timestamp: 1, type: 'info', message: SECRET });
  scope.log({ id: SECRET, timestamp: 1, type: 'user', message: `Timing ${JSON.stringify(fullSummary)}` });
  scope.log(timingLog(fullSummary));
  now.set(1500);
  scope.dispatch({ id: SECRET, name: SECRET, status: SECRET, message: SECRET, timestamp: 1, serverUrl: SECRET, job: { job_id: SECRET } });
  const text = capture.snapshot();
  for (const part of ['SECRET', 'evil', 'AIza', 'sk-']) assert.ok(!text.includes(part), part);
  const [phase, summary, dispatch] = records(capture);
  assert.deepEqual(phase.meta, { message_sequence: 2, status: null, request: 1, stop_tool: false });
  assert.equal(summary.kind, 'turn_summary');
  assert.equal(dispatch.request, 1);
  assert.equal(dispatch.status, null);
  assert.equal(dispatch.has_job, true);
  assert.equal(parse(capture).dropped.unknown_phase, 1);
  assert.equal(parse(capture).dropped.high_rate_phase, 1);
});

test('whitelisted summary metrics pass; null stays null; summary and phase clocks are distinct', () => {
  const now = clock();
  const capture = new TimingCapture({ now });
  const scope = capture.beginConnection();
  scope.trace({ phase: 'audio_hold_end_sent', atMs: 1200, metadata: {} });
  now.set(1800);
  scope.log(timingLog(fullSummary));
  const [phase, summary] = records(capture);
  assert.equal(phase.at_ms, 200);
  assert.equal(summary.received_at_ms, 800);
  assert.equal(summary.turn, 3);
  assert.equal(summary.reason, 'complete');
  assert.equal(summary.metrics.first_model_output_ms, 300);
  assert.equal(summary.metrics.input_transcript_end_ms, null);
  assert.equal(summary.metrics.observed_input_end_to_model_ms, null);
  assert.equal(summary.metrics.native_playback_expected, true);
  assert.equal(summary.metrics.latest_audio_release_sent_ms, 200);
  assert.deepEqual(Object.keys(summary).sort(), ['kind', 'metrics', 'reason', 'received_at_ms', 'turn']);
  assert.ok(!('basis' in summary.metrics) && !('note' in summary.metrics) && !('injected' in summary.metrics));
});

test('malformed, oversized and invalid Timing logs are rejected or nulled', () => {
  const capture = new TimingCapture({ now: clock() });
  const scope = capture.beginConnection();
  scope.log({ type: 'info', message: 'Timing {not json' });
  scope.log({ type: 'info', message: 'Timing [1,2]' });
  scope.log({ type: 'info', message: `Timing ${JSON.stringify({ turn: 1, pad: 'x'.repeat(5000) })}` });
  scope.log(timingLog({ turn: 'one' }));
  scope.log(timingLog({ turn: -1 }));
  assert.equal(capture.count, 0);
  assert.equal(parse(capture).dropped.malformed_timing_log, 5);
  scope.log({ type: 'info', message: `Timing {"turn":2,"reason":"evil","receipt_ms":-5,"tool_reply_ms":1e999,"first_model_output_ms":"12","tool_count":1e12}` });
  const [summary] = records(capture);
  assert.equal(summary.reason, null);
  assert.equal(summary.metrics.receipt_ms, null);
  assert.equal(summary.metrics.tool_reply_ms, null);
  assert.equal(summary.metrics.first_model_output_ms, null);
  assert.equal(summary.metrics.tool_count, null);
});

test('NaN, Infinity and negative phase timing is rejected; invalid metadata is null', () => {
  const capture = new TimingCapture({ now: clock() });
  const scope = capture.beginConnection();
  for (const atMs of [NaN, Infinity, -Infinity, 999, '1500', null]) scope.trace({ phase: 'tts_handle_start', atMs, metadata: {} });
  assert.equal(capture.count, 0);
  assert.equal(parse(capture).dropped.invalid_event, 6);
  scope.trace({ phase: 'follow_handoff_audio_started', atMs: 1001, metadata: { duration_ms: NaN, completed: 'yes' } });
  scope.trace({ phase: 'tts_request_failed', atMs: 1002, metadata: { http_status: -1 } });
  const [a, b] = records(capture);
  assert.deepEqual(a.meta, { duration_ms: null, completed: null });
  assert.deepEqual(b.meta, { http_status: null });
});

test('server_message_callback is coarsened to the first per connection', () => {
  const capture = new TimingCapture({ now: clock() });
  const scope = capture.beginConnection();
  for (let i = 0; i < 5; i++) scope.trace({ phase: 'server_message_callback', atMs: 1001 + i, metadata: { message_sequence: i + 1 } });
  assert.equal(capture.count, 1);
  assert.equal(parse(capture).dropped.repeated_server_callback, 4);
});

test('record, connection, byte and request ID bounds with explicit dropped counts', () => {
  const capture = new TimingCapture({ now: clock(), maxRecords: 10, maxRequests: 3 });
  const scope = capture.beginConnection();
  for (let i = 0; i < 15; i++) scope.trace({ phase: 'tool_call_start', atMs: 1001, metadata: { request_id: `id-${i}` } });
  let body = parse(capture);
  assert.equal(capture.count, 10);
  assert.equal(body.dropped.records_over_cap, 5);
  assert.equal(body.dropped.request_ids_over_cap, 12);
  assert.equal(body.truncated, true);
  assert.deepEqual(body.connections[0].records.map(r => r.meta.request), [1, 2, 3, null, null, null, null, null, null, null]);
  scope.trace({ phase: 'tool_call_start', atMs: 1001, metadata: { request_id: 'x'.repeat(129) } });

  const many = new TimingCapture({ now: clock() });
  for (let i = 0; i < 11; i++) many.beginConnection().trace({ phase: 'tts_handle_start', atMs: 1001, metadata: {} });
  body = parse(many);
  assert.deepEqual(body.connections.map(c => c.connection), [4, 5, 6, 7, 8, 9, 10, 11]);
  assert.equal(body.dropped.connections_evicted, 3);
  assert.equal(body.dropped.records_evicted, 3);
  assert.equal(many.count, 8);

  const small = new TimingCapture({ now: clock(), maxBytes: 6000 });
  const tiny = small.beginConnection();
  for (let i = 0; i < 200; i++) tiny.trace({ phase: 'tool_call_start', atMs: 1001, metadata: { message_sequence: i } });
  assert.ok(small.snapshot().length <= 6000);
  assert.ok(parse(small).dropped.records_over_bytes > 0);

  const defaults = new TimingCapture({ now: clock() });
  const scoped = defaults.beginConnection();
  for (let i = 0; i < 600; i++) scoped.trace({ phase: 'tool_call_start', atMs: 1001, metadata: { request_id: `r${i}`, message_sequence: i } });
  assert.equal(defaults.count, 512);
  assert.ok(defaults.snapshot().length <= 256 * 1024);
});

test('reconnect retires the old scope; late and closed callbacks are ignored', () => {
  const capture = new TimingCapture({ now: clock() });
  const first = capture.beginConnection();
  first.trace({ phase: 'tts_handle_start', atMs: 1001, metadata: {} });
  const second = capture.beginConnection();
  first.trace({ phase: 'tts_handle_return', atMs: 1002, metadata: {} });
  first.log(timingLog(fullSummary));
  first.dispatch({ id: 'a', name: 'stop', status: 'sending' });
  second.trace({ phase: 'tts_handle_start', atMs: 1003, metadata: {} });
  second.close();
  second.trace({ phase: 'tts_handle_return', atMs: 1004, metadata: {} });
  const body = parse(capture);
  assert.deepEqual(body.connections.map(c => [c.connection, c.closed, c.records.length]), [[1, true, 1], [2, true, 1]]);
  assert.equal(body.dropped.late_callback, 4);
  first.close(); // An old close cannot affect a newer scope.
  const third = capture.beginConnection();
  first.close();
  third.trace({ phase: 'tts_handle_start', atMs: 1005, metadata: {} });
  assert.equal(capture.count, 3);
});

test('clear erases data, mappings and counters; the active scope records a marked fresh segment', () => {
  const now = clock();
  const capture = new TimingCapture({ now });
  const scope = capture.beginConnection();
  scope.trace({ phase: 'tool_call_start', atMs: 1001, metadata: { request_id: 'old-id' } });
  scope.trace({ phase: 'audio_send_start', atMs: 1002, metadata: {} });
  now.set(5000);
  capture.clear();
  assert.equal(capture.count, 0);
  let body = parse(capture);
  assert.ok(Object.values(body.dropped).every(value => value === 0));
  assert.deepEqual(body.connections.map(c => [c.connection, c.segment_start, c.records.length]), [[1, 'after_clear', 0]]);
  scope.trace({ phase: 'tool_call_start', atMs: 5010, metadata: { request_id: 'new-id' } });
  scope.trace({ phase: 'tool_call_start', atMs: 5020, metadata: { request_id: 'old-id' } });
  body = parse(capture);
  assert.deepEqual(body.connections[0].records.map(r => [r.at_ms, r.meta.request]), [[10, 1], [20, 2]]);
  scope.close();
  capture.clear();
  scope.trace({ phase: 'tool_call_start', atMs: 5030, metadata: {} });
  assert.equal(parse(capture).connections.length, 0);
  assert.equal(capture.count, 0);
});

test('snapshot is detached from later mutation', () => {
  const capture = new TimingCapture({ now: clock() });
  const scope = capture.beginConnection();
  const metadata = { message_sequence: 1 };
  scope.trace({ phase: 'tts_handle_start', atMs: 1001, metadata });
  const before = capture.snapshot();
  metadata.message_sequence = 99;
  JSON.parse(before).connections[0].records[0].meta.message_sequence = 7;
  assert.equal(records(capture)[0].meta.message_sequence, 1);
});

test('App and DebugPanel wiring', () => {
  const app = readFileSync(`${root}App.tsx`, 'utf8');
  const panel = readFileSync(`${root}components/DebugPanel.tsx`, 'utf8');
  assert.match(app, /if \(!timingRef\.current\) timingRef\.current = new TimingCapture\(\);/);
  assert.match(app, /const timing = timingCapture\.beginConnection\(\);\n\s+try \{/);
  assert.match(app, /entry => \{ if \(geminiRef\.current === service\) \{ addLog\(entry\); timing\.log\(entry\); \} \}/);
  assert.match(app, /event => \{ if \(geminiRef\.current === service\) timing\.trace\(event\); \},\n\s+active =>/);
  assert.match(app, /if \(geminiRef\.current !== service\) return;\n\s+timing\.dispatch\(entry\);/);
  assert.match(app, /await service\.disconnect\(\);\n.*\n\s+timingScopes\.current!\.get\(service\)\?\.close\(\);/);
  assert.ok(!/localStorage[^\n]*timing/i.test(app));
  assert.match(panel, /link\.download = 'pupper-browser-timing\.json';/);
  assert.match(panel, /URL\.revokeObjectURL\(created\)/);
  assert.match(panel, /disabled=\{!timingHasData\}[^\n]*Download timing JSON/);
  assert.match(panel, /numeric timing records; excludes content\. Not physical or acoustic latency\./);
});


test('limits cannot disable bounds or exceed hard production caps', () => {
  for (const value of [0, -1, Infinity, NaN, 1e20]) {
    const capture = new TimingCapture({ now: clock(), maxConnections: value, maxRecords: value, maxBytes: value, maxRequests: value });
    for (let i = 0; i < 9; i++) capture.beginConnection().trace({ phase: 'tool_call_start', atMs: 1001, metadata: { request_id: 'id' } });
    const body = parse(capture);
    assert.ok(body.limits.max_connections >= 1 && body.limits.max_connections <= 8);
    assert.ok(body.limits.max_records >= 1 && body.limits.max_records <= 512);
    assert.ok(body.limits.max_requests_per_connection >= 1 && body.limits.max_requests_per_connection <= 128);
    assert.ok(body.limits.max_bytes >= 4096 && body.limits.max_bytes <= 256 * 1024);
    assert.ok(Buffer.byteLength(capture.snapshot(), 'utf8') <= body.limits.max_bytes);
  }
});

test('clear remains available for metadata-only captures and erases retained callback segments', () => {
  const capture = new TimingCapture({ now: clock(), maxConnections: 1 });
  assert.equal(capture.hasData, false);
  const scope = capture.beginConnection();
  scope.trace({ phase: 'audio_capture', atMs: 1001, metadata: {} });
  assert.equal(capture.count, 0);
  assert.equal(capture.hasData, true);
  scope.close();
  capture.clear();
  assert.equal(capture.hasData, false);
  const first = capture.beginConnection();
  first.trace({ phase: 'tts_handle_start', atMs: 1001, metadata: {} });
  const retainedSegment = capture.segments[0]; // JS inspection ensures retired closures cannot retain erased records.
  capture.beginConnection();
  assert.equal(retainedSegment.records.length, 0);
  const activeSegment = capture.segments[0];
  capture.clear();
  assert.equal(activeSegment.records.length, 0);
  assert.equal(activeSegment.requests.size, 0);
});


test('actual Debug controls export only the sanitized snapshot and clear without robot calls', async t => {
  const panelBundle = await build({ entryPoints: [`${root}components/DebugPanel.tsx`], bundle: true, write: false,
    format: 'esm', platform: 'node', jsx: 'transform', tsconfigRaw: { compilerOptions: { jsx: 'react' } },
    plugins: [{ name: 'inert-timing-hooks', setup(b) {
      b.onResolve({ filter: /^react$/ }, () => ({ path: 'react', namespace: 'stub' }));
      b.onLoad({ filter: /.*/, namespace: 'stub' }, () => ({ contents: `
        export const useRef=value=>({current:value}), useEffect=()=>{},
        useState=value=>[typeof value==='function'?value():value,()=>{}];
        export default {createElement:(type,props,...children)=>({type,props:{...props,children}})};` }));
    }}] });
  const { default: Panel } = await import('data:text/javascript;base64,' + Buffer.from(panelBundle.outputFiles[0].text).toString('base64'));
  const restore = [];
  const setGlobal = (name, value) => {
    const old = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, writable: true, value });
    restore.push(() => old ? Object.defineProperty(globalThis, name, old) : delete globalThis[name]);
  };
  t.after(() => restore.reverse().forEach(fn => fn()));
  const capture = new TimingCapture({ now: clock() });
  const scope = capture.beginConnection();
  scope.trace({ phase: 'tool_call_resolved', atMs: 1001, metadata: { request_id: SECRET, status: 'accepted' } });
  scope.close();
  let blob, clicks = 0, robotCalls = 0; const revoked = [], deferred = [];
  const link = { click() { clicks++; } };
  setGlobal('URL', { createObjectURL(value) { blob = value; return 'blob:local-snapshot'; }, revokeObjectURL(value) { revoked.push(value); } });
  setGlobal('document', { createElement(tag) { assert.equal(tag, 'a'); return link; } });
  setGlobal('setTimeout', fn => { deferred.push(fn); return 1; });
  const props = () => ({ logs: [], videoElement: null, robotBridge: {}, systemStatus: null, onStatusUpdate() {},
    cameraSource: 'disabled', onStop: async () => { robotCalls++; }, stopPending: false, isStopPending: () => false,
    timingRecordCount: capture.count, timingHasData: capture.hasData, getTimingSnapshot: () => capture.snapshot(), onClearTiming: () => capture.clear() });
  const nodes = tree => !tree || typeof tree !== 'object' ? [] : [tree, ...(tree.props?.children?.flat(Infinity) ?? []).flatMap(nodes)];
  const button = (tree, label) => nodes(tree).find(n => n.type === 'button' && n.props['aria-label'] === label);
  let tree = Panel(props());
  assert.equal(button(tree, 'Download timing JSON').props.disabled, false);
  button(tree, 'Download timing JSON').props.onClick();
  assert.equal(clicks, 1);
  assert.equal(link.download, 'pupper-browser-timing.json');
  assert.equal(blob.type, 'application/json');
  const downloaded = await blob.text();
  assert.equal(downloaded, capture.snapshot());
  assert.ok(!downloaded.includes(SECRET));
  assert.deepEqual(revoked, []);
  deferred.splice(0).forEach(fn => fn());
  assert.deepEqual(revoked, ['blob:local-snapshot']);
  button(tree, 'Clear timing').props.onClick();
  tree = Panel(props());
  assert.equal(capture.count, 0);
  assert.equal(button(tree, 'Download timing JSON').props.disabled, true);
  assert.equal(button(tree, 'Clear timing').props.disabled, true);
  assert.equal(robotCalls, 0);
});
