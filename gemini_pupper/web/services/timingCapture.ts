import type { LiveTimingEvent } from './geminiLive';
import type { ToolDispatch } from './executionStatus';
import type { LogEntry } from '../types';

/** Bounded, in-memory, numeric-only browser timing capture for manual export.
 * Raw events, IDs, tool names, messages and logs never enter a record: every
 * exported value is a finite bounded number, null, boolean or fixed constant. */

type Value = number | boolean | string | null;
type TimingRecord = Record<string, Value | Record<string, Value>>;

export interface TimingScope {
  trace: (event: LiveTimingEvent) => void;
  log: (entry: LogEntry) => void;
  dispatch: (entry: ToolDispatch) => void;
  close: () => void;
}

export interface TimingCaptureOptions {
  now?: () => number;
  maxConnections?: number;
  maxRecords?: number;
  maxBytes?: number;
  maxRequests?: number;
}

const FORMAT = 'pupper-browser-timing-v1';
const MAX_MS = 86_400_000;
const MAX_ID_LENGTH = 128;
const MAX_TIMING_LOG = 4096;
const HEADER_RESERVE = 4096;
const SEGMENT_OVERHEAD = 128;

const CLOCK = {
  phase_at_ms: 'browser performance.now() ms relative to this segment origin',
  received_at_ms: 'browser performance.now() ms at local receipt, relative to this segment origin',
  segment_connect: 'origin is the explicit Connect that created the connection',
  segment_after_clear: 'origin is the Clear action; earlier data was erased and the original connection start is not recorded',
  turn_metrics: 'ms from the first observed provider callback of that turn, not from speech end',
  latest_audio_release_sent_ms: 'browser performance.now() ms relative to segment origin; null if before the segment',
};

const CAVEATS = [
  'Numeric browser timing only; content, transcripts, tool names, arguments, IDs, URLs and logs are excluded.',
  'Not physical motion, acoustic or server-side latency; no speech onset/end or server start is inferred.',
  'Audio release is an SDK send boundary and may not belong to the turn it is reported with.',
  'Native playback is the scheduled start, not measured sound.',
  'tool_call_start may precede handoff preparation and speech; it is not necessarily an HTTP send.',
  'tool_reply_ms marks local tool handling completion, including exceptions; it does not prove server acceptance.',
  'Dispatch records use local receipt time on the phase clock; the dispatch wall-clock timestamp is omitted.',
  'Request numbers are local per-connection ordinals; null means missing, invalid or over the cap.',
  'Only the first server message callback per segment is kept; turn summaries carry per-turn detail.',
];

const PHASES = new Set([
  'audio_input_setup', 'audio_hold_start', 'audio_hands_free_start', 'audio_hold_end_sent', 'audio_send_failed',
  'server_message_callback', 'duplicate_tool_call_ignored', 'tool_call_start', 'tool_call_failed', 'tool_call_resolved',
  'function_response_send_start', 'function_response_send_return', 'function_response_send_failed',
  'function_response_session_unavailable', 'cached_receipt_admitted', 'native_receipt_admitted',
  'output_audio_handle_start', 'output_audio_handle_return', 'tts_handle_start', 'tts_handle_return', 'tts_request_failed',
  'follow_handoff_prepare_start', 'follow_handoff_audio_started', 'follow_handoff_audio_finished',
  'follow_handoff_motion_dispatch', 'automatic_stop_call_start', 'automatic_stop_call_resolved', 'session_go_away',
]);
const HIGH_RATE = new Set(['audio_capture', 'audio_send_start', 'audio_send_return']);
const STATUSES = new Set(['sending', 'accepted', 'rejected', 'unknown']);
const REASONS = new Set(['complete', 'playback-update', 'disconnected-before-boundary']);

const ms = (value: unknown) => typeof value === 'number' && Number.isFinite(value) && value >= 0 && value <= MAX_MS ? Math.round(value) : null;
const count = (max: number) => (value: unknown) => typeof value === 'number' && Number.isSafeInteger(value) && value >= 0 && value <= max ? value : null;
const bool = (value: unknown) => typeof value === 'boolean' ? value : null;
const status = (value: unknown) => typeof value === 'string' && STATUSES.has(value) ? value : null;
const own = (value: object, key: string) => Object.prototype.hasOwnProperty.call(value, key);

const PHASE_FIELDS: Record<string, (value: unknown) => Value> = {
  sample_rate: count(1_000_000), buffer_frames: count(1_000_000), encoded_rate: count(1_000_000),
  duration_ms: ms, http_status: count(999), batch_size: count(1000), bytes: count(100_000_000),
  audio_parts: count(10_000), message_sequence: count(1_000_000_000), tool_count: count(1000),
  completed: bool, sent_actions: bool, interrupted: bool, turn_complete: bool, generation_complete: bool,
  disposed: bool, input_transcription_finished: bool, status,
};

// Matches GeminiLiveService.finishTurnTiming; basis/note are replaced by CLOCK/CAVEATS.
const SUMMARY_MS = [
  'latest_audio_release_to_first_callback_ms', 'input_transcript_first_ms', 'input_transcript_end_ms',
  'first_model_output_ms', 'first_native_chunk_ms', 'tool_entry_ms', 'tool_reply_ms', 'receipt_ms',
  'generation_complete_ms', 'turn_complete_ms', 'native_playback_scheduled_ms', 'observed_input_end_to_model_ms',
  'native_chunk_to_turn_complete_ms', 'generation_complete_to_playback_ms', 'turn_complete_to_playback_ms',
];

interface Segment {
  connection: number;
  start: 'connect' | 'after_clear';
  origin: number;
  closed: boolean;
  serverCallbackSeen: boolean;
  bytes: number;
  records: TimingRecord[];
  requests: Map<string, number>;
}
interface Session { closed: boolean; segment: Segment }

const emptyDropped = () => ({
  records_over_cap: 0, records_over_bytes: 0, connections_evicted: 0, records_evicted: 0,
  unknown_phase: 0, high_rate_phase: 0, repeated_server_callback: 0, invalid_event: 0,
  malformed_timing_log: 0, request_ids_over_cap: 0, late_callback: 0,
});

export class TimingCapture {
  private readonly now: () => number;
  private readonly limits: { connections: number; records: number; bytes: number; requests: number };
  private segments: Segment[] = [];
  private active: Session | null = null;
  private connections = 0;
  private records = 0;
  private bytes = 0;
  private dropped = emptyDropped();

  constructor(options: TimingCaptureOptions = {}) {
    this.now = options.now ?? (() => performance.now());
    const bounded = (value: number | undefined, max: number, min = 1) =>
      typeof value === 'number' && Number.isFinite(value) ? Math.max(min, Math.min(max, Math.floor(value))) : max;
    this.limits = { connections: bounded(options.maxConnections, 8), records: bounded(options.maxRecords, 512),
      bytes: bounded(options.maxBytes, 256 * 1024, HEADER_RESERVE), requests: bounded(options.maxRequests, 128) };
  }

  get count() { return this.records; }
  get hasData() { return this.segments.length > 0 || Object.values(this.dropped).some(value => value > 0); }

  private drop(key: keyof ReturnType<typeof emptyDropped>, amount = 1) {
    this.dropped[key] = Math.min(Number.MAX_SAFE_INTEGER, this.dropped[key] + amount);
  }

  /** Retires any previous scope; its late callbacks are ignored. */
  beginConnection(): TimingScope {
    if (this.active) this.retire(this.active);
    const session: Session = { closed: false, segment: this.openSegment('connect') };
    this.active = session;
    const live = () => {
      if (this.active === session && !session.closed) return session.segment;
      this.drop('late_callback');
      return null;
    };
    const guarded = <T>(record: (segment: Segment, value: T) => void) => (value: T) => {
      // Diagnostics must never throw into the voice service.
      try { const segment = live(); if (segment) record(segment, value); } catch { this.drop('invalid_event'); }
    };
    const timingLog = guarded((segment, message: string) => this.addSummary(segment, message));
    return {
      trace: guarded((segment, event: LiveTimingEvent) => this.addPhase(segment, event)),
      log: entry => {
        try {
          if (entry?.type === 'info' && typeof entry.message === 'string' && entry.message.startsWith('Timing ')) timingLog(entry.message);
        } catch { this.drop('invalid_event'); }
      },
      dispatch: guarded((segment, entry: ToolDispatch) => this.addDispatch(segment, entry)),
      close: () => {
        if (this.active === session) this.active = null;
        this.retire(session);
      },
    };
  }

  /** Erases all records, request mappings and counters. An open scope keeps
   * recording into a fresh segment whose origin is the clear time. */
  clear() {
    this.segments.forEach(segment => { segment.requests.clear(); segment.records.length = 0; });
    this.segments = [];
    this.connections = 0;
    this.records = 0;
    this.bytes = 0;
    this.dropped = emptyDropped();
    if (this.active && !this.active.closed) this.active.segment = this.openSegment('after_clear');
  }

  /** Detached JSON text, never larger than maxBytes. */
  snapshot(): string {
    const body = {
      format: FORMAT, clock: CLOCK, caveats: CAVEATS,
      limits: { max_connections: this.limits.connections, max_records: this.limits.records,
        max_bytes: this.limits.bytes, max_requests_per_connection: this.limits.requests },
      dropped: { ...this.dropped },
      truncated: this.dropped.records_over_cap + this.dropped.records_over_bytes + this.dropped.connections_evicted > 0,
      connections: this.segments.map(segment => ({ connection: segment.connection, segment_start: segment.start,
        closed: segment.closed, records: segment.records })),
    };
    const text = JSON.stringify(body);
    return text.length <= this.limits.bytes ? text : JSON.stringify({ ...body, truncated: true, connections: [] });
  }

  private retire(session: Session) {
    session.closed = true;
    session.segment.closed = true;
    session.segment.requests.clear();
  }

  private openSegment(start: Segment['start']): Segment {
    while (this.segments.length >= this.limits.connections) {
      const evicted = this.segments.shift()!;
      evicted.requests.clear();
      this.drop('connections_evicted');
      this.drop('records_evicted', evicted.records.length);
      this.records -= evicted.records.length;
      this.bytes -= evicted.bytes;
      evicted.records.length = 0;
    }
    const segment: Segment = { connection: ++this.connections, start, origin: this.now(), closed: false,
      serverCallbackSeen: false, bytes: SEGMENT_OVERHEAD, records: [], requests: new Map() };
    this.segments.push(segment);
    this.bytes += segment.bytes;
    return segment;
  }

  private add(segment: Segment, record: TimingRecord) {
    if (this.records >= this.limits.records) { this.drop('records_over_cap'); return; }
    const size = JSON.stringify(record).length + 1;
    if (this.bytes + size > this.limits.bytes - HEADER_RESERVE) { this.drop('records_over_bytes'); return; }
    segment.records.push(record);
    segment.bytes += size;
    this.records++;
    this.bytes += size;
  }

  private offset(segment: Segment, at: unknown) {
    if (typeof at !== 'number' || !Number.isFinite(at)) return null;
    const relative = at - segment.origin;
    return relative >= 0 && relative <= MAX_MS ? Math.round(relative * 10) / 10 : null;
  }

  private request(segment: Segment, id: unknown) {
    if (typeof id !== 'string' || !id.length || id.length > MAX_ID_LENGTH) return null;
    const known = segment.requests.get(id);
    if (known !== undefined) return known;
    if (segment.requests.size >= this.limits.requests) { this.drop('request_ids_over_cap'); return null; }
    segment.requests.set(id, segment.requests.size + 1);
    return segment.requests.size;
  }

  private addPhase(segment: Segment, event: LiveTimingEvent) {
    if (!event || typeof event !== 'object') { this.drop('invalid_event'); return; }
    const phase = event.phase;
    if (typeof phase !== 'string' || !PHASES.has(phase)) {
      if (typeof phase === 'string' && HIGH_RATE.has(phase)) this.drop('high_rate_phase');
      else this.drop('unknown_phase');
      return;
    }
    const at = this.offset(segment, event.atMs);
    if (at === null) { this.drop('invalid_event'); return; }
    if (phase === 'server_message_callback') {
      if (segment.serverCallbackSeen) { this.drop('repeated_server_callback'); return; }
      segment.serverCallbackSeen = true;
    }
    const metadata = event.metadata && typeof event.metadata === 'object' ? event.metadata : {};
    const meta: Record<string, Value> = {};
    for (const key of Object.keys(PHASE_FIELDS)) if (own(metadata, key)) meta[key] = PHASE_FIELDS[key](metadata[key]);
    if (own(metadata, 'request_id')) meta.request = this.request(segment, metadata.request_id);
    if (own(metadata, 'tool')) meta.stop_tool = metadata.tool === 'stop';
    this.add(segment, { kind: 'phase', phase, at_ms: at, meta });
  }

  private addSummary(segment: Segment, message: string) {
    const received = this.offset(segment, this.now());
    let parsed: any = null;
    if (message.length <= MAX_TIMING_LOG) { try { parsed = JSON.parse(message.slice('Timing '.length)); } catch { /* malformed */ } }
    const turn = parsed && typeof parsed === 'object' && !Array.isArray(parsed) ? count(1_000_000_000)(parsed.turn) : null;
    if (!turn || received === null) { this.drop('malformed_timing_log'); return; }
    const metrics: Record<string, Value> = {};
    for (const key of SUMMARY_MS) metrics[key] = own(parsed, key) ? ms(parsed[key]) : null;
    metrics.tool_count = own(parsed, 'tool_count') ? count(1000)(parsed.tool_count) : null;
    metrics.native_playback_expected = own(parsed, 'native_playback_expected') ? bool(parsed.native_playback_expected) : null;
    // Absolute performance.now() in the service; re-based onto this segment.
    const release = own(parsed, 'latest_audio_release_sent_at_ms') ? this.offset(segment, parsed.latest_audio_release_sent_at_ms) : null;
    metrics.latest_audio_release_sent_ms = release === null ? null : Math.round(release);
    const reason = typeof parsed.reason === 'string' && REASONS.has(parsed.reason) ? parsed.reason : null;
    this.add(segment, { kind: 'turn_summary', received_at_ms: received, turn, reason, metrics });
  }

  private addDispatch(segment: Segment, entry: ToolDispatch) {
    if (!entry || typeof entry !== 'object') { this.drop('invalid_event'); return; }
    const received = this.offset(segment, this.now());
    if (received === null) { this.drop('invalid_event'); return; }
    this.add(segment, { kind: 'dispatch', received_at_ms: received, request: this.request(segment, entry.id),
      status: status(entry.status), stop_tool: entry.name === 'stop', has_job: entry.job != null });
  }
}
