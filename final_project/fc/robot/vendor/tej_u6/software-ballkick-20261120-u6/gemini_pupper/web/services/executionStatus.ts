import { hasMalformedOutcomeFlags } from './operatorFeedback';
/** Execution evidence comes only from actual requests and robot responses. */
export interface JobIdentity {
  job_id: string;
  server_instance: string;
}

export interface ToolDispatch {
  id: string;
  name: string;
  status: 'sending' | 'accepted' | 'rejected' | 'unknown';
  message: string;
  timestamp: number;
  job?: JobIdentity;
  serverUrl?: string;
}

export interface QueueStatus {
  runtime?: 'fake' | 'robot';
  hardware_connected?: boolean | null;
  queued: number;
  running: boolean;
  following: boolean;
  state: string;
  state_after_queue?: string;
  state_basis?: string;
  observed_controller_mode?: string;
  controller_fault: string | null;
  last_outcome: { name: string; success: boolean; message: string } | null;
  current_action?: unknown;
  session_context?: unknown;
  recent_outcomes?: unknown;
  command_epoch?: unknown;
  capabilities?: unknown;
}

export function parseQueueStatus(value: unknown): QueueStatus | null {
  const v = value as Partial<QueueStatus> | null;
  if (!v || typeof v !== 'object' || !Number.isInteger(v.queued) || v.queued! < 0
      || typeof v.running !== 'boolean' || typeof v.following !== 'boolean'
      || typeof v.state !== 'string'
      || (v.state_after_queue !== undefined && typeof v.state_after_queue !== 'string')
      || (v.state_basis !== undefined && typeof v.state_basis !== 'string')
      || (v.observed_controller_mode !== undefined && typeof v.observed_controller_mode !== 'string')
      || !(v.controller_fault === null || typeof v.controller_fault === 'string')) return null;
  const response = v as Record<string, unknown>;
  if (hasMalformedOutcomeFlags(response) || response.success === false || response.outcome_unknown
      || response.superseded || response.not_dispatched) return null;
  const outcome = v.last_outcome;
  if (outcome !== null && (!outcome || typeof outcome.name !== 'string'
      || typeof outcome.success !== 'boolean' || typeof outcome.message !== 'string')) return null;
  return v as QueueStatus;
}

export function toolResponseStatus(result: unknown): ToolDispatch['status'] {
  const r = result as { success?: boolean; outcome_unknown?: boolean; superseded?: boolean; not_dispatched?: boolean } | null;
  if (!r || hasMalformedOutcomeFlags(r) || r.outcome_unknown || r.superseded || typeof r.success !== 'boolean') return 'unknown';
  if (r.not_dispatched) return 'rejected';
  return r.success ? 'accepted' : 'rejected';
}

const object = (value: unknown): Record<string, any> | null =>
  value !== null && typeof value === 'object' && !Array.isArray(value) ? value as Record<string, any> : null;

function jobIdentity(value: unknown): JobIdentity | null {
  const v = object(value);
  if (!v || typeof v.server_instance !== 'string' || v.server_instance.length !== 32 || !/^[a-f0-9]{32}$/.test(v.server_instance)
      || typeof v.job_id !== 'string' || v.job_id.length > 128 || v.job_id.trim() !== v.job_id
      || !v.job_id.startsWith(`${v.server_instance}:`) || !/^[a-f0-9]{32}:[1-9][0-9]*$/.test(v.job_id)) return null;
  return { job_id: v.job_id, server_instance: v.server_instance };
}

/** A name, model call ID, message or uncertain response cannot establish a job. */
export function acceptedJobIdentity(result: unknown): JobIdentity | undefined {
  const r = object(result), submission = object(r?.details)?.submission;
  if (toolResponseStatus(result) !== 'accepted' || submission?.schema !== 'pupper_submission_v1'
      || submission.state !== 'queued') return undefined;
  return jobIdentity(submission) ?? undefined;
}

export interface MatchedJobEvidence {
  state: 'unknown' | 'claimed' | 'running' | 'succeeded' | 'failed' | 'cancelled';
  text: string;
}

/** Exact retained IDs only. The result concerns this one API job, not a physical outcome or multi-job route. */
export function matchJobEvidence(
  dispatch: ToolDispatch | null, status: QueueStatus | null,
  source: { serverUrl: string; pollStartedAt: number; fresh: boolean },
): MatchedJobEvidence {
  const unknown = (text: string): MatchedJobEvidence => ({ state: 'unknown', text });
  const job = dispatch?.status === 'accepted' ? jobIdentity(dispatch.job) : null;
  if (!job) return unknown('No confirmed queued-job identity for this request.');
  if (dispatch!.serverUrl !== source.serverUrl) return unknown('This receipt belongs to a different server URL.');
  if (!status || !source.fresh) return unknown('Current job evidence is unavailable or stale.');
  if (source.pollStartedAt < dispatch!.timestamp) return unknown('Waiting for a status check after this receipt.');
  const context = object(status.session_context);
  if (context?.schema !== 'pupper_session_context_v1' || context.scope !== 'this_server_session'
      || typeof context.server_instance !== 'string' || context.server_instance.length !== 32 || !/^[a-f0-9]{32}$/.test(context.server_instance)) {
    return unknown('Server instance evidence is missing or invalid.');
  }
  if (context.server_instance !== job.server_instance) return unknown('The server restarted or changed; this job is unconfirmed here.');
  if (status.command_epoch !== undefined && (typeof status.command_epoch !== 'string'
      || !status.command_epoch.startsWith(`${job.server_instance}:`))) return unknown('Server identity fields disagree.');
  const current = status.current_action === null ? null : object(status.current_action);
  if (status.current_action !== null && (!current || !jobIdentity(current)
      || current.server_instance !== context.server_instance || current.state !== 'running'
      || typeof current.execution_started !== 'boolean')) return unknown('Current job identity is missing or invalid.');
  const history = object(status.recent_outcomes);
  if (!history || !Number.isSafeInteger(history.capacity) || history.capacity < 1 || history.capacity > 256
      || !Number.isSafeInteger(history.dropped) || history.dropped < 0
      || !Array.isArray(history.outcomes) || history.outcomes.length > history.capacity) {
    return unknown('Retained job history is missing or invalid.');
  }
  const terminals: Record<string, any>[] = [];
  const ids = new Set<string>();
  for (const raw of history.outcomes) {
    const row = object(raw), id = jobIdentity(row);
    if (!row || !id || id.server_instance !== context.server_instance || ids.has(id.job_id)
        || !['succeeded', 'failed', 'cancelled'].includes(row.terminal_state)
        || typeof row.execution_started !== 'boolean'
        || (row.terminal_state === 'succeeded' && !row.execution_started)
        || typeof row.success !== 'boolean' || row.success !== (row.terminal_state === 'succeeded')
        || !Number.isSafeInteger(row.event_seq) || row.event_seq < 1) {
      return unknown('Retained job history is malformed or contradictory.');
    }
    ids.add(id.job_id);
    if (id.job_id === job.job_id) terminals.push(row);
  }
  const terminal = terminals[0];
  if (terminal) {
    // A terminal record can appear before the queue worker clears current_action.
    const suffix = terminal.execution_started ? '' : ' before execution started';
    return { state: terminal.terminal_state, text: `Server job ${terminal.terminal_state}${suffix}. Physical outcome is unverified.` };
  }
  if (current?.job_id === job.job_id && current.server_instance === job.server_instance) {
    if (!status.running) return unknown('Current job conflicts with the queue running flag.');
    return current.execution_started
      ? { state: 'running', text: 'Server job running. Physical outcome is unverified.' }
      : { state: 'claimed', text: 'Server worker has this job; execution has not started.' };
  }
  return unknown(history.dropped > 0
    ? 'No matching retained outcome; older history has been evicted. Completion is unconfirmed.'
    : 'Accepted earlier; no matching current job or retained outcome. Completion is unconfirmed.');
}

/** Controller/queue summary is independent of any matched per-job result. */
export function queueSummary(status: QueueStatus): string {
  const modes: Record<string, string> = {
    locomotion: 'locomotion', none: 'no active command controller',
    sit_stay_shake: 'Sit', secret_dance: 'generated motion', other_skill: 'skill',
    walking_skill: 'walking skill', locked_skill: 'locked skill',
  };
  const observed = status.observed_controller_mode;
  const controller = observed && Object.prototype.hasOwnProperty.call(modes, observed) ? modes[observed] : 'unverified';
  // The cached planner remains in the structured response for diagnostics. It
  // must not compete with the observed controller when no command is pending.
  const destinations: Record<string, string> = {
    ...modes, 'motors idle': modes.none, 'locked trick': modes.locked_skill,
  };
  const planned = status.state_after_queue;
  const next = planned && Object.prototype.hasOwnProperty.call(destinations, planned) ? destinations[planned] : null;
  const transition = (status.running || status.queued > 0) && next && next !== controller ? ` · Next: ${next}` : '';
  return `Controller: ${controller} · ${status.running ? 'action running' : 'no queued action running'} · ${status.queued} queued${transition}${status.following ? ' · following' : ''}`;
}

/** Pause further model dispatch synchronously, then use the direct server stop. */
export function requestIndependentStop(
  model: { pauseRobotTools(): void } | null,
  bridge: { immediateStop(): Promise<any> },
): Promise<any> {
  model?.pauseRobotTools();
  return bridge.immediateStop();
}

/** A late response may update only the request currently displayed. */
export function updateLatestDispatch(current: ToolDispatch | null, entry: ToolDispatch): ToolDispatch | null {
  return entry.status === 'sending' || current?.id === entry.id ? entry : current;
}

/** API mode is not evidence of physical connectivity. Missing metadata stays unknown. */
export function runtimeSummary(status: QueueStatus): string {
  if (status.runtime === 'fake') return 'FAKE REHEARSAL — no robot hardware; commands simulate queue behavior only';
  if (status.runtime === 'robot') return 'ROBOT API — physical connection and motion require observation';
  return 'Runtime not identified — do not infer hardware from this server response';
}

/** Capability metadata is configuration evidence, never collision-free motion proof. */
export function motionCapabilitySummary(status: QueueStatus | null, fresh: boolean): string {
  const c = object(status?.capabilities);
  if (!fresh || !c || c.schema !== 'pupper_motion_capabilities_v1'
      || typeof c.autonomous_visual_navigation !== 'boolean'
      || typeof c.collision_avoidance !== 'boolean'
      || typeof c.person_following !== 'boolean') {
    return 'Navigation and obstacle avoidance: unverified';
  }
  const navigation = c.autonomous_visual_navigation ? 'reported available' : 'unavailable';
  const avoidance = c.collision_avoidance ? 'reported available' : 'unavailable';
  const following = c.person_following ? 'configured' : 'unavailable';
  return `Visual navigation: ${navigation} · Obstacle avoidance: ${avoidance} · Follower service: ${following}`;
}
