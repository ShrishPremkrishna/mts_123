import { hasMalformedOutcomeFlags } from './operatorFeedback';
/** Execution evidence comes only from actual requests and robot responses. */
export interface ToolDispatch {
  id: string;
  name: string;
  status: 'sending' | 'accepted' | 'rejected' | 'unknown';
  message: string;
  timestamp: number;
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

/** No per-job IDs exist in queue/status: never attribute its last outcome to a call. */
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
