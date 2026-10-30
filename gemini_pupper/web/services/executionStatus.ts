/** Execution evidence comes only from actual requests and robot responses. */
export interface ToolDispatch {
  id: string;
  name: string;
  status: 'sending' | 'accepted' | 'rejected' | 'unknown';
  message: string;
  timestamp: number;
}

export interface QueueStatus {
  queued: number;
  running: boolean;
  following: boolean;
  state: string;
  controller_fault: string | null;
  last_outcome: { name: string; success: boolean; message: string } | null;
}

export function parseQueueStatus(value: unknown): QueueStatus | null {
  const v = value as Partial<QueueStatus> | null;
  if (!v || typeof v !== 'object' || !Number.isInteger(v.queued) || v.queued! < 0
      || typeof v.running !== 'boolean' || typeof v.following !== 'boolean'
      || typeof v.state !== 'string'
      || !(v.controller_fault === null || typeof v.controller_fault === 'string')) return null;
  const outcome = v.last_outcome;
  if (outcome !== null && (!outcome || typeof outcome.name !== 'string'
      || typeof outcome.success !== 'boolean' || typeof outcome.message !== 'string')) return null;
  return v as QueueStatus;
}

export function toolResponseStatus(result: unknown): ToolDispatch['status'] {
  const r = result as { success?: boolean; outcome_unknown?: boolean } | null;
  if (!r || r.outcome_unknown || typeof r.success !== 'boolean') return 'unknown';
  return r.success ? 'accepted' : 'rejected';
}

/** No per-job IDs exist in queue/status: never attribute its last outcome to a call. */
export function queueSummary(status: QueueStatus): string {
  return `${status.state} · ${status.running ? 'action running' : 'no queued action running'} · ${status.queued} queued${status.following ? ' · following' : ''}`;
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
