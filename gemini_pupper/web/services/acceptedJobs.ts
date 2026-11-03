import { matchJobEvidence, type QueueStatus, type ToolDispatch } from './executionStatus';

/** One fresh ExecutionStatus poll result, with the bridge and Stop generation it used. */
export interface StatusReport {
  bridge: object;
  serverUrl: string;
  status: QueueStatus;
  pollStartedAt: number;
  receivedAt: number;
  generation: number;
}

export interface TrackedJob {
  dispatch: ToolDispatch;
  turn: number;
  generation: number;
  epoch: string;
  acceptedAt: number;
}

export interface LateJobFailure extends TrackedJob { text: string; }

const MAX_JOBS = 64;
const MAX_FAILED_TURNS = 16;
// Dropping only silences a job; unknown evidence never becomes completion/failure.
const MAX_AGE_MS = 5 * 60_000;
const FRESH_MS = 2500;

function sessionInstance(status: QueueStatus): string | null {
  const context = status.session_context as Record<string, unknown> | null | undefined;
  const instance = context && typeof context === 'object' ? context.server_instance : undefined;
  return typeof instance === 'string' && /^[a-f0-9]{32}$/.test(instance) ? instance : null;
}

/** Accepted voice jobs awaiting an exact terminal record. Bounded; Stop/disconnect clear it. */
export class AcceptedJobTracker {
  private jobs = new Map<string, TrackedJob>();
  private failedTurns: number[] = [];

  /** Only a clean queued acceptance with exact identity, URL and sent command epoch. */
  track(dispatch: ToolDispatch, turn: number, generation: number, epoch: string | undefined, now = Date.now()) {
    const job = dispatch.status === 'accepted' ? dispatch.job : undefined;
    if (!job || !dispatch.serverUrl || !epoch || !epoch.startsWith(`${job.server_instance}:`)) return;
    this.jobs.delete(job.job_id);
    this.jobs.set(job.job_id, { dispatch: { ...dispatch, timestamp: now }, turn, generation, epoch, acceptedAt: now });
    while (this.jobs.size > MAX_JOBS) this.jobs.delete(this.jobs.keys().next().value!);
  }

  /** True while jobs from this Stop generation remain; older generations are discarded. */
  pending(generation: number, now = Date.now()): boolean {
    for (const [id, job] of this.jobs) {
      if (job.generation !== generation || now - job.acceptedAt > MAX_AGE_MS) this.jobs.delete(id);
    }
    return this.jobs.size > 0;
  }

  /** A turn that already produced a failure cue stays quiet for related failures. */
  noteFailedTurn(turn: number) {
    if (this.failedTurns.includes(turn)) return;
    this.failedTurns.push(turn);
    if (this.failedTurns.length > MAX_FAILED_TURNS) this.failedTurns.shift();
  }

  failedTurn(turn: number): boolean { return this.failedTurns.includes(turn); }

  clear() { this.jobs.clear(); }

  /** Remove every exactly matched terminal job; return only same-epoch failures. */
  observe(report: StatusReport, generation: number, now = Date.now()): LateJobFailure[] {
    if (!this.pending(generation, now) || report.generation !== generation) return [];
    const fresh = now - report.receivedAt >= 0 && now - report.receivedAt <= FRESH_MS;
    const instance = fresh ? sessionInstance(report.status) : null;
    const failures: LateJobFailure[] = [];
    for (const [id, job] of this.jobs) {
      const evidence = matchJobEvidence(job.dispatch, report.status,
        { serverUrl: report.serverUrl, pollStartedAt: report.pollStartedAt, fresh });
      if (evidence.state === 'succeeded' || evidence.state === 'cancelled') this.jobs.delete(id);
      else if (evidence.state === 'failed') {
        this.jobs.delete(id);
        if (report.status.command_epoch === job.epoch) failures.push({ ...job, text: evidence.text });
      } else if (evidence.state === 'unknown' && instance && report.serverUrl === job.dispatch.serverUrl
          && instance !== job.dispatch.job!.server_instance) this.jobs.delete(id); // Restarted: never observable here.
    }
    return failures;
  }
}
