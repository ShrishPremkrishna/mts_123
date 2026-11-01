import type { OutputPolicy } from '../types';

export function normalizeOutputPolicy(value: unknown): OutputPolicy {
  return value === undefined || value === 'conversation' ? 'conversation' : 'receipts';
}
const LABELS: Record<string, string> = Object.freeze({
  stand_up: 'stand', lie_down: 'leg relaxation', move: 'move', turn: 'turn', arc: 'arc', wait: 'wait',
  stop: 'Stop', robot_status: 'status', play_trick: 'trick', interactive_action: 'interactive action',
  follow: 'follow', unlock: 'unlock', lock_tricks: 'lock',
});
const QUEUED_TOOLS = new Set(['stand_up', 'lie_down', 'move', 'turn', 'arc', 'wait', 'play_trick', 'interactive_action']);
export function toolLabel(name: unknown): string {
  return typeof name === 'string' && Object.prototype.hasOwnProperty.call(LABELS, name) ? LABELS[name] : 'robot';
}
export function hasMalformedOutcomeFlags(value: any): boolean {
  return !!value && typeof value === 'object'
    && ['superseded', 'outcome_unknown', 'not_dispatched'].some(flag => flag in value && typeof value[flag] !== 'boolean');
}

function numberText(value: unknown): string | null {
  return typeof value === 'number' && Number.isFinite(value) && value >= 0 && value <= 1_000_000
    && !(value > 0 && value < .000001) ? String(value) : null;
}
function motionReason(details: any): string | null {
  const reason = details?.refusal;
  if (!reason || reason.schema !== 'pupper_motion_refusal_v1' || !reason.limits || typeof reason.limits !== 'object' || Array.isArray(reason.limits)) return null;
  const fields: Record<string, string[]> = {
    speed_limit: ['forward_max', 'backward_max', 'sideways_max', 'turn_max'],
    minimum_speed: ['forward_min', 'sideways_min', 'turn_min'], duration_limit: ['duration_max'],
    distance_limit: ['distance_max'], radius_limit: ['radius_min'], invalid_values: [],
  };
  if (typeof reason.code !== 'string' || !Object.prototype.hasOwnProperty.call(fields, reason.code)) return null;
  const n = fields[reason.code].map(key => numberText(reason.limits[key]));
  if (n.some(value => value === null)) return null;
  switch (reason.code) {
    case 'speed_limit': return `speed exceeds the limit: ${n[0]} m/s forward, ${n[1]} backward, ${n[2]} sideways, ${n[3]} deg/s turning.`;
    case 'minimum_speed': return `speed below minimum. Use at least ${n[0]} m/s forward/backward, ${n[1]} sideways or ${n[2]} deg/s turning.`;
    case 'duration_limit': return `duration must be above zero and at most ${n[0]} seconds.`;
    case 'distance_limit': return `commanded distance exceeds ${n[0]} metres.`;
    case 'radius_limit': return `arc radius must be at least ${n[0]} metres at that speed.`;
    default: return 'motion values must be finite numbers.';
  }
}
function statusFeedback(details: any): string | null {
  if (!details || typeof details !== 'object' || typeof details.running !== 'boolean'
      || numberText(details.queued) === null || !Number.isInteger(details.queued)) return null;
  const modes: Record<string, string> = {locomotion: 'locomotion', 'motors idle': 'motors idle', fallen: 'fall stop latched', 'locked trick': 'locked skill', sit_stay_shake: 'Sit controller selected', secret_dance: 'generated-motion controller selected'};
  const mode = typeof details.state === 'string' && Object.prototype.hasOwnProperty.call(modes, details.state) ? modes[details.state] : null;
  const observedModes: Record<string, string> = {locomotion: 'locomotion', none: 'no active command controller', unknown: 'unknown', sit_stay_shake: 'Sit', secret_dance: 'generated motion', other_skill: 'skill', walking_skill: 'walking skill', locked_skill: 'locked skill'};
  const observed = details.observed_controller_mode;
  const observation = typeof observed === 'string' && Object.prototype.hasOwnProperty.call(observedModes, observed) ? observedModes[observed] : null;
  const parts = observation ? [`Controller: ${observation}.`] : [mode ? `Cached mode: ${mode}.` : 'Cached status:', 'Controller unverified.'];
  const current = details.current_action;
  if (details.running) {
    const name = current?.state === 'running' ? current.name : null;
    let kind = 'command';
    if (typeof name === 'string') {
      if (name.startsWith('move ')) kind = 'movement command';
      else if (name.startsWith('wait ')) kind = 'wait command';
      else if (name.startsWith('trick ') || name === 'locked trick') kind = 'skill command';
      else if (name === 'stand up') kind = 'stand command';
      else if (name === 'lie down') kind = 'leg-relaxation command';
    }
    parts.push(`Running a ${kind}; ${details.queued} queued.`);
  } else if (current != null) parts.push(`Current action unavailable; ${details.queued} queued.`);
  else parts.push(`No command running; ${details.queued} queued.`);
  if (typeof details.following === 'boolean') parts.push(details.following ? 'Following on.' : 'Following off.');
  if (typeof details.controller_fault === 'string' && details.controller_fault) parts.push('Controller fault reported.');
  return parts.join(' ');
}

/** Closed vocabulary only: never read arbitrary tool names, arguments or server prose aloud. */
export function operatorFeedback(name: unknown, value: unknown): string {
  const result = value as any, label = toolLabel(name), spoken = label[0].toUpperCase() + label.slice(1);
  if (result?.superseded === true) return 'That request was superseded.';
  if (!result || typeof result !== 'object' || hasMalformedOutcomeFlags(result) || result.outcome_unknown)
    return 'I could not confirm that request.';
  if (result.not_dispatched === true) return `${spoken} request not sent.`;
  if (typeof result.success !== 'boolean') return 'I could not confirm that request.';
  if (result.success === false) {
    const reason = ['move', 'turn', 'arc'].includes(name as string) ? motionReason(result.details) : null;
    if (reason) return `${spoken} rejected: ${reason}`;
    if (name !== 'stop' && name !== 'robot_status' && result.command_epoch_required === true) return `${spoken} rejected: command authorization is stale or missing. Check status and make a new request.`;
    return 'That request failed. Check the controls for details.';
  }
  if (name === 'robot_status') { const status = statusFeedback(result.details); if (status) return status; }
  const submission = result.details?.submission;
  const queued = typeof name === 'string' && QUEUED_TOOLS.has(name)
    && submission?.schema === 'pupper_submission_v1' && submission.state === 'queued';
  return name === 'stop' ? 'Stop requested.' : `${spoken} ${queued ? 'queued' : 'request accepted'}.`;
}

/** At most one active utterance and one newest pending receipt. Never awaited by dispatch. */
export class ReceiptSpeaker {
  private active: { utterance: SpeechSynthesisUtterance; timer?: ReturnType<typeof setTimeout> } | null = null;
  private pending: { text: string; expires: number } | null = null;
  say(text: string) {
    try { this.enqueue(text); } catch { this.cancel(); }
  }
  private enqueue(text: string) {
    if (typeof window === 'undefined' || !window.speechSynthesis || typeof SpeechSynthesisUtterance === 'undefined') return;
    const receipt = { text: text.slice(0, 240), expires: Date.now() + 8000 };
    if (this.active) { this.pending = receipt; return; }
    this.start(receipt);
  }
  private start(receipt: { text: string; expires: number }) {
    if (receipt.expires <= Date.now()) return;
    let utterance: SpeechSynthesisUtterance;
    try { utterance = new SpeechSynthesisUtterance(receipt.text); } catch { return; }
    const active = { utterance, timer: undefined as ReturnType<typeof setTimeout> | undefined };
    this.active = active;
    const finish = () => {
      if (this.active !== active) return;
      clearTimeout(active.timer); this.active = null;
      const pending = this.pending; this.pending = null;
      if (pending) this.start(pending);
    };
    utterance.onend = finish; utterance.onerror = finish;
    active.timer = setTimeout(() => {
      if (this.active !== active) return;
      // Retire callbacks before cancelling; a synchronous onend must not start stale speech.
      const pending = this.pending; this.pending = null; this.active = null;
      try { window.speechSynthesis.cancel(); } catch { /* Optional audio cannot affect dispatch. */ }
      if (pending) this.start(pending);
    }, 8000);
    try { window.speechSynthesis.speak(utterance); } catch { finish(); }
  }
  cancel() {
    if (this.active) clearTimeout(this.active.timer);
    this.active = null; this.pending = null;
    try { if (typeof window !== 'undefined' && window.speechSynthesis) window.speechSynthesis.cancel(); } catch { /* Stop must proceed. */ }
  }
}
