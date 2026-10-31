import type { OutputPolicy } from '../types';

export function normalizeOutputPolicy(value: unknown): OutputPolicy {
  return value === 'conversation' ? 'conversation' : 'receipts';
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

/** Closed vocabulary only: never read arbitrary tool names, arguments or server prose aloud. */
export function operatorFeedback(name: unknown, value: unknown): string {
  const result = value as any, label = toolLabel(name);
  if (result?.superseded === true) return 'This request was superseded. Earlier execution is possible.';
  if (!result || typeof result !== 'object' || hasMalformedOutcomeFlags(result) || result.outcome_unknown)
    return `The ${label} response is unknown. Check server status and the robot.`;
  if (result.not_dispatched === true) return `The ${label} request was not dispatched.`;
  if (typeof result.success !== 'boolean') return `The ${label} response is unknown. Check server status and the robot.`;
  if (result.success === false) return `Server reported the ${label} request failed. Earlier effects are not ruled out.`;
  const submission = result.details?.submission;
  const queued = typeof name === 'string' && QUEUED_TOOLS.has(name)
    && submission?.schema === 'pupper_submission_v1' && submission.state === 'queued';
  return `Server ${queued ? 'queued' : 'accepted'} the ${label} request. Completion is unconfirmed.`
    + (name === 'stop' ? ' Physical stopping is unverified.' : '');
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
