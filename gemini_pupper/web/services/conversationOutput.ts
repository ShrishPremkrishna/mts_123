import guard from '../../conversation_guard.json';
import { hasMalformedOutcomeFlags } from './operatorFeedback';
import type { LiveServerMessage } from '@google/genai';

export const OUTPUT_LIMITS = Object.freeze({ bytes: 2 * 1024 * 1024, items: 256, text: 16000, timeoutMs: 20000 });
export const BLOCKED_CONVERSATION = 'No confirmed robot action in this reply; check server status.';
const patterns = guard.patterns.map(pattern => new RegExp(pattern, 'i'));
const historySpans = guard.nonassertive_history_spans.map(pattern => new RegExp(pattern, 'gi'));
const historyResidualClaims = guard.history_residual_claim_patterns.map(pattern => new RegExp(pattern, 'i'));
export function screenConversation(text: string): boolean {
  if (text.length > OUTPUT_LIMITS.text) return false;
  let normalized = text.normalize('NFKC').replace(/[‘’]/g, "'").replace(/\s+/g, ' ').trim();
  // Ignore only the embedded history question, then scan every remaining clause.
  // The original output is preserved; this is not a whole-reply exemption.
  const original = normalized;
  for (const span of historySpans) normalized = normalized.replace(span, " ");
  return !patterns.some(pattern => pattern.test(normalized))
    && !(normalized !== original && historyResidualClaims.some(pattern => pattern.test(normalized)));
}
export interface ConversationOutput {
  audio: string[];
  text: string[];
  speech: string[];
  transcript: string;
}
interface Turn extends ConversationOutput {
  sealed: boolean;
  toolBearing: boolean;
  dropped: boolean;
  bytes: number;
  items: number;
  textChars: number;
  timer: ReturnType<typeof setTimeout>;
}
/** Synchronous packet accounting; tools never wait for an output turn to finish. */
export class ConversationOutputGate {
  private turn: Turn | null = null;
  constructor(private release: (output: ConversationOutput) => void, private notice: (message: string) => void) {}
  private start(): Turn {
    const turn = { audio: [], text: [], speech: [], transcript: '', sealed: false, toolBearing: false, dropped: false,
      bytes: 0, items: 0, textChars: 0, timer: undefined } as unknown as Turn;
    turn.timer = setTimeout(() => {
      if (this.turn === turn) this.drop('Conversation output omitted: no complete turn boundary arrived.');
    }, OUTPUT_LIMITS.timeoutMs);
    // Offline Node tests should not be kept alive by a browser-session deadline.
    (turn.timer as any).unref?.();
    this.turn = turn;
    return turn;
  }
  private drop(message?: string) {
    const turn = this.turn;
    if (!turn || turn.dropped) return;
    turn.dropped = true; turn.audio = []; turn.text = []; turn.speech = []; turn.transcript = '';
    clearTimeout(turn.timer);
    if (message) this.notice(message);
  }
  /** Keep a dropped-turn tombstone until its boundary; never play only its tail. */
  discard() { if (!this.turn) this.start(); this.drop(); }
  /** A new deliberate input invalidates existing output, not its first future reply. */
  discardPending() { if (this.turn) this.drop(); }
  close() { if (this.turn) clearTimeout(this.turn.timer); this.turn = null; }
  observe(message: LiveServerMessage, pendingTool: boolean, textModel: boolean) {
    const content = message.serverContent;
    if (content?.interrupted || message.toolCallCancellation?.ids?.length) this.discard();
    const calls = message.toolCall?.functionCalls ?? [];
    const meaningful = !!content?.modelTurn?.parts?.length || !!content?.outputTranscription?.text || calls.length > 0 || !!content?.generationComplete;
    if (!meaningful && !this.turn) return;
    const turn = this.turn ?? this.start();
    // Generation is complete, but its logical turn still owns any late tail.
    // Never open another output turn until the provider closes this one.
    if (turn.sealed) {
      if (content?.turnComplete) { clearTimeout(turn.timer); this.turn = null; }
      return;
    }
    if (pendingTool || calls.some(call => !['speak', 'say', 'narrate'].includes(call.name || ''))) {
      turn.toolBearing = true; this.drop();
    }
    const add = (value: unknown, kind: 'audio' | 'text' | 'speech' | 'transcript') => {
      if (turn.dropped || typeof value !== 'string' || !value) return;
      turn.items++; turn.bytes += value.length * (kind === 'audio' ? 1 : 2);
      if (kind !== 'audio') turn.textChars += value.length;
      if (turn.bytes > OUTPUT_LIMITS.bytes || turn.items > OUTPUT_LIMITS.items || turn.textChars > OUTPUT_LIMITS.text) {
        this.drop('Conversation output omitted: response exceeded the bounded turn buffer.'); return;
      }
      if (kind === 'transcript') turn.transcript += value;
      else turn[kind].push(value);
    };
    for (const part of content?.modelTurn?.parts ?? []) {
      if (part.thought) continue;
      add(part.inlineData?.data, 'audio'); add(part.text, 'text');
    }
    add(content?.outputTranscription?.text, 'transcript');
    if (textModel) for (const call of calls) {
      if (['speak', 'say', 'narrate'].includes(call.name || '')) {
        const args = call.args as any;
        add(args?.text ?? args?.message ?? args?.content, 'speech');
      }
    }
    // The Live protocol places the final output transcript before generationComplete.
    // turnComplete may wait for virtual playback; retain its tombstone but do not
    // wait that extra playback duration. Older providers can close with turnComplete.
    // https://ai.google.dev/api/live#bidigeneratecontentservercontent
    if (!content?.generationComplete && !content?.turnComplete) return;
    clearTimeout(turn.timer); turn.sealed = true;
    if (content?.turnComplete) this.turn = null;
    if (turn.dropped || turn.toolBearing) return;
    if (turn.audio.length && !turn.transcript.trim()) {
      this.notice('Conversation audio omitted: no output transcript was available at the turn boundary.'); return;
    }
    const words = [turn.transcript, turn.text.join(''), ...turn.speech].join(' ');
    if (!screenConversation(words)) { this.notice(BLOCKED_CONVERSATION); return; }
    // Robotics ER's Live TEXT response can arrive only as outputTranscription.
    // Promote it after the same complete-turn/tool/claim checks, never early.
    if (textModel && !turn.audio.length && !turn.text.length && !turn.speech.length && turn.transcript.trim()) {
      turn.text.push(turn.transcript);
    }
    if (turn.audio.length || turn.text.length || turn.speech.length) this.release(turn);
  }
}

/** Only punctuation at word/sentence boundaries is optional; number signs and decimals stay significant. */
export function normalizeReceiptTranscript(text: string): string {
  return text.normalize('NFKC').toLowerCase().replace(/[.!?,;:]+(?=\s|$)/g, '').replace(/\s+/g, ' ').trim();
}
interface ReceiptRequest {
  token: object;
  id: string | null;
  role: 'status' | 'entry' | 'left' | 'right' | 'other';
  returned: boolean;
  accepted: boolean;
  queued: boolean;
}
export const PAW_CLARIFICATION = 'Which paw would you like, left or right?';
interface StatusChoice { text: string; questionAllowed: boolean; current: () => boolean; speak?: (text: string) => void; }
interface ReceiptTurn {
  tools: number;
  statusChoice?: StatusChoice;
  requests: ReceiptRequest[];
  invalid: boolean;
  cancelled: boolean;
  outputSeen: boolean;
  authorized: string | null;
  queuedCueUsed: boolean;
  receiptAudioMuted: boolean;
  codeOwnedJoke?: { current: () => boolean; speak: () => void };
  audio: string[];
  transcript: string;
  text: string;
  bytes: number;
  items: number;
  timer: ReturnType<typeof setTimeout>;
}
export const HANDSHAKE_JOKE = 'At this rate, I should charge for pawtographs.';
/** Exact receipts for a single tool or the class status/[Sit entry]/one-paw chain.
 * The streak counts accepted requests in this connection, never physical handshakes. */
export class NativeReceiptOutputGate {
  private turn: ReceiptTurn | null = null;
  private generationSealed = false;
  private unscopedOutputSeen = false;
  private handshakeStreak = 0;
  constructor(private release: (audio: string[]) => void) {}
  invalidate(resetStreak = true) {
    if (this.turn) {
      if (resetStreak) this.handshakeStreak = 0;
      this.turn.invalid = true;
      if (resetStreak) this.turn.cancelled = true;
      this.turn.audio = []; this.turn.transcript = ''; this.turn.text = '';
    }
  }
  /** New input cancels output but preserves a handshake already counted at generation end. */
  discardPending() {
    this.invalidate(!this.generationSealed);
    if (this.turn) this.turn.queuedCueUsed = false;
  }
  /** One code-owned acceptance cue; actual reply/currentness stays the caller's duty.
   * Muting audio must not discard accepted-request evidence for the fourth joke. */
  hasQueuedCue(turn: ReceiptTurn | null = this.turn) { return !!turn?.queuedCueUsed; }
  claimQueuedCue(turn: ReceiptTurn | null): boolean {
    if (!turn || turn.queuedCueUsed) return false;
    turn.queuedCueUsed = true; turn.receiptAudioMuted = true; turn.audio = [];
    return true;
  }
  /** The fourth joke keeps the complete-turn accepted-chain test. A later
   * extra/failing call or interruption must revoke it before generation ends. */
  offerQueuedCueJoke(turn: ReceiptTurn | null, text: string, current: () => boolean, speak: () => void) {
    if (!turn || this.turn !== turn || this.generationSealed || turn.codeOwnedJoke
        || turn.authorized !== text || !current() || !this.acceptedHandshake(turn)
        || this.handshakeStreak !== 3 || ![
          `Left paw command queued. ${HANDSHAKE_JOKE}`, `Right paw command queued. ${HANDSHAKE_JOKE}`,
        ].includes(text)) return;
    turn.codeOwnedJoke = { current, speak };
  }
  resetContext() { this.handshakeStreak = 0; this.invalidate(); }
  close() { if (this.turn) clearTimeout(this.turn.timer); this.turn = null; this.generationSealed = false; this.unscopedOutputSeen = false; this.handshakeStreak = 0; }
  /** A code-owned cached receipt replaces only this authorized native response.
   * Preserve accepted-request bookkeeping; do not permit later duplicate audio. */
  consumeAuthorized(turn: ReceiptTurn | null, text: string): boolean {
    if (!turn || this.turn !== turn || this.generationSealed || turn.invalid || turn.outputSeen || turn.authorized !== text) return false;
    turn.invalid = true; turn.authorized = null;
    turn.audio = []; turn.transcript = ''; turn.text = '';
    return true;
  }
  private handshakePrefix(turn: ReceiptTurn) {
    return ['status,entry', 'status,left', 'status,right', 'status,entry,left', 'status,entry,right'].includes(turn.requests.map(r => r.role).join(','));
  }
  private acceptedHandshake(turn: ReceiptTurn) {
    const roles = turn.requests.map(r => r.role).join(',');
    return !turn.cancelled && ['status,left', 'status,right', 'status,entry,left', 'status,entry,right'].includes(roles)
      && turn.requests.every(r => r.returned && r.accepted && (r.role === 'status' || r.queued));
  }
  authorize(turn: ReceiptTurn | null, text: string, source: object, result: any): string | null {
    if (!turn || this.turn !== turn || this.generationSealed) return null;
    const request = turn.requests.find(r => r.token === source);
    if (!request || request.returned) { this.invalidate(); return null; }
    request.returned = true;
    const known = result && typeof result.success === 'boolean'
      && !hasMalformedOutcomeFlags(result) && !result.superseded && !result.outcome_unknown && !result.not_dispatched;
    request.accepted = known && result.success === true;
    request.queued = result?.details?.submission?.schema === 'pupper_submission_v1' && result.details.submission.state === 'queued';
    if (!known) { this.invalidate(); return null; }
    if (!request.accepted) {
      this.handshakeStreak = 0;
      if (turn.tools > 1) this.invalidate();
    }
    if (turn.invalid || turn.outputSeen || !text.trim() || text.length > 1000) return null;
    if (turn.tools > 1) {
      if (!this.acceptedHandshake(turn)) return null;
      const paw = turn.requests.at(-1)!.role === 'left' ? 'Left' : 'Right';
      text = `${paw} paw command queued.${this.handshakeStreak === 3 ? ' ' + HANDSHAKE_JOKE : ''}`;
    }
    turn.authorized = text;
    return text;
  }
  /** One settled read-only lookup may offer a fixed question. ER fallback always
   * uses the actual typed receipt; model paraphrase is never synthesized. */
  offerPawClarification(turn: ReceiptTurn | null, source: object, text: string,
      current: () => boolean, speak?: (text: string) => void): { question: string | null } | null {
    if (!turn || this.turn !== turn || this.generationSealed || turn.cancelled || turn.tools !== 1
        || turn.requests.length !== 1 || !current() || !text.trim() || text.length > 1000) return null;
    const request = turn.requests[0];
    if (request.token !== source || request.role !== 'status' || !request.returned || !request.accepted) return null;
    const questionAllowed = !turn.invalid && !turn.outputSeen;
    turn.statusChoice = { text, questionAllowed, current, speak };
    return { question: questionAllowed ? PAW_CLARIFICATION : null };
  }
  observe(message: LiveServerMessage): ReceiptTurn | null {
    const content = message.serverContent;
    const calls = (message.toolCall?.functionCalls ?? []).filter(call => !['speak', 'say', 'narrate'].includes(call.name || ''));
    if (content?.interrupted || message.toolCallCancellation?.ids?.length || calls.some(call => call.name === 'stop')) this.handshakeStreak = 0;
    if (this.generationSealed) {
      const turn = this.turn;
      if (calls.length || content?.interrupted || message.toolCallCancellation?.ids?.length) this.invalidate();
      if (content?.turnComplete) {
        if (turn) clearTimeout(turn.timer);
        this.turn = null; this.generationSealed = false; this.unscopedOutputSeen = false;
      }
      return turn;
    }
    const parts = (content?.modelTurn?.parts ?? []).filter(part => !part.thought);
    const hasOutput = !!content?.outputTranscription?.text || parts.some(part => !!part.text || !!part.inlineData?.data);
    if (calls.length && !this.turn) {
      const turn = { tools: 0, requests: [], invalid: this.unscopedOutputSeen, cancelled: false, outputSeen: false, authorized: null,
        audio: [], transcript: '', text: '', queuedCueUsed: false, receiptAudioMuted: false, bytes: 0, items: 0, timer: undefined } as unknown as ReceiptTurn;
      turn.timer = setTimeout(() => { if (this.turn === turn) this.invalidate(); }, OUTPUT_LIMITS.timeoutMs);
      (turn.timer as any).unref?.(); this.turn = turn;
    }
    const turn = this.turn;
    if (turn) {
      if (calls.length) {
        if (turn.outputSeen) this.invalidate(false);
        turn.authorized = null;
      }
      turn.tools += calls.length;
      for (const call of calls) {
        if (turn.requests.length >= 3) { this.invalidate(); break; }
        const id = typeof call.id === 'string' && call.id ? call.id : null;
        if (id && turn.requests.some(r => r.id === id)) this.invalidate();
        const args = call.args as any;
        const role = call.name === 'robot_status' ? 'status'
          : call.name === 'play_trick' && args?.name === 'sit_stay_shake' ? 'entry'
          : call.name === 'interactive_action' && args?.action === 'shake_left' ? 'left'
          : call.name === 'interactive_action' && args?.action === 'shake_right' ? 'right' : 'other';
        turn.requests.push({ token: call, id, role, returned: false, accepted: false, queued: false });
      }
      if ((turn.tools > 1 && !this.handshakePrefix(turn)) || content?.interrupted || message.toolCallCancellation?.ids?.length) this.invalidate();
      if (hasOutput) {
        turn.outputSeen = true;
        if (!turn.authorized) this.invalidate(false);
      }
      const add = (value: unknown, kind: 'audio' | 'transcript' | 'text') => {
        if (turn.invalid || typeof value !== 'string' || !value) return;
        turn.items++; turn.bytes += value.length * (kind === 'audio' ? 1 : 2);
        if (turn.items > OUTPUT_LIMITS.items || turn.bytes > OUTPUT_LIMITS.bytes
            || (kind !== 'audio' && turn.transcript.length + turn.text.length + value.length > OUTPUT_LIMITS.text)) { this.invalidate(); return; }
        if (kind === 'audio') { if (!turn.receiptAudioMuted) turn.audio.push(value); } else turn[kind] += value;
      };
      for (const part of parts) { add(part.inlineData?.data, 'audio'); add(part.text, 'text'); }
      add(content?.outputTranscription?.text, 'transcript');
    }
    if (hasOutput) this.unscopedOutputSeen = true;
    if (content?.generationComplete || content?.turnComplete) {
      // A duplicate empty closure is not a new unrelated conversational turn.
      const hadActivity = !!turn || this.unscopedOutputSeen;
      this.generationSealed = true;
      const acceptedFourth = turn && this.handshakeStreak === 3 && this.acceptedHandshake(turn);
      if (hadActivity) this.handshakeStreak = turn && this.acceptedHandshake(turn) ? Math.min(4, this.handshakeStreak + 1) : 0;
      if (acceptedFourth && turn?.codeOwnedJoke?.current()) turn.codeOwnedJoke.speak();
      if (turn) {
        clearTimeout(turn.timer);
        const choice = turn.statusChoice;
        const statusOnly = !turn.cancelled && turn.tools === 1 && turn.requests.length === 1
          && turn.requests[0].role === 'status' && turn.requests[0].returned && turn.requests[0].accepted;
        const currentChoice = statusOnly && choice?.current() ? choice : null;
        const words = normalizeReceiptTranscript(turn.transcript || turn.text);
        const consistent = !turn.text || !turn.transcript || normalizeReceiptTranscript(turn.text) === words;
        const questionChosen = currentChoice?.questionAllowed && !turn.invalid && consistent
          && words === normalizeReceiptTranscript(PAW_CLARIFICATION);
        // A silent status-only generation must not sound like the requested
        // action happened. This reports only this reply's actual dispatches;
        // it neither infers user intent nor requests an action or a retry.
        if (currentChoice?.speak) currentChoice.speak(questionChosen ? PAW_CLARIFICATION
          : !turn.outputSeen ? 'I checked status, but sent no action request in this reply.' : currentChoice.text);
        const expected = questionChosen ? normalizeReceiptTranscript(PAW_CLARIFICATION)
          : turn.authorized && normalizeReceiptTranscript(turn.authorized);
        if (!turn.invalid && !turn.receiptAudioMuted && expected && turn.audio.length && normalizeReceiptTranscript(turn.transcript) === expected
            && (!turn.text || normalizeReceiptTranscript(turn.text) === expected)) this.release(turn.audio);
      }
      if (content?.turnComplete) {
        this.turn = null; this.generationSealed = false; this.unscopedOutputSeen = false;
      }
    }
    return turn;
  }
}
