import guard from '../../conversation_guard.json';
import type { LiveServerMessage } from '@google/genai';

export const OUTPUT_LIMITS = Object.freeze({ bytes: 2 * 1024 * 1024, items: 256, text: 16000, timeoutMs: 20000 });
export const BLOCKED_CONVERSATION = 'No confirmed robot action in this reply; check server status.';
const patterns = guard.patterns.map(pattern => new RegExp(pattern, 'i'));
export function screenConversation(text: string): boolean {
  if (text.length > OUTPUT_LIMITS.text) return false;
  const normalized = text.normalize('NFKC').replace(/[‘’]/g, "'").replace(/\s+/g, ' ').trim();
  return !patterns.some(pattern => pattern.test(normalized));
}
export interface ConversationOutput {
  audio: string[];
  text: string[];
  speech: string[];
  transcript: string;
}
interface Turn extends ConversationOutput {
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
    const turn = { audio: [], text: [], speech: [], transcript: '', toolBearing: false, dropped: false,
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
  close() { if (this.turn) clearTimeout(this.turn.timer); this.turn = null; }
  observe(message: LiveServerMessage, pendingTool: boolean, textModel: boolean) {
    const content = message.serverContent;
    if (content?.interrupted || message.toolCallCancellation?.ids?.length) this.discard();
    const calls = message.toolCall?.functionCalls ?? [];
    const meaningful = !!content?.modelTurn?.parts?.length || !!content?.outputTranscription?.text || calls.length > 0;
    if (!meaningful && !this.turn) return;
    const turn = this.turn ?? this.start();
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
    // generationComplete/transcription.finished are not an aligned turn boundary.
    if (!content?.turnComplete) return;
    clearTimeout(turn.timer); this.turn = null;
    if (turn.dropped || turn.toolBearing) return;
    if (turn.audio.length && !turn.transcript.trim()) {
      this.notice('Conversation audio omitted: no output transcript was available at the turn boundary.'); return;
    }
    const words = [turn.transcript, turn.text.join(''), ...turn.speech].join(' ');
    if (!screenConversation(words)) { this.notice(BLOCKED_CONVERSATION); return; }
    if (turn.audio.length || turn.text.length || turn.speech.length) this.release(turn);
  }
}
