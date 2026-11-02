import { normalizeReceiptTranscript } from './conversationOutput';
import { hasMalformedOutcomeFlags, queuedHandshakePaw } from './operatorFeedback';

const MODEL = 'gemini-3.1-flash-live-preview';
const VOICE = 'Puck';
/** Asset model remains its generation provenance; this closed list names consumers. */
export function supportsPuckReceiptConsumer(model: string, voice: string, ttsModel?: string): boolean {
  const name = model.replace(/^models\//, '');
  return voice === VOICE && (name === MODEL || (name === 'gemini-robotics-er-2-streaming-preview'
    && (ttsModel || 'gemini-3.1-flash-tts-preview') === 'gemini-3.1-flash-tts-preview'));
}
const PHRASES = new Set([
  'Okay.', 'Move queued.', 'Stop requested.', 'Left paw command queued.', 'Right paw command queued.',
  'Stand queued.', 'Turn queued.', 'Arc queued.', 'Wait queued.', 'Trick queued.', 'Interactive action queued.',
  'That request failed. Check the controls for details.', 'Exact physical repetition counts are not supported.',
]);
const MAX_CLIP_BYTES = 24000 * 2 * 10;

/** Select only code-owned phrases. Caller must separately require a current
 * actual server reply; this helper cannot establish transport provenance. */
export function serverReceiptAudioText(name: unknown, args: any, result: any): string | null {
  if (!result || typeof result !== 'object' || hasMalformedOutcomeFlags(result)
      || result.outcome_unknown || result.superseded || result.not_dispatched
      || typeof result.success !== 'boolean') return null;
  if (result.success === false) {
    const refusal = result.details?.refusal;
    return name === 'play_trick' && refusal?.schema === 'pupper_trick_quantity_refusal_v1'
      && refusal.code === 'physical_count_unsupported'
      ? 'Exact physical repetition counts are not supported.'
      : 'That request failed. Check the controls for details.';
  }
  if (name === 'stop') return 'Stop requested.';
  if (name === 'shake_hand') {
    const paw = queuedHandshakePaw(result, args?.paw);
    return paw ? `${paw === 'left' ? 'Left' : 'Right'} paw command queued.` : null;
  }
  const submission = result.details?.submission;
  if (submission?.schema !== 'pupper_submission_v1' || submission.state !== 'queued') return null;
  const labels: Record<string, string> = {
    stand_up: 'Stand queued.', move: 'Move queued.', turn: 'Turn queued.', arc: 'Arc queued.',
    wait: 'Wait queued.', play_trick: 'Trick queued.', interactive_action: 'Interactive action queued.',
  };
  if (name === 'interactive_action' && args?.action === 'shake_left') return 'Left paw command queued.';
  if (name === 'interactive_action' && args?.action === 'shake_right') return 'Right paw command queued.';
  return typeof name === 'string' && Object.prototype.hasOwnProperty.call(labels, name) ? labels[name] : null;
}

interface ReadyClip { audio: string; approved: boolean; }

/** Explicitly supplied, preloaded audio. No fetch or decoding runs on tool dispatch. */
export class ReceiptAudioCache {
  private constructor(private readonly clips: ReadonlyMap<string, ReadyClip>) {}

  lookup(text: string, model: string, voice: string, ttsModel?: string): string | null {
    if (!supportsPuckReceiptConsumer(model, voice, ttsModel)) return null;
    const clip = this.clips.get(text);
    return clip?.approved ? clip.audio : null;
  }

  static async load(manifest: unknown, readAsset: (file: string) => Promise<Uint8Array>): Promise<ReceiptAudioCache> {
    const value = manifest as any;
    if (value?.schema !== 'pupper_receipt_audio_v1' || value.model !== MODEL || value.voice !== VOICE
        || value.encoding !== 'pcm_s16le' || value.sample_rate_hz !== 24000 || value.channels !== 1
        || !Array.isArray(value.clips) || !value.clips.length || value.clips.length > PHRASES.size) {
      throw new Error('Unsupported receipt audio manifest');
    }
    const clips = new Map<string, ReadyClip>();
    // Snapshot untrusted manifest fields before any asynchronous asset reader.
    for (const item of value.clips.map((entry: any) => ({ ...entry }))) {
      if (!item || !PHRASES.has(item.text) || clips.has(item.text) || typeof item.transcript !== 'string'
          || normalizeReceiptTranscript(item.transcript) !== normalizeReceiptTranscript(item.text)
          || typeof item.file !== 'string' || !/^[a-zA-Z0-9_-]+\.pcm$/.test(item.file)
          || typeof item.sha256 !== 'string' || !/^[a-f0-9]{64}$/.test(item.sha256) || item.sha256.length !== 64
          || !Number.isSafeInteger(item.bytes) || item.bytes < 2 || item.bytes > MAX_CLIP_BYTES || item.bytes % 2
          || typeof item.duration_s !== 'number' || !Number.isFinite(item.duration_s)
          || Math.abs(item.duration_s - item.bytes / 48000) > 1 / 24000
          || !['pending_human', 'approved_human'].includes(item.review_status)) {
        throw new Error('Invalid receipt audio clip');
      }
      const returned = await readAsset(item.file);
      if (!(returned instanceof Uint8Array) || returned.byteLength !== item.bytes) throw new Error('Receipt audio size mismatch');
      const data = new Uint8Array(returned);
      const digest = Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', data)))
        .map(byte => byte.toString(16).padStart(2, '0')).join('');
      if (digest !== item.sha256) throw new Error('Receipt audio hash mismatch');
      let binary = '';
      for (let start = 0; start < data.length; start += 8192) binary += String.fromCharCode(...data.subarray(start, start + 8192));
      clips.set(item.text, Object.freeze({ audio: btoa(binary), approved: item.review_status === 'approved_human' }));
    }
    return new ReceiptAudioCache(clips);
  }
}
