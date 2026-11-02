import { normalizeReceiptTranscript } from './conversationOutput';

const MODEL = 'gemini-3.1-flash-live-preview';
const VOICE = 'Puck';
const PHRASES = new Set(['Move queued.', 'Stop requested.', 'Left paw command queued.', 'Right paw command queued.']);
const MAX_CLIP_BYTES = 24000 * 2 * 10;

interface ReadyClip { audio: string; approved: boolean; }

/** Explicitly supplied, preloaded audio. No fetch or decoding runs on tool dispatch. */
export class ReceiptAudioCache {
  private constructor(private readonly clips: ReadonlyMap<string, ReadyClip>) {}

  lookup(text: string, model: string, voice: string): string | null {
    if (model.replace(/^models\//, '') !== MODEL || voice !== VOICE) return null;
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
