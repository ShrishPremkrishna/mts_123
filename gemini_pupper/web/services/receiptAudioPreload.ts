import approvedManifest from './receiptAudioManifest.json';
import { ReceiptAudioCache } from './receiptAudioCache';
import type { AppConfig } from '../types';

export const RECEIPT_PRELOAD_TIMEOUT_MS = 2500;
export function supportsQuickReceipts(config: Pick<AppConfig, 'modelName' | 'voiceName' | 'outputPolicy' | 'quickReceipts'>): boolean {
  return config.quickReceipts === true && config.outputPolicy === 'conversation'
    && config.modelName.replace(/^models\//, '') === approvedManifest.model && config.voiceName === approvedManifest.voice;
}

/** Bundled hashes pin the exact human-approved clips; only PCM is fetched.
 * A single bounded connect-time attempt, never an action-dispatch dependency. */
export async function preloadReceiptAudio(config: Pick<AppConfig, 'modelName' | 'voiceName' | 'outputPolicy' | 'quickReceipts'>,
    signal?: AbortSignal, fetchAsset: typeof fetch = fetch): Promise<ReceiptAudioCache | null> {
  if (!supportsQuickReceipts(config) || signal?.aborted) return null;
  const controller = new AbortController();
  let timer: ReturnType<typeof setTimeout> | undefined;
  let rejectAbort: (() => void) | undefined;
  const cancelled = new Promise<never>((_, reject) => { rejectAbort = () => reject(new Error('Receipt audio preload cancelled')); });
  const cancel = () => { controller.abort(); rejectAbort?.(); };
  signal?.addEventListener('abort', cancel, { once: true });
  timer = setTimeout(cancel, RECEIPT_PRELOAD_TIMEOUT_MS);
  try {
    const load = ReceiptAudioCache.load(approvedManifest, async file => {
      const expected = approvedManifest.clips.find(clip => clip.file === file)!;
      const response = await fetchAsset(`/receipt-audio/${encodeURIComponent(file)}`, { signal: controller.signal, cache: 'force-cache' });
      if (!response.ok || !response.body) throw new Error('Receipt audio unavailable');
      const length = response.headers.get('content-length');
      if (length !== null && Number(length) !== expected.bytes) throw new Error('Receipt audio length mismatch');
      const reader = response.body.getReader(), bytes = new Uint8Array(expected.bytes);
      let offset = 0;
      try {
        while (true) {
          if (controller.signal.aborted) throw new Error('Receipt audio preload cancelled');
          const part = await reader.read();
          if (part.done) break;
          if (offset + part.value.byteLength > bytes.byteLength) throw new Error('Receipt audio exceeds pinned size');
          bytes.set(part.value, offset); offset += part.value.byteLength;
        }
      } catch (error) {
        void reader.cancel().catch(() => {});
        throw error;
      } finally { reader.releaseLock(); }
      if (offset !== expected.bytes) throw new Error('Receipt audio truncated');
      return bytes;
    });
    return await Promise.race([load, cancelled]);
  } catch { return null; }
  finally {
    clearTimeout(timer); signal?.removeEventListener('abort', cancel); controller.abort();
  }
}
