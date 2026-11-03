import type { AppConfig } from '../types';
import { MODEL_OPTIONS, TTS_MODEL_OPTIONS, VOICE_OPTIONS } from '../constants';

/** One explicit kiosk launch; not a saved credential or reconnect mechanism. */
export function takeOnboardBootstrap(): { present: boolean; nonce: string | null } {
  if (typeof window === 'undefined') return { present: false, nonce: null };
  const location = window.location;
  const fragment = new URLSearchParams((location.hash || '').replace(/^#/, ''));
  if (!fragment.has('onboard')) return { present: false, nonce: null };
  const values = fragment.getAll('onboard');
  // Strip before fetch/media and regardless of validity. Do not put the token
  // into a query, saved state, console or error message.
  try { window.history.replaceState(null, '', location.pathname + location.search); }
  catch { return { present: true, nonce: null }; }
  const local = ['127.0.0.1', 'localhost', '[::1]'].includes(location.hostname);
  const nonce = values.length === 1 && /^[A-Za-z0-9_-]{32,128}$/.test(values[0]) ? values[0] : null;
  return { present: true, nonce: local ? nonce : null };
}

export interface OnboardBootstrap {
  schema: 'pupper_onboard_bootstrap_v1';
  apiKey: string;
  autoConnect: true;
  config: Pick<AppConfig, 'voiceInputMode' | 'robotApiUrl' | 'cameraSource' | 'eyeTrackingSource'
    | 'enableAudio' | 'enableVideo' | 'modelName' | 'ttsModelName' | 'voiceName' | 'outputPolicy' | 'quickReceipts'>;
}

export async function consumeOnboardBootstrap(nonce: string, signal: AbortSignal): Promise<OnboardBootstrap> {
  if (!/^[A-Za-z0-9_-]{32,128}$/.test(nonce)) throw new Error('Invalid onboard setup');
  const controller = new AbortController();
  const cancel = () => controller.abort();
  signal.addEventListener('abort', cancel, { once: true });
  if (signal.aborted) cancel();
  const timer = setTimeout(cancel, 10_000);
  let value: any;
  try {
    const response = await fetch('/onboard/bootstrap', {
      method: 'POST', credentials: 'same-origin', cache: 'no-store', signal: controller.signal,
      headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ nonce }),
    });
    if (!response.ok) throw new Error('Onboard setup unavailable');
    value = await response.json();
    if (controller.signal.aborted) throw new Error('Onboard setup cancelled');
  } finally {
    clearTimeout(timer);
    signal.removeEventListener('abort', cancel);
  }
  const c = value?.config;
  if (value?.schema !== 'pupper_onboard_bootstrap_v1' || value.autoConnect !== true
      || typeof value.apiKey !== 'string' || !value.apiKey.trim() || value.apiKey.length > 4096
      || !c || c.voiceInputMode !== 'hands_free' || c.robotApiUrl !== ''
      || c.cameraSource !== 'server' || c.eyeTrackingSource !== 'server'
      || c.enableAudio !== true || c.enableVideo !== true || c.outputPolicy !== 'conversation'
      || c.quickReceipts !== true || !MODEL_OPTIONS.includes(c.modelName)
      || !TTS_MODEL_OPTIONS.includes(c.ttsModelName) || !VOICE_OPTIONS.includes(c.voiceName)) {
    throw new Error('Invalid onboard setup');
  }
  // Whitelist only the agreed fields. Never merge arbitrary provider/server
  // properties (including an API key or prompt) into persisted configuration.
  return { schema: 'pupper_onboard_bootstrap_v1', apiKey: value.apiKey, autoConnect: true,
    config: { voiceInputMode: 'hands_free', robotApiUrl: '', cameraSource: 'server',
      eyeTrackingSource: 'server', enableAudio: true, enableVideo: true,
      modelName: c.modelName, ttsModelName: c.ttsModelName, voiceName: c.voiceName,
      outputPolicy: 'conversation', quickReceipts: true } };
}
