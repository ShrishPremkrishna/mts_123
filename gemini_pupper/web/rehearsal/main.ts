import { GeminiLiveService, type LiveTimingEvent } from '../services/geminiLive';
import { DEFAULT_CONFIG } from '../constants';
import { assertFake, fakeOnlyFetch, loopbackBase } from './fakeOnly';
import { mock } from './mockGoogle';

const output = document.querySelector<HTMLPreElement>('#state')!;
const events = document.querySelector<HTMLPreElement>('#events')!;
const run = document.querySelector<HTMLButtonElement>('#run')!;
const rawFetch = window.fetch.bind(window);
const base = loopbackBase(new URLSearchParams(location.search).get('api') || 'http://127.0.0.1:18091');
window.fetch = fakeOnlyFetch(base, rawFetch);
let service: GeminiLiveService | null = null;
let sourceContext: AudioContext | null = null;
let syntheticStream: MediaStream | null = null;
let syntheticVideo: MediaStream | null = null;
let releaseVideoRead: (() => void) | null = null;
let restoreVideoRead: (() => void) | null = null;
let releaseDecode: (() => void) | null = null;
let pendingStops = 0;
let finished = false;
const timeline: LiveTimingEvent[] = [];
const dispatches: unknown[] = [];
const receipt: Record<string, any> = { api: base, mode: 'synthetic-media-mock-Google-fake-robot',
  hardware_connected: false, real_microphone: false, real_camera: false, cloud_connection: false, checks: {} };
function show() { output.textContent = JSON.stringify(receipt, null, 2); }
function check(name: string, condition: boolean) {
  receipt.checks[name] = condition; show();
  if (!condition) throw new Error(`Check failed: ${name}`);
}
function trace(event: LiveTimingEvent) {
  if (timeline.length < 1500) timeline.push(event);
  events.textContent = JSON.stringify(timeline.slice(-8), null, 2);
}
async function until(predicate: () => boolean, label: string, timeout = 4000) {
  const deadline = performance.now() + timeout;
  while (!predicate()) {
    if (performance.now() > deadline) throw new Error(`Timed out: ${label}`);
    await new Promise(resolve => setTimeout(resolve, 20));
  }
}
// Do not retain or fall back to the real method, even if synthetic setup fails.
Object.defineProperty(navigator.mediaDevices, 'getUserMedia', { configurable: true, value: async (constraints: MediaStreamConstraints) => {
  if (constraints.audio && !constraints.video && syntheticStream) return syntheticStream;
  if (constraints.video && !constraints.audio && syntheticVideo) return syntheticVideo;
  throw new Error('Real microphone/camera access is prohibited.');
} });
function silenceWav(): string {
  const count = 960, buffer = new ArrayBuffer(44 + count * 2), view = new DataView(buffer);
  const word = (offset: number, text: string) => [...text].forEach((c, i) => view.setUint8(offset + i, c.charCodeAt(0)));
  word(0, 'RIFF'); view.setUint32(4, 36 + count * 2, true); word(8, 'WAVE'); word(12, 'fmt ');
  view.setUint32(16, 16, true); view.setUint16(20, 1, true); view.setUint16(22, 1, true);
  view.setUint32(24, 24000, true); view.setUint32(28, 48000, true); view.setUint16(32, 2, true);
  view.setUint16(34, 16, true); word(36, 'data'); view.setUint32(40, count * 2, true);
  return btoa(String.fromCharCode(...new Uint8Array(buffer)));
}
async function decodeJpeg(data: string) {
  const bytes = Uint8Array.from(atob(data), c => c.charCodeAt(0));
  const bitmap = await createImageBitmap(new Blob([bytes], { type: 'image/jpeg' }));
  const canvas = document.createElement('canvas'); canvas.width = bitmap.width; canvas.height = bitmap.height;
  const ctx = canvas.getContext('2d')!; ctx.drawImage(bitmap, 0, 0);
  const pixel = [...ctx.getImageData(Math.floor(canvas.width / 2), Math.floor(canvas.height / 2), 1, 1).data];
  const result = { width: bitmap.width, height: bitmap.height, bytes: bytes.length, center_rgba: pixel };
  bitmap.close(); return result;
}
async function cleanup(stop: boolean) {
  releaseDecode?.(); releaseDecode = null;
  if (stop && service) receipt.cleanup_stop = await service.robotBridge.immediateStop();
  await service?.disconnect();
  syntheticStream?.getTracks().forEach(track => track.stop());
  syntheticVideo?.getTracks().forEach(track => track.stop());
  restoreVideoRead?.(); restoreVideoRead = null;
  releaseVideoRead?.(); releaseVideoRead = null;
  if (sourceContext && sourceContext.state !== 'closed') await sourceContext.close();
}
run.onclick = async () => {
  if (finished) return;
  run.disabled = true; receipt.started_at = new Date().toISOString();
  try {
    // Create/resume the synthetic source directly in the user gesture.
    sourceContext = new AudioContext({ sampleRate: 16000 });
    await sourceContext.resume();
    const destination = sourceContext.createMediaStreamDestination();
    const oscillator = sourceContext.createOscillator(), gain = sourceContext.createGain();
    oscillator.frequency.value = 440; gain.gain.value = 0.03;
    oscillator.connect(gain).connect(destination); oscillator.start();
    syntheticStream = destination.stream;
    const canvas = document.createElement('canvas'); canvas.width = 640; canvas.height = 480;
    const camera = canvas.getContext('2d')!;
    const paint = (color: string) => { camera.fillStyle = color; camera.fillRect(0, 0, canvas.width, canvas.height); };
    paint('rgb(230,25,25)'); syntheticVideo = canvas.captureStream(5);
    const video = document.createElement('video'); video.muted = true; video.playsInline = true;
    await assertFake(base, rawFetch); check('initialized_fake_api', true);
    const config = { ...DEFAULT_CONFIG, robotApiUrl: base, enableVideo: true, enableAudio: true, cameraSource: 'browser' as const,
      enableAffectiveDialog: false, enableProactiveAudio: false };
    service = new GeminiLiveService(config, () => {}, () => {}, () => {}, entry => dispatches.push(entry),
      () => {}, pending => { pendingStops += pending ? 1 : -1; }, trace);
    await service.connect('REHEARSAL-NOT-A-KEY', config, video);
    await until(() => mock.audioFrames >= 2 && mock.nonzeroFrames >= 1, 'synthetic audio capture');
    check('real_webaudio_capture_encoded_nonzero_pcm', true);
    await until(() => mock.videoFrames.length > 0, 'synthetic canvas video frame');
    const firstFrame = mock.videoFrames[0], firstImage = await decodeJpeg(firstFrame.data);
    receipt.first_video_frame = firstImage;
    check('synthetic_browser_video_is_decodable_red_jpeg', firstFrame.mimeType === 'image/jpeg'
      && firstImage.width === 320 && firstImage.height === 240 && firstImage.bytes > 100
      && firstImage.center_rgba[0] > 180 && firstImage.center_rgba[2] < 60);
    const beforeChange = mock.videoFrames.length; paint('rgb(25,65,230)');
    await until(() => mock.videoFrames.length > beforeChange, 'changed synthetic canvas video');
    let changedImage = await decodeJpeg(mock.videoFrames.at(-1)!.data);
    const changedDeadline = performance.now() + 4000;
    while (changedImage.center_rgba[2] < 180 && performance.now() < changedDeadline) {
      const count = mock.videoFrames.length;
      await until(() => mock.videoFrames.length > count, 'updated blue canvas frame');
      changedImage = await decodeJpeg(mock.videoFrames.at(-1)!.data);
    }
    receipt.changed_video_frame = changedImage;
    check('changed_canvas_frame_reaches_production_video_path', changedImage.center_rgba[2] > 180
      && changedImage.center_rgba[0] < 60 && mock.videoFrames.at(-1)!.data !== firstFrame.data);
    await mock.emit({ toolCall: { functionCalls: [{ id: 'rehearsal-wait', name: 'wait', args: { duration: 60 } }] } });
    check('fake_wait_accepted', mock.responses.at(-1)?.functionResponses?.response?.result?.success === true);
    await until(() => dispatches.some((d: any) => d.id === 'rehearsal-wait' && d.status === 'accepted'), 'wait receipt');
    let before = await service.robotBridge.getQueueStatus();
    check('fake_wait_pending_or_running', before.running === true || before.queued > 0);

    const originalDecode = AudioContext.prototype.decodeAudioData;
    let decodeEntered = false, nativeDecode: { sample_rate: number; frames: number } | null = null, release!: () => void;
    const gate = new Promise<void>(resolve => { release = resolve; });
    releaseDecode = release;
    (AudioContext.prototype as any).decodeAudioData = function (bytes: ArrayBuffer) {
      decodeEntered = true;
      return gate.then(() => originalDecode.call(this, bytes)).then(buffer => {
        nativeDecode = { sample_rate: buffer.sampleRate, frames: buffer.length };
        return buffer;
      });
    };
    let audio: Promise<void> | null = null;
    try {
      audio = mock.emit({ serverContent: { modelTurn: { parts: [{ inlineData: { mimeType: 'audio/wav', data: silenceWav() } }] } } });
      await until(() => decodeEntered, 'held native WAV decode');
      const start = performance.now();
      await mock.emit({ toolCall: { functionCalls: [{ id: 'rehearsal-stop', name: 'stop', args: {} }] } });
      receipt.stop_callback_to_reply_ms = performance.now() - start;
      const stopReply = mock.responses.at(-1)?.functionResponses;
      check('stop_reply_before_audio_decode_release', stopReply?.id === 'rehearsal-stop' && stopReply.name === 'stop'
        && stopReply.response?.result?.success === true && nativeDecode === null);
      const status = await service.robotBridge.getQueueStatus();
      receipt.after_stop = status;
      check('fake_queue_cancelled', status.running === false && status.queued === 0 && status.last_outcome?.message === 'cancelled');
      check('typed_stop_lifecycle_balanced', pendingStops === 0);
    } finally {
      release(); releaseDecode = null;
      AudioContext.prototype.decodeAudioData = originalDecode;
      if (audio) await audio;
    }
    receipt.native_wav_decode = nativeDecode;
    check('real_native_wav_decode_completed', nativeDecode !== null && nativeDecode.sample_rate === 24000
      && nativeDecode.frames === 960 && timeline.some(e => e.phase === 'output_audio_handle_return'));
    const originalRead = FileReader.prototype.readAsDataURL;
    let heldRead = false, heldReadCompleted = false;
    restoreVideoRead = () => { FileReader.prototype.readAsDataURL = originalRead; };
    FileReader.prototype.readAsDataURL = function (blob: Blob) {
      if (blob.type === 'image/jpeg' && !heldRead) {
        heldRead = true;
        this.addEventListener('loadend', () => { heldReadCompleted = true; }, { once: true });
        releaseVideoRead = () => originalRead.call(this, blob);
        return;
      }
      originalRead.call(this, blob);
    };
    await until(() => heldRead, 'late video conversion held before disconnect');
    const videoFramesBeforeClose = mock.videoFrames.length;
    await cleanup(false);
    await until(() => heldReadCompleted, 'held video conversion actually finished after disconnect');
    const frames = mock.audioFrames;
    await new Promise(resolve => setTimeout(resolve, 1100));
    check('video_stops_and_late_conversion_never_sends_after_disconnect', mock.videoFrames.length === videoFramesBeforeClose);
    check('synthetic_video_tracks_ended', syntheticVideo!.getTracks().every(track => track.readyState === 'ended'));
    check('synthetic_capture_stopped_after_disconnect', mock.audioFrames === frames);
    check('owned_tracks_ended', syntheticStream!.getTracks().every(track => track.readyState === 'ended'));
    check('mock_sdk_closed', mock.closed === 1);
    receipt.result = 'PASS';
  } catch (error) {
    receipt.result = 'FAIL'; receipt.error = String(error);
    await cleanup(true).catch(error => { receipt.cleanup_error = String(error); });
  } finally {
    finished = true; receipt.finished_at = new Date().toISOString();
    receipt.synthetic_video_frames = mock.videoFrames.length;
    receipt.synthetic_audio = { frames: mock.audioFrames, nonzero_frames: mock.nonzeroFrames, bytes: mock.audioBytes };
    receipt.timeline = timeline; receipt.dispatches = dispatches; show();
  }
};
document.querySelector<HTMLButtonElement>('#stop')!.onclick = async () => {
  service?.pauseRobotTools(); await cleanup(true); receipt.operator_cleanup = true; show();
};
document.querySelector<HTMLButtonElement>('#download')!.onclick = () => {
  const link = document.createElement('a'); link.href = URL.createObjectURL(new Blob([JSON.stringify(receipt, null, 2)], { type: 'application/json' }));
  link.download = 'pupper-browser-media-rehearsal.json'; link.click(); setTimeout(() => URL.revokeObjectURL(link.href), 1000);
};
