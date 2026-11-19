import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';

// The DSP helpers, and the service wiring on a recording fake AudioContext.
// The sound itself (pitch, level, breath) is checked by rendering in Chrome; see vaderVoice.ts.
const built = await build({ stdin: { contents: "export * from './services/vaderVoice'; export { GeminiLiveService } from './services/geminiLive'; export { DEFAULT_CONFIG } from './constants';", resolveDir: process.cwd(), loader: 'ts' },
  bundle: true, write: false, format: 'esm', platform: 'node',
  plugins: [{ name: 'offline-sdk', setup(b) {
    b.onResolve({ filter: /^@google\/genai$/ }, () => ({ path: 'sdk', namespace: 'stub' }));
    b.onLoad({ filter: /.*/, namespace: 'stub' }, () => ({ contents: "export class GoogleGenAI { constructor(){this.live={connect:x=>globalThis.__vaderSdkConnect(x)}} } export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};" }));
  } }] });
const { VADER, BREATH_TIMING, renderBreath, pitchShifterControl, helmetImpulse, saturationCurve, createVaderVoice, GeminiLiveService, DEFAULT_CONFIG } =
  await import('data:text/javascript;base64,' + Buffer.from(built.outputFiles[0].text).toString('base64'));
const SR = 24000;

test('pitch-shifter control: ramps loop in [0, 1), fades add to 1 and hide each ramp jump', () => {
  const [rampA, fadeA, rampB, fadeB] = pitchShifterControl(SR, VADER.pitchRatio, VADER.grainSeconds);
  const n = rampA.length;
  assert.equal(n, Math.round(VADER.grainSeconds / (1 - VADER.pitchRatio) * SR));
  for (let i = 0; i < n; i++) {
    assert.ok(rampA[i] >= 0 && rampA[i] < 1 && rampB[i] >= 0 && rampB[i] < 1);
    assert.ok(Math.abs(fadeA[i] + fadeB[i] - 1) < 1e-6, `fades at ${i}`);
  }
  // Delay grows by grain/n per sample (one wrap per ramp): the read point moves at the pitch ratio.
  for (const ramp of [rampA, rampB]) {
    let wraps = 0;
    for (let i = 1; i < n; i++) if (Math.abs(ramp[i] - ramp[i - 1] - 1 / n) > 1e-6) wraps++;
    assert.ok(wraps <= 1);
  }
  assert.ok(Math.abs(1 - VADER.grainSeconds * SR / n - VADER.pitchRatio) < 1e-3);
  // Each ramp jumps back where its fade is (near) zero.
  assert.equal(fadeA[0], 0);
  const jumpB = rampB.findIndex((x, i) => i > 0 && x < rampB[i - 1]);
  assert.ok(fadeB[jumpB] < 1e-6 && fadeA[n - 1] < 1e-5);
  assert.throws(() => pitchShifterControl(SR, 1, 0.05));
  assert.throws(() => pitchShifterControl(SR, 0.5, 0));
});

test('breath: deterministic, finite, pinned peak, silent ends and silent pause', () => {
  const a = renderBreath(SR), b = renderBreath(SR);
  const { inhale, pause, exhale } = BREATH_TIMING;
  assert.equal(a.length, Math.round((inhale + pause + exhale) * SR));
  assert.deepEqual(a, b);
  let peak = 0;
  for (const x of a) { assert.ok(Number.isFinite(x)); peak = Math.max(peak, Math.abs(x)); }
  assert.ok(Math.abs(peak - VADER.breathPeak) < 1e-6);
  assert.equal(a[0], 0);
  assert.ok(Math.abs(a.at(-1)) < 1e-3);
  const gap = a.subarray(Math.ceil(inhale * SR) + 1, Math.floor((inhale + pause) * SR));
  assert.ok(gap.length > 0 && gap.every(x => x === 0));
  const rms = x => Math.sqrt(x.reduce((s, v) => s + v * v, 0) / x.length);
  assert.ok(rms(a.subarray(0, inhale * SR)) > VADER.breathPeak / 8 && rms(a.subarray((inhale + pause) * SR)) > VADER.breathPeak / 8);
});

test('helmet impulse has unit energy; saturation is odd, monotonic and keeps +-1', () => {
  const ir = helmetImpulse(SR);
  assert.ok(Math.abs(ir.reduce((s, v) => s + v * v, 0) - 1) < 1e-4 && ir.every(Number.isFinite));
  const curve = saturationCurve(VADER.drive);
  assert.ok(Math.abs(curve[0] + 1) < 1e-6 && Math.abs(curve.at(-1) - 1) < 1e-6 && curve[(curve.length - 1) / 2] === 0);
  for (let i = 1; i < curve.length; i++) assert.ok(curve[i] > curve[i - 1]);
});

// Records every node and connection. Optional `missing` deletes a factory to model an old browser.
function fakeContext(missing) {
  const nodes = [];
  const param = value => ({ value, events: [], setValueAtTime(v, t) { this.events.push(['set', v, t]); },
    linearRampToValueAtTime(v, t) { this.events.push(['ramp', v, t]); }, cancelScheduledValues(t) { this.events.push(['cancel', t]); } });
  const node = (kind, extra = {}) => { const n = { kind, out: [], connect(to, output) { this.out.push({ to, output }); return to; }, disconnect() { this.disconnected = true; }, ...extra }; nodes.push(n); return n; };
  const ctx = {
    nodes, sampleRate: SR, currentTime: 0, state: 'running',
    destination: node('destination'),
    createGain: () => node('gain', { gain: param(1) }),
    createDelay: max => node('delay', { max, delayTime: param(0) }),
    createBiquadFilter: () => node('biquad', { frequency: param(350), Q: param(1), gain: param(0) }),
    createWaveShaper: () => node('shaper', { curve: null, oversample: 'none' }),
    createConvolver: () => node('convolver', { normalize: true, buffer: null }),
    createDynamicsCompressor: () => node('compressor', Object.fromEntries(['threshold', 'knee', 'ratio', 'attack', 'release'].map(k => [k, param(0)]))),
    createChannelSplitter: n => node('splitter', { n }),
    createAnalyser: () => node('analyser', { getByteTimeDomainData(x) { x.fill(128); } }),
    createMediaStreamSource: () => node('stream'), createScriptProcessor: () => node('script'),
    createBuffer: (channels, length, sampleRate) => { const data = Array.from({ length: channels }, () => new Float32Array(length));
      return { numberOfChannels: channels, length, sampleRate, duration: length / sampleRate, getChannelData: i => data[i] }; },
    createBufferSource: () => node('source', { buffer: null, loop: false, onended: null, start(t = 0) { this.started = t; }, stop(t = 0) { this.stopped = t; } }),
    async resume() {}, async close() { this.state = 'closed'; },
  };
  if (missing) delete ctx[missing];
  return ctx;
}
const reaches = (from, to, seen = new Set()) => from === to || (!seen.has(from) && (seen.add(from), from.out.some(e => reaches(e.to, to, seen))));

test('effect graph: input reaches output through the delays; breath plays once, fades, and stops', () => {
  const ctx = fakeContext();
  const voice = createVaderVoice(ctx);
  assert.ok(reaches(voice.input, voice.output));
  const delays = ctx.nodes.filter(n => n.kind === 'delay');
  assert.equal(delays.length, 2);
  assert.ok(delays.every(d => d.max > VADER.grainSeconds && voice.input.out.some(e => e.to === d)));
  const control = ctx.nodes.find(n => n.kind === 'source');
  assert.ok(control.loop && control.started === 0 && control.buffer.numberOfChannels === 4);
  assert.equal(voice.breathe(), true);
  assert.equal(voice.breathe(), false, 'one breath at a time');
  const breath = ctx.nodes.filter(n => n.kind === 'source').at(-1);
  assert.ok(reaches(breath, voice.output) && breath.started === 0);
  ctx.currentTime = 1;
  voice.stopBreath();
  assert.equal(breath.stopped, 1 + VADER.breathFadeSeconds);
  assert.equal(voice.breathe(), true, 'a new breath may start after a stop');
  voice.dispose();
  assert.equal(control.stopped, 0);
  assert.ok(voice.input.disconnected && voice.output.disconnected);
});

function replace(t, key, value) { const old = Object.getOwnPropertyDescriptor(globalThis, key); Object.defineProperty(globalThis, key, { configurable: true, writable: true, value }); t.after(() => old ? Object.defineProperty(globalThis, key, old) : delete globalThis[key]); }
async function service(t, patch = {}, missing) {
  const contexts = [], logs = [], setups = [];
  replace(t, 'window', { AudioContext: class { constructor() { const c = fakeContext(contexts.length ? undefined : missing); contexts.push(c); return c; } }, setInterval });
  replace(t, '__vaderSdkConnect', async setup => { setups.push(setup); setup.callbacks.onopen(); return { close() {}, sendRealtimeInput() {}, sendToolResponse() {} }; });
  const config = { ...DEFAULT_CONFIG, quickReceipts: false, enableAudio: false, enableVideo: false, outputPolicy: 'conversation', robotApiUrl: 'http://vader-tests', ...patch };
  const s = new GeminiLiveService(config, x => logs.push(x), () => {}, () => {});
  s.robotBridge.getTools = async () => ({ prompt: 'Test prompt', tools: [] });
  t.after(() => s.disconnect());
  await s.connect('UNIT_TEST', config, {});
  return { s, ctx: contexts[0], logs, setup: setups[0] };
}
const DELIVERY = /like Darth Vader/;

test('Vader is the default: analyser -> effect -> speaker, delivery hint added, Puck kept', async t => {
  const { ctx, logs, setup, s } = await service(t);
  assert.equal(DEFAULT_CONFIG.voiceEffect, 'vader');
  assert.equal(setup.config.speechConfig.voiceConfig.prebuiltVoiceConfig.voiceName, 'Puck');
  const analyser = ctx.nodes.find(n => n.kind === 'analyser');
  assert.ok(!analyser.out.some(e => e.to === ctx.destination), 'no dry path to the speaker');
  assert.ok(reaches(analyser, ctx.destination) && ctx.nodes.some(n => n.kind === 'delay'));
  assert.match(setup.config.systemInstruction, DELIVERY);
  assert.ok(logs.some(x => x.message === 'Vader voice on.'));
  await s.disconnect();
  assert.ok(ctx.nodes.find(n => n.kind === 'source' && n.loop).stopped !== undefined, 'disconnect stops the effect');
});

test('none, absent, receipt-only and a failed build all keep the dry voice and the plain prompt', async t => {
  for (const [patch, missing] of [[{ voiceEffect: 'none' }], [{ voiceEffect: undefined }], [{ outputPolicy: 'receipts' }], [{}, 'createDelay']]) {
    const { ctx, logs, setup } = await service(t, patch, missing);
    const analyser = ctx.nodes.find(n => n.kind === 'analyser');
    assert.deepEqual(analyser.out.map(e => e.to), [ctx.destination], JSON.stringify(patch));
    assert.doesNotMatch(setup.config.systemInstruction, DELIVERY);
    assert.equal(logs.some(x => /Vader voice unavailable/.test(x.message)), !!missing);
  }
});

// Plays one real chunk through playAudioBase64 and returns its source node.
async function speak(e) {
  await e.s.playAudioBase64(Buffer.from(new Int16Array(240).buffer).toString('base64'));
  return e.ctx.nodes.filter(n => n.kind === 'source' && !n.loop).at(-1);
}
const breaths = e => e.ctx.nodes.filter(n => n.kind === 'source' && n.buffer?.length === renderBreath(SR).length).length;

test('one breath after a reply ends; never after a stop, during new speech, while holding talk, or when off', async t => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  const e = await service(t);
  let chunk = await speak(e);
  chunk.onended();
  t.mock.timers.tick(399); assert.equal(breaths(e), 0);
  t.mock.timers.tick(1); assert.equal(breaths(e), 1, 'breath 400 ms after the last chunk');

  chunk = await speak(e); chunk.onended(); t.mock.timers.tick(400);
  assert.equal(breaths(e), 1, 'cooldown');
  e.ctx.currentTime = 9;

  chunk = await speak(e); e.s.stopPlayback(); chunk.onended(); t.mock.timers.tick(1000);
  assert.equal(breaths(e), 1, 'interrupted or stopped speech does not breathe');

  chunk = await speak(e); chunk.onended(); t.mock.timers.tick(200); await speak(e); t.mock.timers.tick(1000);
  assert.equal(breaths(e), 1, 'new speech cancels the pending breath');
  e.s.stopPlayback();

  e.s.audioInputActive = true;
  chunk = await speak(e); chunk.onended(); t.mock.timers.tick(1000);
  assert.equal(breaths(e), 1, 'no breath while the talk button is held');
  e.s.audioInputActive = false;

  chunk = await speak(e); chunk.onended(); t.mock.timers.tick(1000);
  assert.equal(breaths(e), 2);
  let breath = e.ctx.nodes.filter(n => n.kind === 'source').at(-1);
  await speak(e);
  assert.notEqual(breath.stopped, undefined, 'new speech fades out a playing breath');
  e.s.stopPlayback();

  e.ctx.currentTime = 30;
  chunk = await speak(e); chunk.onended(); t.mock.timers.tick(200); e.s.stopPlayback(); t.mock.timers.tick(1000);
  assert.equal(breaths(e), 2, 'Stop cancels a pending breath');
  chunk = await speak(e); chunk.onended(); t.mock.timers.tick(400);
  assert.equal(breaths(e), 3);
  breath = e.ctx.nodes.filter(n => n.kind === 'source').at(-1);
  e.s.stopPlayback();
  assert.notEqual(breath.stopped, undefined, 'Stop fades out a playing breath');

  const off = await service(t, { vaderBreathing: false });
  chunk = await speak(off); chunk.onended(); t.mock.timers.tick(1000);
  assert.equal(breaths(off), 0);
});
