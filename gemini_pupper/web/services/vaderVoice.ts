/**
 * Darth Vader voice effect for Pupper's speech output.
 *
 * Every reply (native Live audio, cached receipts, TTS, the follow-handoff cue)
 * already goes through the mouth analyser. The effect sits between that
 * analyser and the speaker:
 *
 *   analyser (dry, drives the mouth) -> pitch shifter -> mask EQ -> saturation
 *     -> helmet reverb (dry + wet) -> compressor -> limiter -> speaker
 *
 * The voice model stays Puck: the cached receipts and the follow-handoff cue
 * are Puck-only, so a different voice would silence them. Only native Web Audio
 * nodes are used: no AudioWorklet module to load, so the effect builds
 * synchronously and also runs in an OfflineAudioContext.
 *
 * The respirator breath is synthesized here (no recorded audio). It enters at
 * the limiter: its level does not depend on the compressor, a cut leaves no
 * reverb tail, and it never moves the mouth.
 */

export const VADER = {
  /** Output pitch / input pitch: 0.67 is about -7 semitones (Puck median f0 ~190 Hz -> ~125 Hz). */
  pitchRatio: 0.67,
  /** Pitch-shifter delay window, s. Longer = smoother but more echo. */
  grainSeconds: 0.05,
  /** Gain before the EQ and saturation: headroom for the EQ boosts (Puck peaks reach 0.9). */
  preGain: 0.5,
  /** tanh drive of the saturation stage. */
  drive: 2,
  /** Helmet reverb level relative to the dry signal. */
  reverbWet: 0.3,
  /** Gain after the compressor: sets the speech level near the dry voice. */
  makeupGain: 1.0,
  /** Breath peak into the limiter (its fixed makeup adds ~1.7 dB): ~8 dB under speech. */
  breathPeak: 0.3,
  /** Fade time when new speech or Stop cuts a breath, s. */
  breathFadeSeconds: 0.03,
} as const;

/** Deterministic noise, so every breath and reverb is the same and tests can pin them. */
function noise(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return (((t ^ (t >>> 14)) >>> 0) / 4294967296) * 2 - 1;
  };
}

/** RBJ-cookbook band-pass (constant 0 dB peak gain), direct form I. */
function bandPass(sampleRate: number, frequency: number, q: number): (x: number) => number {
  const w = (2 * Math.PI * frequency) / sampleRate, alpha = Math.sin(w) / (2 * q), a0 = 1 + alpha;
  const b0 = alpha / a0, b2 = -alpha / a0, a1 = (-2 * Math.cos(w)) / a0, a2 = (1 - alpha) / a0;
  let x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  return x => {
    const y = b0 * x + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  };
}

/** 0 -> 1 -> 0 envelope with raised-cosine attack and release; t, length in s. */
function envelope(t: number, length: number, attack: number, release: number): number {
  if (t <= 0 || t >= length) return 0;
  if (t < attack) return 0.5 - 0.5 * Math.cos((Math.PI * t) / attack);
  if (t > length - release) return 0.5 - 0.5 * Math.cos((Math.PI * (length - t)) / release);
  return 1;
}

/** Breath timing, s: inhale, pause, exhale. */
export const BREATH_TIMING = { inhale: 1.2, pause: 0.25, exhale: 1.5 } as const;

/**
 * One respirator breath (inhale, pause, exhale) as mono samples.
 * Inhale: band-limited hiss with a narrow valve whistle and a fast flutter.
 * Exhale: lower, hollow hiss with a slower flutter. Peak = VADER.breathPeak.
 */
export function renderBreath(sampleRate: number, seed = 0x5eed): Float32Array {
  const { inhale, pause, exhale } = BREATH_TIMING;
  const out = new Float32Array(Math.round((inhale + pause + exhale) * sampleRate));
  const random = noise(seed);
  const inhaleBody = bandPass(sampleRate, 1500, 0.8), inhaleValve = bandPass(sampleRate, 2600, 7);
  const exhaleBody = bandPass(sampleRate, 650, 0.7), exhaleValve = bandPass(sampleRate, 1250, 5);
  const exhaleStart = inhale + pause;
  let peak = 0;
  for (let i = 0; i < out.length; i++) {
    const t = i / sampleRate, x = random();
    // Run both filter pairs on every sample, so each starts settled.
    const a = inhaleBody(x) + 0.5 * inhaleValve(x), b = exhaleBody(x) + 0.4 * exhaleValve(x);
    const inhaleLevel = envelope(t, inhale, 0.3, 0.35) * (1 + 0.25 * Math.sin(2 * Math.PI * 22 * t));
    const exhaleLevel = 0.85 * envelope(t - exhaleStart, exhale, 0.2, 0.7) * (1 + 0.15 * Math.sin(2 * Math.PI * 16 * t));
    const y = a * inhaleLevel + b * exhaleLevel;
    out[i] = y;
    peak = Math.max(peak, Math.abs(y));
  }
  const scale = peak > 0 ? VADER.breathPeak / peak : 0;
  for (let i = 0; i < out.length; i++) out[i] *= scale;
  return out;
}

/**
 * Control signals of the two-tap delay-line pitch shifter, one grain period long
 * (it loops): [delay ramp A, fade A, delay ramp B, fade B], each in [0, 1].
 * Tap B runs half a period behind tap A. A ramp rises from 0 to 1 over the period,
 * so its delay grows at (1 - ratio) s/s and the read point moves at `ratio`:
 * the pitch drops by `ratio`. Each fade is sin^2, zero where its ramp jumps back,
 * and the two fades always add to 1.
 */
export function pitchShifterControl(sampleRate: number, ratio: number, grainSeconds: number): Float32Array[] {
  if (!(ratio > 0 && ratio < 1) || !(grainSeconds > 0)) throw new Error('pitch ratio must be in (0, 1) and grain > 0');
  const length = Math.max(2, Math.round((grainSeconds / (1 - ratio)) * sampleRate));
  const channels = [0, 1, 2, 3].map(() => new Float32Array(length));
  for (let i = 0; i < length; i++) {
    const a = i / length, b = (a + 0.5) % 1;
    channels[0][i] = a;
    channels[1][i] = Math.sin(Math.PI * a) ** 2;
    channels[2][i] = b;
    channels[3][i] = Math.sin(Math.PI * b) ** 2;
  }
  return channels;
}

/** Short "inside a helmet" impulse response: early reflections plus a 60 ms noise tail, unit energy. */
export function helmetImpulse(sampleRate: number, seed = 0x4e1): Float32Array {
  const out = new Float32Array(Math.round(0.12 * sampleRate));
  const random = noise(seed);
  for (const [ms, gain] of [[2.9, 0.7], [5.3, -0.55], [8.1, 0.4], [11.7, -0.3]]) {
    out[Math.round((ms / 1000) * sampleRate)] += gain;
  }
  for (let i = 0; i < out.length; i++) out[i] += 0.35 * random() * Math.exp(-i / (0.06 * sampleRate));
  let energy = 0;
  for (const x of out) energy += x * x;
  const scale = 1 / Math.sqrt(energy);
  for (let i = 0; i < out.length; i++) out[i] *= scale;
  return out;
}

/** tanh saturation curve, normalized so that +-1 maps to +-1. */
export function saturationCurve(drive: number, points = 2049): Float32Array<ArrayBuffer> {
  const curve = new Float32Array(points);
  for (let i = 0; i < points; i++) {
    const x = (i / (points - 1)) * 2 - 1;
    curve[i] = Math.tanh(drive * x) / Math.tanh(drive);
  }
  return curve;
}

export interface VaderVoice {
  /** Connect the speech (the mouth analyser) here. */
  readonly input: AudioNode;
  /** Connect this to the speaker. */
  readonly output: AudioNode;
  /** Start one breath now. False if a breath is already playing. */
  breathe(): boolean;
  /** Fade out and stop the current breath, if any. */
  stopBreath(): void;
  /** Stop every source and disconnect the chain. */
  dispose(): void;
}

function biquad(ctx: BaseAudioContext, type: BiquadFilterType, frequency: number, q: number, gainDb = 0): BiquadFilterNode {
  const filter = ctx.createBiquadFilter();
  filter.type = type;
  filter.frequency.value = frequency;
  filter.Q.value = q;
  filter.gain.value = gainDb;
  return filter;
}

function monoBuffer(ctx: BaseAudioContext, samples: Float32Array): AudioBuffer {
  const buffer = ctx.createBuffer(1, samples.length, ctx.sampleRate);
  buffer.getChannelData(0).set(samples);
  return buffer;
}

/**
 * Build the effect on `ctx`. Throws if the context lacks a needed node type;
 * nothing plays until the caller connects `output`.
 */
export function createVaderVoice(ctx: BaseAudioContext): VaderVoice {
  const sampleRate = ctx.sampleRate;
  const input = ctx.createGain();

  // Pitch shifter: two delay taps, each delay driven by its ramp and gated by its fade.
  const control = pitchShifterControl(sampleRate, VADER.pitchRatio, VADER.grainSeconds);
  const controlBuffer = ctx.createBuffer(4, control[0].length, sampleRate);
  control.forEach((channel, index) => controlBuffer.getChannelData(index).set(channel));
  const controlSource = ctx.createBufferSource();
  controlSource.buffer = controlBuffer;
  controlSource.loop = true;
  const split = ctx.createChannelSplitter(4);
  controlSource.connect(split);
  const shifted = ctx.createGain();
  for (const tap of [0, 1]) {
    const delay = ctx.createDelay(VADER.grainSeconds + 0.01);
    delay.delayTime.value = 0;
    const depth = ctx.createGain();
    depth.gain.value = VADER.grainSeconds;
    split.connect(depth, 2 * tap);
    depth.connect(delay.delayTime);
    const fade = ctx.createGain();
    fade.gain.value = 0;
    split.connect(fade.gain, 2 * tap + 1);
    input.connect(delay);
    delay.connect(fade);
    fade.connect(shifted);
  }

  // Mask tone.
  const tone = ctx.createGain();
  tone.gain.value = VADER.preGain;
  shifted.connect(tone);
  const filters = [
    biquad(ctx, 'highpass', 70, 0.707),        // remove rumble below the new voice
    biquad(ctx, 'peaking', 160, 0.9, 3),       // chest
    biquad(ctx, 'peaking', 1000, 1.4, 4),      // boxy mask resonance
    biquad(ctx, 'lowpass', 3800, 0.8),         // muffled through the mask
  ];
  let last: AudioNode = tone;
  for (const filter of filters) { last.connect(filter); last = filter; }
  const shaper = ctx.createWaveShaper();
  shaper.curve = saturationCurve(VADER.drive);
  shaper.oversample = '4x';
  last.connect(shaper);

  // Helmet: dry plus a short, unnormalized reverb.
  const reverb = ctx.createConvolver();
  reverb.normalize = false;
  reverb.buffer = monoBuffer(ctx, helmetImpulse(sampleRate));
  const wet = ctx.createGain();
  wet.gain.value = VADER.reverbWet;
  const mix = ctx.createGain();
  shaper.connect(mix);
  shaper.connect(reverb);
  reverb.connect(wet);
  wet.connect(mix);

  const compressor = ctx.createDynamicsCompressor();
  compressor.threshold.value = -24;
  compressor.knee.value = 6;
  compressor.ratio.value = 4;
  compressor.attack.value = 0.004;
  compressor.release.value = 0.2;
  const makeup = ctx.createGain();
  makeup.gain.value = VADER.makeupGain;
  const limiter = ctx.createDynamicsCompressor();
  limiter.threshold.value = -3;
  limiter.knee.value = 0;
  limiter.ratio.value = 20;
  limiter.attack.value = 0.001;
  limiter.release.value = 0.05;
  mix.connect(compressor);
  compressor.connect(makeup);
  makeup.connect(limiter);
  const output = limiter;

  const breathBuffer = monoBuffer(ctx, renderBreath(sampleRate));
  let breath: { source: AudioBufferSourceNode; level: GainNode } | null = null;
  const endBreath = (current: { source: AudioBufferSourceNode; level: GainNode }) => {
    if (breath === current) breath = null;
    try { current.source.disconnect(); current.level.disconnect(); } catch { /* Already detached. */ }
  };

  controlSource.start();

  return {
    input,
    output,
    breathe() {
      if (breath) return false;
      const source = ctx.createBufferSource();
      source.buffer = breathBuffer;
      const level = ctx.createGain();
      source.connect(level);
      level.connect(limiter);
      const current = { source, level };
      breath = current;
      source.onended = () => endBreath(current);
      source.start();
      return true;
    },
    stopBreath() {
      const current = breath;
      if (!current) return;
      breath = null;
      const now = ctx.currentTime;
      try {
        current.level.gain.cancelScheduledValues(now);
        current.level.gain.setValueAtTime(current.level.gain.value, now);
        current.level.gain.linearRampToValueAtTime(0, now + VADER.breathFadeSeconds);
        current.source.stop(now + VADER.breathFadeSeconds);
      } catch { endBreath(current); }
    },
    dispose() {
      const current = breath;
      breath = null;
      if (current) { try { current.source.stop(); } catch { /* Ended. */ } endBreath(current); }
      try { controlSource.stop(); } catch { /* Ended. */ }
      try { input.disconnect(); output.disconnect(); } catch { /* Detached. */ }
    },
  };
}
