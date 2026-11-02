import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';

const built = await build({ entryPoints: ['services/geminiLive.ts'], bundle: true, write: false, format: 'esm', platform: 'node',
  plugins: [{ name: 'no-network-sdk', setup(b) {
    b.onResolve({ filter: /^@google\/genai$/ }, () => ({ path: 'sdk', namespace: 'stub' }));
    b.onLoad({ filter: /.*/, namespace: 'stub' }, () => ({ contents: "export class GoogleGenAI { constructor() { return globalThis.__mediaSdk(); } } export const Modality={AUDIO:'AUDIO',TEXT:'TEXT'}; export const Type={OBJECT:'OBJECT',STRING:'STRING'};" }));
  } }] });
const { GeminiLiveService } = await import('data:text/javascript;base64,' + Buffer.from(built.outputFiles[0].text).toString('base64'));
const deferred = () => { let resolve; const promise = new Promise(r => { resolve = r; }); return { promise, resolve }; };
function replace(t, name, value) {
  const old = Object.getOwnPropertyDescriptor(globalThis, name);
  Object.defineProperty(globalThis, name, { configurable: true, writable: true, value });
  t.after(() => old ? Object.defineProperty(globalThis, name, old) : delete globalThis[name]);
}
function environment(t, { visibility = 'visible', config: patch = {}, media, fetcher } = {}) {
  const listeners = new Set(), tracks = [], contexts = [], calls = { sdk: 0, connect: 0, media: 0, fetch: 0, tools: 0 };
  const document = visibility === null ? undefined : {
    visibilityState: visibility,
    addEventListener(name, f) { assert.equal(name, 'visibilitychange'); listeners.add(f); },
    removeEventListener(name, f) { assert.equal(name, 'visibilitychange'); listeners.delete(f); },
  };
  class Stream {
    constructor(owned = []) { this.tracks = owned; }
    addTrack(track) { this.tracks.push(track); }
    getTracks() { return this.tracks; }
    getAudioTracks() { return this.tracks.filter(track => track.kind === 'audio'); }
    getVideoTracks() { return this.tracks.filter(track => track.kind === 'video'); }
  }
  const node = () => ({ connect() {}, disconnect() {}, getByteTimeDomainData() {} });
  class Context {
    constructor() { this.state = 'running'; this.destination = {}; contexts.push(this); }
    createAnalyser() { return node(); }
    createMediaStreamSource() { return node(); }
    createScriptProcessor() { return node(); }
    async close() { this.state = 'closed'; }
  }
  replace(t, 'document', document);
  replace(t, 'window', { AudioContext: Context, setInterval, speechSynthesis: { cancel() {} } });
  replace(t, 'MediaStream', Stream);
  replace(t, 'navigator', { mediaDevices: { getUserMedia: constraints => {
    calls.media++;
    if (media) return media(constraints);
    const track = { kind: constraints.audio ? 'audio' : 'video', enabled: true, stopped: 0, stop() { this.stopped++; } };
    tracks.push(track); return Promise.resolve(new Stream([track]));
  } } });
  replace(t, 'fetch', (...args) => { calls.fetch++; return fetcher ? fetcher(...args) : Promise.resolve(new Response('', { status: 404 })); });
  replace(t, '__mediaSdk', () => { calls.sdk++; return { live: { connect: async ({ callbacks }) => {
    calls.connect++; callbacks.onopen(); return { close() {}, sendRealtimeInput() {} };
  } } }; });
  const config = { modelName: 'gemini-3.1-flash-live-preview', voiceName: 'Puck', outputPolicy: 'conversation',
    robotApiUrl: 'http://media-admission.test', enableAudio: true, enableVideo: false, quickReceipts: false, ...patch };
  const service = new GeminiLiveService(config, () => {}, () => {}, () => {});
  service.robotBridge.getTools = async () => { calls.tools++; return { prompt: 'Test', tools: [] }; };
  service.setupVideoInput = () => {};
  t.after(() => service.disconnect());
  return { service, config, calls, tracks, contexts, listeners, Stream,
    connect: () => service.connect('TEST', config, { play: async () => {} }),
    visibility(value) { document.visibilityState = value; for (const f of [...listeners]) f(); } };
}

for (const [name, config] of [['microphone', { enableAudio: true }], ['webcam', { enableAudio: false, enableVideo: true, cameraSource: 'browser' }]]) {
  test(`hidden ${name} connection is refused before preload, SDK, media or robot requests`, async t => {
    const e = environment(t, { visibility: 'hidden', config: { ...config, quickReceipts: true } });
    await assert.rejects(e.connect(), /Bring this browser tab forward.*connect/i);
    assert.deepEqual(e.calls, { sdk: 0, connect: 0, media: 0, fetch: 0, tools: 0 });
    assert.equal(e.contexts.length, 0); assert.equal(e.listeners.size, 0);
  });
}

test('tab hidden during receipt preload is refused before any media or SDK creation', async t => {
  const asset = deferred();
  const e = environment(t, { config: { quickReceipts: true }, fetcher: () => asset.promise });
  const pending = e.connect();
  assert.ok(e.calls.fetch > 0); e.visibility('hidden'); asset.resolve(new Response('', { status: 404 }));
  await assert.rejects(pending, /Bring this browser tab forward.*connect/i);
  assert.equal(e.calls.media, 0); assert.equal(e.calls.sdk, 0); assert.equal(e.calls.tools, 0);
  assert.equal(e.listeners.size, 0);
});

for (const [name, config] of [['microphone', {}], ['optional webcam', { enableAudio: false, enableVideo: true, cameraSource: 'browser' }]]) {
  test(`tab hidden during pending ${name} cancels promptly and stops a late grant`, async t => {
    const permission = deferred(); const e = environment(t, { config, media: () => permission.promise });
    const pending = e.connect(); assert.equal(e.calls.media, 1);
    const rejected = assert.rejects(pending, /Bring this browser tab forward.*connect/i);
    e.visibility('hidden'); await rejected;
    assert.equal(e.calls.connect, 0); assert.equal(e.listeners.size, 0);
    assert.ok(e.contexts.every(context => context.state === 'closed'));
    const track = { kind: name === 'microphone' ? 'audio' : 'video', stopped: 0, stop() { this.stopped++; } };
    permission.resolve(new e.Stream([track])); await Promise.resolve();
    assert.equal(track.stopped, 1); assert.equal(e.service.mediaStream, null);
  });
}

test('webcam acquisition rechecks foreground after microphone permission resolves', async t => {
  const microphone = deferred(); const e = environment(t, { config: { enableVideo: true, cameraSource: 'browser' }, media: () => microphone.promise });
  const pending = e.connect();
  const track = { kind: 'audio', stopped: 0, stop() { this.stopped++; } };
  microphone.resolve(new e.Stream([track]));
  // Stream adoption removes the first visibility listener; next microtask
  // hides the tab before the continuation requests the optional webcam.
  await Promise.resolve(); e.visibility('hidden');
  await assert.rejects(pending, /Bring this browser tab forward.*connect/i);
  assert.equal(e.calls.media, 1); assert.equal(e.calls.connect, 0); assert.equal(track.stopped, 1);
  assert.equal(e.listeners.size, 0);
});

for (const [name, visibility, config] of [
  ['visible microphone', 'visible', {}],
  ['Node without document', null, {}],
  ['hidden text-only', 'hidden', { enableAudio: false }],
  ['hidden server camera without microphone', 'hidden', { enableAudio: false, enableVideo: true, cameraSource: 'server' }],
]) {
  test(`${name} can connect; subsequent tab switches do not close an established session`, async t => {
    const e = environment(t, { visibility, config }); await e.connect();
    assert.equal(e.calls.connect, 1); assert.equal(e.service.connected, true);
    assert.equal(e.calls.media, config.enableAudio === false ? 0 : 1); assert.equal(e.listeners.size, 0);
    if (visibility !== null) e.visibility('hidden');
    assert.equal(e.service.connected, true); assert.ok(e.tracks.every(track => track.stopped === 0));
  });
}

for (const ending of ['cancel', 'timeout', 'denial']) {
  test(`pending permission removes its visibility listener on ${ending}`, async t => {
    const permission = deferred(); let expire;
    const e = environment(t, { media: () => ending === 'denial'
      ? Promise.reject(Object.assign(new Error('denied'), { name: 'NotAllowedError' })) : permission.promise });
    if (ending === 'timeout') {
      const native = globalThis.setTimeout;
      replace(t, 'setTimeout', (callback, ms, ...args) => ms === 30_000 ? (expire = callback, 987656) : native(callback, ms, ...args));
    }
    const pending = e.connect();
    const rejected = assert.rejects(pending, ending === 'cancel' ? /cancelled/ : ending === 'timeout' ? /30 seconds/ : /permission denied/);
    assert.equal(e.listeners.size, 1);
    if (ending === 'cancel') await e.service.disconnect();
    if (ending === 'timeout') expire();
    await rejected;
    assert.equal(e.listeners.size, 0); assert.equal(e.calls.connect, 0);
    if (ending !== 'denial') {
      const track = { kind: 'audio', stopped: 0, stop() { this.stopped++; } };
      permission.resolve(new e.Stream([track])); await Promise.resolve(); assert.equal(track.stopped, 1);
    }
  });
}
