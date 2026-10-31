// Build-time alias: no Google SDK, credentials, WebSocket or network connection.
export const Modality = { AUDIO: 'AUDIO', TEXT: 'TEXT' };
export const Type = { OBJECT: 'OBJECT', STRING: 'STRING' };
export const mock = {
  callbacks: null as any,
  audioFrames: 0,
  nonzeroFrames: 0,
  audioBytes: 0,
  videoFrames: [] as { data: string; mimeType: string }[],
  responses: [] as any[],
  closed: 0,
  setups: [] as any[],
  ttsCalls: 0,
  emit(message: any): Promise<void> {
    if (!this.callbacks) throw new Error('Mock Live session is not connected.');
    // Like the real SDK, one callback does not await a previous callback.
    return Promise.resolve(this.callbacks.onmessage(message));
  },
};
export class GoogleGenAI {
  constructor(_options: any) {}
  live = { connect: async ({ callbacks, config }: any) => {
    mock.setups.push(config);
    mock.callbacks = callbacks;
    const session = {
      sendRealtimeInput(value: any) {
        if (value.video) {
          if (value.audio || value.video.mimeType !== 'image/jpeg' || !value.video.data) throw new Error('Expected one synthetic JPEG frame.');
          mock.videoFrames.push({ data: value.video.data, mimeType: value.video.mimeType });
          return;
        }
        if (!value.audio) throw new Error('Only synthetic audio/video is allowed.');
        const data = atob(value.audio.data);
        mock.audioFrames++; mock.audioBytes += data.length;
        if ([...data].some(c => c.charCodeAt(0) !== 0)) mock.nonzeroFrames++;
      },
      sendToolResponse(value: any) { mock.responses.push(value); },
      close() { mock.closed++; },
    };
    callbacks.onopen();
    return session;
  } };
  models = {
    generateContent() { mock.ttsCalls++; throw new Error('TTS network access is prohibited in rehearsal.'); },
    generateContentStream() { mock.ttsCalls++; throw new Error('TTS network access is prohibited in rehearsal.'); },
  };
}
