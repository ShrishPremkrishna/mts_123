
import { GoogleGenAI, LiveServerMessage, Modality, Type, FunctionDeclaration } from '@google/genai';
import { createPcmBlob, decodeAudioData, blobToBase64 } from './audioUtils';
import { AppConfig, LogEntry } from '../types';
import RobotBridge from './robotBridge';
import { toolResponseStatus, type ToolDispatch } from './executionStatus';

/**
 * Gemini's tools and system prompt come from the robot (GET /tools): they are
 * written in Python, in lab/ (lab/prompt.md, and the @tool functions). Every tool call is
 * sent back to the robot (POST /tools/<name>), and its {success, message} goes
 * to Gemini as the result. Only `speak` lives here: text-only models use it to
 * talk through the browser's speech synthesis.
 */
const SPEAK_TOOL: FunctionDeclaration = {
  name: 'speak',
  description: 'Speak a message out loud to the user via voice synthesis. Use this tool whenever you want to talk, reply, narrate actions, or converse.',
  parameters: {
    type: Type.OBJECT,
    properties: { text: { type: Type.STRING, description: 'The text message to synthesize and speak aloud.' } },
    required: ['text'],
  },
};

export function isAudioOutputModel(modelName: string): boolean {
  const name = modelName.toLowerCase();
  if (name.includes('robotics') || name.includes('er-2') || name.includes('er2')) {
    return false;
  }
  return true;
}

export class GeminiLiveService {
  private ai: GoogleGenAI | null = null;
  private sessionPromise: Promise<any> | null = null;
  private closedSession: any = null;
  private audioContext: AudioContext | null = null;
  private analyser: AnalyserNode | null = null;
  private inputAudioContext: AudioContext | null = null;
  private inputSource: MediaStreamAudioSourceNode | null = null;
  private disposed = false;
  private connecting = false;
  private playbackGeneration = 0;
  private apiKey = '';
  private cancelledToolIds = new Set<string>();
  private dispatchedActionIds = new Set<string>();
  private automaticStop: Promise<void> | null = null;
  private stopRequestsPending = 0;
  private speechVisualInterval: number | null = null;
  private inputScriptProcessor: ScriptProcessorNode | null = null;
  private mediaStream: MediaStream | null = null;
  private nextStartTime = 0;
  private videoInterval: number | null = null;
  private volumeInterval: number | null = null;
  private queuedSources: AudioBufferSourceNode[] = [];
  private connected = false;
  private config: AppConfig;
  private robotToolsPaused = false;
  onToolDispatch: (entry: ToolDispatch) => void;

  public robotBridge: RobotBridge;

  // Callbacks
  onLog: (entry: LogEntry) => void;
  onAudioLevel: (level: number) => void;
  onDisconnect: () => void;

  constructor(
    config: AppConfig,
    onLog: (entry: LogEntry) => void,
    onAudioLevel: (level: number) => void,
    onDisconnect: () => void,
    onToolDispatch: (entry: ToolDispatch) => void = () => {},
    private onAutomaticStop: (message: string) => void = () => {},
    private onStopPending: (pending: boolean) => void = () => {}
  ) {
    this.config = config;
    this.robotBridge = new RobotBridge(config.robotApiUrl);
    this.onLog = onLog;
    this.onAudioLevel = onAudioLevel;
    this.onDisconnect = onDisconnect;
    this.onToolDispatch = onToolDispatch;
    this.queuedSources = [];
  }

  /** Synchronous: later model motion calls cannot rearm; stop/status remain available. */
  public pauseRobotTools() {
    this.robotToolsPaused = true;
  }

  async connect(apiKey: string, config: AppConfig, videoElement: HTMLVideoElement) {
    if (this.disposed || this.connecting || this.connected) throw new Error('Create a new voice session to reconnect.');
    this.connecting = true;
    this.apiKey = apiKey;
    try {
      await this.openSession(apiKey, config, videoElement);
    } catch (error: any) {
      const failedWhileOpen = !this.disposed;
      this.cleanup();
      if (failedWhileOpen && this.dispatchedActionIds.size) await this.requestAutomaticStop('Voice connection failed after an action request');
      // Do not expose a reconnect while an automatic Stop is still outstanding.
      if (this.automaticStop) await this.automaticStop;
      throw new Error(this.safeMessage(error?.message || String(error)));
    } finally {
      this.connecting = false;
    }
  }

  private ensureOpen() {
    if (this.disposed) throw new Error('Voice connection cancelled.');
  }

  private async acquireMedia(constraints: MediaStreamConstraints) {
    const stream = await navigator.mediaDevices.getUserMedia(constraints);
    if (this.disposed) {
      stream.getTracks().forEach(track => track.stop());
      this.ensureOpen();
    }
    // Own each track immediately: a later camera/SDK failure must release the mic.
    if (!this.mediaStream) this.mediaStream = new MediaStream();
    stream.getTracks().forEach(track => this.mediaStream!.addTrack(track));
  }

  private async openSession(apiKey: string, config: AppConfig, videoElement: HTMLVideoElement) {
    this.config = config;
    this.ai = new GoogleGenAI({ apiKey: apiKey });

    this.audioContext = new (window.AudioContext || (window as any).webkitAudioContext)({ sampleRate: 24000 });
    
    // Create Analyser for output audio to visualize speaker sound
    this.analyser = this.audioContext.createAnalyser();
    this.analyser.fftSize = 512;
    this.analyser.smoothingTimeConstant = 0.1;
    this.analyser.connect(this.audioContext.destination);

    this.startOutputVolumeMonitoring();

    if (config.enableAudio || (config.enableVideo && config.cameraSource === 'browser')) {
      if (!navigator.mediaDevices?.getUserMedia) throw new Error('MediaDevices unavailable. Use HTTPS or localhost.');
      if (config.enableAudio) {
        await this.acquireMedia({ audio: { echoCancellation: true, noiseSuppression: true } });
        this.inputAudioContext = new (window.AudioContext || (window as any).webkitAudioContext)({ sampleRate: 16000 });
        this.log('info', 'Microphone connected');
      }
      if (config.enableVideo && config.cameraSource === 'browser') {
        try {
          await this.acquireMedia({ video: { width: { ideal: 640 }, height: { ideal: 480 } } });
          this.log('info', 'Camera connected');
        } catch (error: any) {
          this.ensureOpen();
          this.log('error', `Camera unavailable: ${error.message || error}`);
        }
      }
    }
    this.ensureOpen();

    if (config.enableVideo && config.cameraSource === 'browser' && videoElement && this.mediaStream && this.mediaStream.getVideoTracks().length > 0) {
      videoElement.srcObject = this.mediaStream;
      videoElement.play().catch(e => console.error("Video play failed", e));
    }

    const modelName = config.modelName === 'gemini-2.5-flash-native-audio-preview'
      ? 'gemini-2.5-flash-native-audio-preview-12-2025'
      : config.modelName;
    const isAudioOutput = isAudioOutputModel(modelName);

    // Ask the robot for Gemini's system prompt and tools (lab/prompt.md, the tools in lab/).
    const spec: any = await this.robotBridge.getTools();
    this.ensureOpen();
    const robotTools: FunctionDeclaration[] = Array.isArray(spec?.tools) ? spec.tools : [];
    if (robotTools.length) {
      this.log('info', `Robot offers ${robotTools.length} tools: ${robotTools.map(t => t.name).join(', ')}`);
    } else {
      this.log('error', `Could not get tools from the robot (${spec?.message ?? 'no tools'}); Gemini can talk but not move.`);
    }
    let systemInstruction = [spec?.prompt, config.extraInstructions].filter(Boolean).join('\n\n');
    if (!isAudioOutput) {
      systemInstruction += '\n\nIMPORTANT: You do not have direct voice audio output. To speak out loud to the user or narrate your actions, you MUST invoke the "speak" function tool with your response text.';
    }

    const functionDeclarations = isAudioOutput ? robotTools : [...robotTools, SPEAK_TOOL];

    const liveConfig: any = {
      responseModalities: isAudioOutput ? [Modality.AUDIO] : [Modality.TEXT],
      systemInstruction: systemInstruction,
      tools: [{ functionDeclarations }],
      contextWindowCompression: { slidingWindow: {} },
    };

    // 3.1 Live otherwise defaults to MINIMAL. Keep the tested HIGH setup
    // explicit, without sending an unsupported option to other models.
    // https://ai.google.dev/gemini-api/docs/models/gemini-3.1-flash-live-preview
    if (modelName.toLowerCase().replace(/^models\//, '') === 'gemini-3.1-flash-live-preview') {
      liveConfig.thinkingConfig = { thinkingLevel: 'HIGH' };
    }

    if (isAudioOutput) {
      liveConfig.speechConfig = {
        voiceConfig: { prebuiltVoiceConfig: { voiceName: config.voiceName } },
      };
      // Gemini 3.1 Flash Live rejects these Gemini 2.5-only options during
      // session setup, which otherwise looks like a successful connection
      // followed immediately by an unexplained WebSocket close.
      const supportsExperimentalAudioFeatures = !modelName.toLowerCase().includes('gemini-3.1');
      if (supportsExperimentalAudioFeatures && config.enableAffectiveDialog) {
        liveConfig.enableAffectiveDialog = true;
      }
      if (supportsExperimentalAudioFeatures && config.enableProactiveAudio) {
        liveConfig.proactivity = { proactiveAudio: true };
      }
    }

    // Connect and store the promise
    this.log('info', `Connecting to ${modelName} (Audio Output: ${isAudioOutput ? 'Native' : 'Flash TTS via speak tool'})...`);
    this.sessionPromise = this.ai.live.connect({
      model: modelName,
      config: liveConfig,
      callbacks: {
        onopen: () => {
          if (this.disposed) return;
          this.connected = true;
          this.log('info', 'Session connected');
          if (this.inputAudioContext && this.mediaStream?.getAudioTracks().length) {
            this.setupAudioInput(this.inputAudioContext, this.mediaStream);
          }
          if (config.enableVideo) {
            this.setupVideoInput(videoElement, config.cameraSource);
          }
        },
        onmessage: async (msg: LiveServerMessage) => {
          if (!this.connected) return;
          await this.handleMessage(msg);
        },
        onclose: async (event: CloseEvent) => {
          if (this.disposed) return;
          const details = [
            event.code ? `code ${event.code}` : '',
            event.reason || '',
          ].filter(Boolean).join(': ');
          this.log('error', `Session closed${details ? ` (${details})` : ''}`);
          this.cleanup();
          if (this.dispatchedActionIds.size) await this.requestAutomaticStop('Voice connection closed after an action request');
          this.onDisconnect();
        },
        onerror: (err: ErrorEvent) => {
          if (this.disposed) return;
          const details = err.message || err.error?.message || String(err.error || 'WebSocket error');
          this.log('error', `Session error: ${details}`);

        }
      }
    });

    // Wait for connection to ensure validity
    const session = await this.sessionPromise;
    if (this.disposed) {
      this.closeSession(session);
      this.ensureOpen();
    }
  }

  private startOutputVolumeMonitoring() {
    if (this.volumeInterval) clearInterval(this.volumeInterval);
    this.volumeInterval = window.setInterval(() => {
      if (!this.analyser) return;
      
      const array = new Uint8Array(this.analyser.fftSize);
      this.analyser.getByteTimeDomainData(array);
      
      let sum = 0;
      for (let i = 0; i < array.length; i++) {
        const a = (array[i] - 128) / 128;
        sum += a * a;
      }
      const rms = Math.sqrt(sum / array.length);
      // Boost factor to make mouth movement more visible
      this.onAudioLevel(Math.min(1.0, rms * 3)); 
    }, 50);
  }

  private setupAudioInput(ctx: AudioContext, stream: MediaStream) {
    const source = ctx.createMediaStreamSource(stream);
    this.inputSource = source;
    this.inputScriptProcessor = ctx.createScriptProcessor(4096, 1, 1);
    
    this.inputScriptProcessor.onaudioprocess = (e) => {
      if (!this.connected || !this.sessionPromise) return;

      const inputData = e.inputBuffer.getChannelData(0);
      const pcmBlob = createPcmBlob(inputData);
      
      this.sessionPromise.then(session => {
        if (!this.connected) return;
        try {
           session.sendRealtimeInput({ 
             audio: { 
               data: pcmBlob.data, 
               mimeType: pcmBlob.mimeType 
             } 
           });
        } catch(err) {
           // Ignore errors if session is closing
        }
      });
    };

    source.connect(this.inputScriptProcessor);
    this.inputScriptProcessor.connect(ctx.destination);
  }

  private setupVideoInput(videoEl: HTMLVideoElement, source: 'browser' | 'server') {
    if (this.videoInterval) clearInterval(this.videoInterval);
    
    if (source === 'browser') {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        
        this.videoInterval = window.setInterval(async () => {
          if (!this.connected || !this.sessionPromise || !ctx || !videoEl.videoWidth) return;
          
          canvas.width = videoEl.videoWidth * 0.5;
          canvas.height = videoEl.videoHeight * 0.5;
          ctx.drawImage(videoEl, 0, 0, canvas.width, canvas.height);
          
          canvas.toBlob(async (blob) => {
            if (!blob || !this.connected) return;
            const base64 = await blobToBase64(blob);
            this.sessionPromise?.then(session => {
                if (!this.connected) return;
                try {
                    session.sendRealtimeInput({ 
                      video: { 
                        mimeType: 'image/jpeg', 
                        data: base64 
                      }
                    })
                } catch(err) {}
            });
          }, 'image/jpeg', 0.6);
        }, 1000);
    } else {
        // Server source
        this.videoInterval = window.setInterval(async () => {
             if (!this.connected || !this.sessionPromise) return;
             const blob = await this.robotBridge.getCameraImage();
             if (blob && this.connected) {
                 const base64 = await blobToBase64(blob);
                 this.sessionPromise?.then(session => {
                    if (!this.connected) return;
                    try {
                        session.sendRealtimeInput({ 
                          video: { 
                            mimeType: 'image/jpeg', 
                            data: base64 
                          }
                        })
                    } catch(err) {}
                });
             }
        }, 1000); // 2Hz
    }
  }

  private async playAudioBase64(audioData: string): Promise<void> {
    const ctx = this.audioContext, analyser = this.analyser, generation = this.playbackGeneration;
    if (!ctx || !analyser || this.disposed) return;
    if (ctx.state === 'suspended') await ctx.resume();
    const buffer = await decodeAudioData(audioData, ctx);
    if (this.disposed || generation !== this.playbackGeneration || ctx !== this.audioContext) return;
    this.nextStartTime = Math.max(this.nextStartTime, ctx.currentTime);
    const source = ctx.createBufferSource();
    source.buffer = buffer;
    source.connect(analyser);
    source.start(this.nextStartTime);
    this.queuedSources.push(source);
    this.nextStartTime += buffer.duration;

    // Streaming callers must be able to enqueue the next chunk immediately;
    // waiting for playback to finish here would recreate full-response latency.
    source.onended = () => {
      const idx = this.queuedSources.indexOf(source);
      if (idx !== -1) {
        this.queuedSources.splice(idx, 1);
      }
    };
  }

  private async synthesizeAndPlaySpeech(text: string): Promise<void> {
    const generation = this.playbackGeneration;
    const cancelled = () => this.disposed || generation !== this.playbackGeneration;
    try {
      if (!this.ai) return;
      const configuredModel = this.config.ttsModelName || "gemini-3.1-flash-tts-preview";
      const ttsModel = configuredModel === "gemini-2.5-flash-preview"
        ? "gemini-2.5-flash-preview-tts"
        : configuredModel;
      const supportsStreaming = ttsModel.startsWith("gemini-3.1-");
      this.log("info", "Synthesizing speech via " + ttsModel + (supportsStreaming ? " (streaming)..." : "..."));

      const request: any = {
        model: ttsModel,
        contents: [{ parts: [{ text }] }],
        config: {
          responseModalities: ["AUDIO"],
          speechConfig: {
            voiceConfig: {
              prebuiltVoiceConfig: {
                voiceName: this.config.voiceName || "Puck",
              }
            }
          }
        }
      };

      let receivedAudio = false;
      if (supportsStreaming) {
        const responseStream = await this.ai.models.generateContentStream(request);
        for await (const chunk of responseStream) {
          if (cancelled()) return;
          const parts = chunk.candidates?.[0]?.content?.parts || [];
          for (const part of parts) {
            const audioData = part.inlineData?.data;
            if (audioData) {
              receivedAudio = true;
              await this.playAudioBase64(audioData);
            }
          }
        }
      } else {
        const response = await this.ai.models.generateContent(request);
        if (cancelled()) return;
        const parts = response.candidates?.[0]?.content?.parts || [];
        for (const part of parts) {
          const audioData = part.inlineData?.data;
          if (audioData) {
            receivedAudio = true;
            await this.playAudioBase64(audioData);
          }
        }
      }

      if (cancelled()) return;
      if (!receivedAudio) {
        console.warn("No audio data returned by TTS model, using browser speech fallback");
        this.speakWithWebSpeech(text);
      }
    } catch (err: any) {
      if (cancelled()) return;
      this.log("error", "TTS error: " + (err.message || err) + ". Using browser speech fallback.");
      this.speakWithWebSpeech(text);
    }
  }

  private speakWithWebSpeech(text: string): void {
    if (typeof window !== 'undefined' && 'speechSynthesis' in window) {
      window.speechSynthesis.cancel();
      const utterance = new SpeechSynthesisUtterance(text);
      const generation = this.playbackGeneration;
      utterance.onstart = () => {
        if (this.disposed || generation !== this.playbackGeneration) return;
        this.speechVisualInterval = window.setInterval(() => {
          this.onAudioLevel(0.2 + Math.random() * 0.3);
        }, 100);
      };
      const stopVisual = () => {
        if (this.speechVisualInterval) clearInterval(this.speechVisualInterval);
        this.speechVisualInterval = null;
        this.onAudioLevel(0);
      };
      utterance.onend = stopVisual;
      utterance.onerror = stopVisual;
      window.speechSynthesis.speak(utterance);
    }
  }

  private async handleMessage(message: LiveServerMessage) {
    if (this.disposed) return;
    // Interruption must invalidate pending decodes/TTS before any awaits.
    if (message.serverContent?.interrupted) {
      this.log('info', 'Model speech interrupted; robot actions are unchanged');
      this.stopPlayback();
    }
    let cancelSentAction = false;
    for (const id of message.toolCallCancellation?.ids ?? []) {
      this.cancelledToolIds.add(id);
      if (this.dispatchedActionIds.has(id)) cancelSentAction = true;
    }
    if (cancelSentAction) await this.requestAutomaticStop('Live cancelled an already-dispatched action');
    const generation = this.playbackGeneration;

    // Check for text output (useful for ER2 streaming / text models)
    const textPart = message.serverContent?.modelTurn?.parts?.find((p: any) => p.text)?.text;
    if (textPart) {
      this.log('model', `Text: ${textPart}`);
    }

    if (message.toolCall) {
      const calls = message.toolCall.functionCalls ?? [];
      const hasStop = calls.some(call => call.name === 'stop' && !(call.id && this.cancelledToolIds.has(call.id)));
      // Stop is an interrupt, never the tail of a queued sequence. Dispatch it
      // first and reject other actions in that same model batch before HTTP.
      const ordered = hasStop ? [...calls].sort((a, b) => Number(b.name === 'stop') - Number(a.name === 'stop')) : calls;
      for (const fc of ordered) {
        if (this.disposed) break;
        if (fc.id && this.cancelledToolIds.has(fc.id)) {
          this.log('info', `Cancelled before dispatch: ${fc.name}`);
          continue;
        }
        this.log('function', `Call: ${fc.name}(${fc.name === 'unlock' ? '[redacted]' : JSON.stringify(fc.args)})`);
        let result: any = { success: true };
        
        try {
          // Helper args
          const args = fc.args as any;

          switch (fc.name) {
            case 'speak':
            case 'say':
            case 'narrate':
              if (isAudioOutputModel(this.config.modelName)) {
                result = { success: false, error: 'Native-audio models must speak through their Live API audio response' };
                break;
              }
              const speechText = args.text || args.message || args.content || '';
              if (speechText) {
                this.log('model', `Pupper: "${speechText}"`);
                await this.synthesizeAndPlaySpeech(speechText);
                result = { success: true, spoken: true, text: speechText };
              } else {
                result = { success: false, error: 'No text provided to speak' };
              }
              break;
            default:
              // Every other tool is defined and run on the robot (the tools in lab/).
              if (hasStop && fc.name !== 'stop' && fc.name !== 'robot_status') {
                result = { success: false, message: 'Not dispatched: Stop takes priority over other actions in the same batch. No delayed Stop was scheduled.' };
                break;
              }
              if (this.stopRequestsPending > 0 && fc.name !== 'stop' && fc.name !== 'robot_status') {
                result = { success: false, message: 'Not dispatched: a Stop response is pending. Check status before requesting a new action.' };
                break;
              }
              if (this.robotToolsPaused && fc.name !== 'stop' && fc.name !== 'robot_status') {
                result = { success: false, message: 'Model motion requests paused. Stop and status remain available; reconnect voice to enable motion requests.' };
                break;
              }
              const dispatch = { id: fc.id || crypto.randomUUID(), name: fc.name || 'unknown', timestamp: Date.now() };
              if (!['stop', 'robot_status', 'unlock', 'lock_tricks'].includes(dispatch.name)) this.dispatchedActionIds.add(dispatch.id);
              this.onToolDispatch({ ...dispatch, status: 'sending', message: 'Sending request' });
              // This lifecycle remains balanced even after disconnect suppresses
              // UI receipts. A late Stop must not race new manual motion.
              const stopping = fc.name === 'stop';
              if (stopping) this.reportStopPending(true);
              try { result = await this.robotBridge.callTool(fc.name, args ?? {}); }
              finally { if (stopping) this.reportStopPending(false); }
              const status = toolResponseStatus(result);
              if (status === 'unknown') this.pauseRobotTools();
              if (this.connected) this.onToolDispatch({ ...dispatch, status, message: result?.message || 'No response details', timestamp: Date.now() });
          }
        } catch (e: any) {
          result = { success: false, error: e.message };
        }

        this.log(result?.success === false ? 'error' : 'function', `Result: ${fc.name} -> ${result?.message ?? JSON.stringify(result)}`);
        if (this.connected && this.sessionPromise && !(fc.id && this.cancelledToolIds.has(fc.id))) {
          this.sessionPromise.then(session => {
            if (!this.connected) return;
            try {
              session.sendToolResponse({
                functionResponses: {
                  id: fc.id,
                  name: fc.name,
                  response: { result }
                }
              });
            } catch (err) {
              // Session may have closed between check and send
            }
          });
        }
      }
    }

    // Dispatch tool requests (especially Stop) before waiting for audio decoding.
    for (const part of message.serverContent?.modelTurn?.parts ?? []) {
      if (generation !== this.playbackGeneration || this.disposed) break;
      if (part.inlineData?.data) await this.playAudioBase64(part.inlineData.data);
    }

  }

  private reportStopPending(pending: boolean) {
    this.stopRequestsPending += pending ? 1 : -1;
    this.onStopPending(pending);
  }

  private requestAutomaticStop(reason: string): Promise<void> {
    this.pauseRobotTools();
    if (this.automaticStop) return this.automaticStop;
    // No per-job cancellation IDs exist. This deliberately cancels the whole
    // robot queue/follow, never retries a command, and requires manual reconnect.
    this.reportStopPending(true);
    this.onAutomaticStop(`${reason}. Global Stop requested; waiting for server response.`);
    this.automaticStop = (async () => {
      try {
        let result: any;
        try { result = await this.robotBridge.immediateStop(); }
        catch { result = { outcome_unknown: true }; }
        const status = toolResponseStatus(result);
        const message = status === 'unknown'
          ? 'Global Stop response unknown. Observe the robot and use R3/power if needed.'
          : `Global Stop ${status}: ${result.message || 'No details'}. This does not prove physical stillness.`;
        this.log(status === 'accepted' ? 'info' : 'error', message);
        this.onAutomaticStop(message);
      } finally {
        this.reportStopPending(false);
      }
    })();
    return this.automaticStop;
  }

  private stopPlayback() {
    this.playbackGeneration++;
    if (this.speechVisualInterval) clearInterval(this.speechVisualInterval);
    this.speechVisualInterval = null;
    this.queuedSources.forEach(source => { try { source.stop(); } catch {} });
    this.queuedSources = [];
    this.nextStartTime = 0;
    if (typeof window !== 'undefined' && 'speechSynthesis' in window) window.speechSynthesis.cancel();
    this.onAudioLevel(0);
  }

  /**
   * Stop all intervals and null out session/resources.
   * Called both on explicit disconnect and on server-side onclose.
   */
  private cleanup() {
    this.disposed = true;
    this.connected = false;
    this.stopPlayback();
    // An old onmessage handler may still be waiting for a tool response.
    // Do not let the remainder of that message dispatch motion after close.
    this.pauseRobotTools();

    if (typeof window !== 'undefined' && 'speechSynthesis' in window) {
      window.speechSynthesis.cancel();
    }

    this.inputSource?.disconnect();
    this.inputSource = null;
    if (this.inputScriptProcessor) {
      this.inputScriptProcessor.onaudioprocess = null;
      this.inputScriptProcessor.disconnect();
      this.inputScriptProcessor = null;
    }
    if (this.videoInterval) {
      clearInterval(this.videoInterval);
      this.videoInterval = null;
    }
    if (this.volumeInterval) {
      clearInterval(this.volumeInterval);
      this.volumeInterval = null;
    }
    if (this.mediaStream) {
      this.mediaStream.getTracks().forEach(track => track.stop());
      this.mediaStream = null;
    }
    this.analyser = null;
    for (const ctx of [this.audioContext, this.inputAudioContext]) {
      if (ctx && ctx.state !== 'closed') void ctx.close().catch(() => {});
    }
    this.audioContext = null;
    this.inputAudioContext = null;
  }

  public async disconnect() {
    this.cleanup();
    // Never wait for a hung SDK connection. A session that opens late is closed.
    const pending = this.sessionPromise;
    this.sessionPromise = null;
    this.ai = null;
    if (pending) void pending.then(session => this.closeSession(session)).catch(() => {});
  }

  private closeSession(session: any) {
    if (session === this.closedSession) return;
    this.closedSession = session;
    try { session.close(); } catch { /* Already closed by the remote peer. */ }
  }

  private safeMessage(message: string) {
    const redacted = this.apiKey ? message.split(this.apiKey).join('[redacted]') : message;
    return redacted.replace(/([?&]key=)[^&\s]+/gi, '$1[redacted]');
  }

  private log(type: LogEntry['type'], message: string) {
    this.onLog({
      id: Math.random().toString(36),
      timestamp: Date.now(),
      type,
      message: this.safeMessage(message)
    });
  }
}
