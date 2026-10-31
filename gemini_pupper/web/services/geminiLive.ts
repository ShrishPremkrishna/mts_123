
import { GoogleGenAI, LiveServerMessage, Modality, Type, FunctionDeclaration } from '@google/genai';
import { createPcmBlob, decodeAudioData, blobToBase64 } from './audioUtils';
import { AppConfig, LogEntry, OutputPolicy } from '../types';
import RobotBridge from './robotBridge';
import { normalizeOutputPolicy, operatorFeedback, ReceiptSpeaker, toolLabel } from './operatorFeedback';
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

/** Optional diagnostics: no PCM, transcript text, tool arguments or credentials. */
export interface LiveTimingEvent {
  phase: string;
  atMs: number;
  metadata: Record<string, string | number | boolean | null>;
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
  public readonly outputPolicy: OutputPolicy;
  private receiptSpeaker = new ReceiptSpeaker();
  private feedbackGeneration = 0;
  private feedbackRequest = 0;
  private apiKey = '';
  private cancelledToolIds = new Set<string>();
  private dispatchedActionIds = new Set<string>();
  private automaticStop: Promise<void> | null = null;
  private stopRequestsPending = 0;
  private messageSequence = 0;
  private audioSequence = 0;
  private speechVisualInterval: number | null = null;
  private inputScriptProcessor: ScriptProcessorNode | null = null;
  private mediaStream: MediaStream | null = null;
  private nextStartTime = 0;
  private videoInterval: number | null = null;
  private videoGeneration = 0;
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
    private onStopPending: (pending: boolean) => void = () => {},
    private onTiming?: (event: LiveTimingEvent) => void
  ) {
    this.config = config;
    this.outputPolicy = normalizeOutputPolicy(config.outputPolicy);
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
    this.stopPlayback();
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

  private trace(phase: string, metadata: LiveTimingEvent['metadata'] = {}) {
    if (!this.onTiming) return;
    // A diagnostic consumer must never delay/fail dispatch by throwing.
    try { this.onTiming({ phase, atMs: performance.now(), metadata }); } catch {}
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
    const offeredTools: FunctionDeclaration[] = Array.isArray(spec?.tools) ? spec.tools : [];
    const robotTools = this.outputPolicy === 'conversation'
      ? offeredTools.filter(tool => tool.name === 'stop' || tool.name === 'robot_status') : offeredTools;
    if (robotTools.length) {
      this.log('info', `Robot offers ${robotTools.length} tools: ${robotTools.map(t => t.name).join(', ')}`);
    } else {
      this.log('error', `Could not get tools from the robot (${spec?.message ?? 'no tools'}); no robot tools are available.`);
    }
    let systemInstruction = [spec?.prompt, config.extraInstructions].filter(Boolean).join('\n\n');
    if (!isAudioOutput && this.outputPolicy === 'conversation') {
      systemInstruction += '\n\nIMPORTANT: You do not have direct voice audio output. To speak out loud to the user or narrate your actions, you MUST invoke the "speak" function tool with your response text.';
    }

    const functionDeclarations = !isAudioOutput && this.outputPolicy === 'conversation' ? [...robotTools, SPEAK_TOOL] : robotTools;
    systemInstruction += this.outputPolicy === 'receipts'
      ? '\n\nThe operator is using receipt-only control. Your free-form speech and text are muted; tool receipts are presented by the application. Continue using tools for requests.'
      : '\n\nThis is unverified conversation mode. You cannot move the robot or change gates. Only Stop and robot status tools are available; do not imply that conversation has executed an action.';

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
    this.log('info', this.outputPolicy === 'receipts' ? 'Receipt-only control: model narration is muted.' : 'UNVERIFIED CONVERSATION: model movement and gate tools are disabled.');
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
    this.trace('audio_input_setup', { sample_rate: ctx.sampleRate ?? null, buffer_frames: 4096, encoded_rate: 16000 });
    
    this.inputScriptProcessor.onaudioprocess = (e) => {
      if (!this.connected || !this.sessionPromise) return;

      const inputData = e.inputBuffer.getChannelData(0);
      const audioSequence = ++this.audioSequence;
      this.trace('audio_capture', { audio_sequence: audioSequence, samples: inputData.length,
        sample_rate: e.inputBuffer.sampleRate ?? ctx.sampleRate ?? null });
      const pcmBlob = createPcmBlob(inputData);
      
      this.sessionPromise.then(session => {
        if (!this.connected) return;
        try {
           this.trace('audio_send_start', { audio_sequence: audioSequence });
           session.sendRealtimeInput({ 
             audio: { 
               data: pcmBlob.data, 
               mimeType: pcmBlob.mimeType 
             } 
           });
           // SDK method return is not proof of wire delivery or model receipt.
           this.trace('audio_send_return', { audio_sequence: audioSequence });
        } catch(err) {
           this.trace('audio_send_failed', { audio_sequence: audioSequence });
           // Ignore errors if session is closing
        }
      });
    };

    source.connect(this.inputScriptProcessor);
    this.inputScriptProcessor.connect(ctx.destination);
  }

  private setupVideoInput(videoEl: HTMLVideoElement, source: 'browser' | 'server') {
    if (this.videoInterval) clearInterval(this.videoInterval);
    const generation = ++this.videoGeneration;
    let captured = 0, sent = 0, latestServerResponse = 0;
    let serverAvailable: boolean | null = null;
    const active = () => this.connected && this.sessionPromise && generation === this.videoGeneration;
    const currentFrame = (sequence: number) => active() && sequence > sent
      && (source !== 'server' || sequence >= latestServerResponse);
    const sendFrame = async (blob: Blob, sequence: number) => {
      if (!currentFrame(sequence)) return;
      try {
        const base64 = await blobToBase64(blob);
        if (!currentFrame(sequence)) return;
        const session = await this.sessionPromise;
        if (!currentFrame(sequence)) return;
        // FileReader/HTTP completion can reorder captures. Never replace a
        // newer transmitted image with a slower old frame.
        sent = sequence;
        session.sendRealtimeInput({ video: { mimeType: 'image/jpeg', data: base64 } });
      } catch { /* A failed frame is skipped; never retry stale camera data. */ }
    };
    if (source === 'browser') {
      const canvas = document.createElement('canvas');
      const ctx = canvas.getContext('2d');
      this.videoInterval = window.setInterval(() => {
        if (!active() || !ctx || !videoEl.videoWidth) return;
        const sequence = ++captured;
        canvas.width = videoEl.videoWidth * 0.5;
        canvas.height = videoEl.videoHeight * 0.5;
        ctx.drawImage(videoEl, 0, 0, canvas.width, canvas.height);
        canvas.toBlob(blob => { if (blob) void sendFrame(blob, sequence); }, 'image/jpeg', 0.6);
      }, 1000);
    } else {
      this.videoInterval = window.setInterval(async () => {
        if (!active()) return;
        const sequence = ++captured;
        const blob = await this.robotBridge.getCameraImage();
        if (!active() || sequence < latestServerResponse) return;
        latestServerResponse = sequence;
        const available = !!blob;
        if (available !== serverAvailable) {
          serverAvailable = available;
          this.log(available ? 'info' : 'error', available
            ? 'Robot camera frame received.'
            : 'Robot camera image unavailable; no new frame received. Earlier images may remain in the conversation.');
        }
        if (blob) await sendFrame(blob, sequence);
      }, 1000);
    }
  }

  private async playAudioBase64(audioData: string): Promise<void> {
    if (this.outputPolicy !== 'conversation') return;
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
    if (this.outputPolicy !== 'conversation') return;
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
    if (this.outputPolicy !== 'conversation' || this.disposed) return;
    try { if (typeof window !== 'undefined' && 'speechSynthesis' in window) {
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
    } } catch { /* Conversation audio is optional; never block tools or Stop. */ }
  }

  private async handleMessage(message: LiveServerMessage) {
    const messageSequence = ++this.messageSequence;
    const content = message.serverContent;
    this.trace('server_message_callback', { message_sequence: messageSequence,
      tool_count: message.toolCall?.functionCalls?.length ?? 0,
      audio_parts: content?.modelTurn?.parts?.filter(part => part.inlineData?.data).length ?? 0,
      input_transcription_chars: content?.inputTranscription?.text?.length ?? 0,
      input_transcription_finished: content?.inputTranscription?.finished ?? null,
      output_transcription_chars: content?.outputTranscription?.text?.length ?? 0,
      interrupted: !!content?.interrupted, turn_complete: !!content?.turnComplete,
      generation_complete: !!content?.generationComplete, disposed: this.disposed });
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
    if (message.toolCallCancellation?.ids?.length) this.stopPlayback();
    if (cancelSentAction) await this.requestAutomaticStop('Live cancelled an already-dispatched action');
    const generation = this.playbackGeneration;

    // Check for text output (useful for ER2 streaming / text models)
    const textPart = message.serverContent?.modelTurn?.parts?.find((p: any) => p.text)?.text;
    if (textPart && this.outputPolicy === 'conversation') {
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
        if (fc.name === 'stop') this.stopPlayback();
        let feedbackGeneration = this.feedbackGeneration;
        const feedbackRequest = ++this.feedbackRequest;
        if (fc.id && this.cancelledToolIds.has(fc.id)) {
          this.log('info', `Cancelled before dispatch: ${toolLabel(fc.name)}`);
          continue;
        }
        this.log('function', `Request: ${toolLabel(fc.name)}`);
        let result: any = { success: true };
        let dispatch: { id: string; name: string; timestamp: number } | null = null;
        
        try {
          // Helper args
          const args = fc.args as any;

          switch (fc.name) {
            case 'speak':
            case 'say':
            case 'narrate':
              if (this.outputPolicy === 'receipts') {
                result = { success: false, message: 'Free-form speech is muted in receipt-only control.' };
                break;
              }
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
              // This is enforced locally even if a model calls an unadvertised tool.
              if (this.outputPolicy === 'conversation' && fc.name !== 'stop' && fc.name !== 'robot_status') {
                result = { success: false, not_dispatched: true, message: 'Conversation mode cannot move the robot or change gates.' };
                break;
              }
              // Every other tool is defined and run on the robot (the tools in lab/).
              if (hasStop && fc.name !== 'stop' && fc.name !== 'robot_status') {
                result = { success: false, not_dispatched: true, message: 'Not dispatched: Stop takes priority over other actions in the same batch. No delayed Stop was scheduled.' };
                break;
              }
              if (this.stopRequestsPending > 0 && fc.name !== 'stop' && fc.name !== 'robot_status') {
                result = { success: false, not_dispatched: true, message: 'Not dispatched: a Stop response is pending. Check status before requesting a new action.' };
                break;
              }
              if (this.robotToolsPaused && fc.name !== 'stop' && fc.name !== 'robot_status') {
                result = { success: false, not_dispatched: true, message: 'Model motion requests paused. Stop and status remain available; reconnect voice to enable motion requests.' };
                break;
              }
              dispatch = { id: fc.id || crypto.randomUUID(), name: fc.name || 'unknown', timestamp: Date.now() };
              if (!['stop', 'robot_status', 'unlock', 'lock_tricks'].includes(dispatch.name)) this.dispatchedActionIds.add(dispatch.id);
              this.onToolDispatch({ ...dispatch, status: 'sending', message: 'Sending request' });
              // This lifecycle remains balanced even after disconnect suppresses
              // UI receipts. A late Stop must not race new manual motion.
              const stopping = fc.name === 'stop';
              if (stopping) this.reportStopPending(true);
              const timing = { message_sequence: messageSequence, request_id: dispatch.id, tool: dispatch.name };
              this.trace('tool_call_start', timing);
              try { result = await this.robotBridge.callTool(fc.name, args ?? {}); }
              catch (error) { this.trace('tool_call_failed', timing); throw error; }
              finally { if (stopping) this.reportStopPending(false); }
              const status = toolResponseStatus(result);
              this.trace('tool_call_resolved', { ...timing, status });

          }
        } catch (e: any) {
          result = { success: false, outcome_unknown: true, error: e.message };
        }

        const currentFeedback = () => this.connected && !this.disposed && feedbackGeneration === this.feedbackGeneration
          && feedbackRequest === this.feedbackRequest && !(fc.id && this.cancelledToolIds.has(fc.id));
        if (dispatch) {
          const status = toolResponseStatus(result);
          if (status === 'unknown' || (result?.success === false && result?.command_epoch_required === true)) {
            const wasCurrent = currentFeedback();
            this.pauseRobotTools();
            // Keep this current unknown/error receipt, while invalidating older speech.
            if (wasCurrent) feedbackGeneration = this.feedbackGeneration;
          }
          if (currentFeedback()) this.onToolDispatch({ ...dispatch, status, message: operatorFeedback(fc.name, result), timestamp: Date.now() });
        }
        const speechAlias = ['speak', 'say', 'narrate'].includes(fc.name || '');
        if (!speechAlias && currentFeedback()) {
          const feedback = operatorFeedback(fc.name, result);
          this.log('receipt', feedback);
          if (this.outputPolicy === 'receipts') this.receiptSpeaker.say(feedback);
        }
        if (this.connected && this.sessionPromise && !(fc.id && this.cancelledToolIds.has(fc.id))) {
          this.sessionPromise.then(session => {
            if (!this.connected) return;
            try {
              this.trace('function_response_send_start', { message_sequence: messageSequence,
                request_id: fc.id ?? '', tool: fc.name ?? 'unknown' });
              session.sendToolResponse({
                functionResponses: {
                  id: fc.id,
                  name: fc.name,
                  response: { result }
                }
              });
              this.trace('function_response_send_return', { message_sequence: messageSequence,
                request_id: fc.id ?? '', tool: fc.name ?? 'unknown' });
            } catch (err) {
              this.trace('function_response_send_failed', { message_sequence: messageSequence,
                request_id: fc.id ?? '', tool: fc.name ?? 'unknown' });
              // Session may have closed between check and send
            }
          });
        }
      }
    }

    // Dispatch tool requests (especially Stop) before waiting for audio decoding.
    for (const part of this.outputPolicy === 'conversation' ? message.serverContent?.modelTurn?.parts ?? [] : []) {
      if (generation !== this.playbackGeneration || this.disposed) break;
      if (part.inlineData?.data) {
        this.trace('output_audio_handle_start', { message_sequence: messageSequence });
        await this.playAudioBase64(part.inlineData.data);
        this.trace('output_audio_handle_return', { message_sequence: messageSequence });
      }
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
        this.trace('automatic_stop_call_start');
        try { result = await this.robotBridge.immediateStop(); }
        catch { result = { outcome_unknown: true }; }
        const status = toolResponseStatus(result);
        this.trace('automatic_stop_call_resolved', { status });
        const message = operatorFeedback('stop', result);
        this.log(status === 'accepted' ? 'info' : 'error', message);
        this.onAutomaticStop(message);
      } finally {
        this.reportStopPending(false);
      }
    })();
    return this.automaticStop;
  }

  private stopPlayback() {
    this.feedbackGeneration++;
    this.receiptSpeaker.cancel();
    this.playbackGeneration++;
    if (this.speechVisualInterval) clearInterval(this.speechVisualInterval);
    this.speechVisualInterval = null;
    this.queuedSources.forEach(source => { try { source.stop(); } catch {} });
    this.queuedSources = [];
    this.nextStartTime = 0;
    try { if (typeof window !== 'undefined' && 'speechSynthesis' in window) window.speechSynthesis.cancel(); } catch { /* Audio cleanup cannot block Stop. */ }
    this.onAudioLevel(0);
  }

  /**
   * Stop all intervals and null out session/resources.
   * Called both on explicit disconnect and on server-side onclose.
   */
  private cleanup() {
    this.disposed = true;
    this.connected = false;
    this.videoGeneration++;
    this.stopPlayback();
    // An old onmessage handler may still be waiting for a tool response.
    // Do not let the remainder of that message dispatch motion after close.
    this.pauseRobotTools();

    try { if (typeof window !== 'undefined' && 'speechSynthesis' in window) window.speechSynthesis.cancel(); } catch { /* Optional audio. */ }

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
    if (type === 'model') {
      if (this.outputPolicy !== 'conversation') return;
      message = `UNVERIFIED CONVERSATION: ${message}`;
    }
    this.onLog({
      id: Math.random().toString(36),
      timestamp: Date.now(),
      type,
      message: this.safeMessage(message)
    });
  }
}
