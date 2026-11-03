
import { GoogleGenAI, LiveServerMessage, Modality, FunctionDeclaration } from '@google/genai';
import { createPcmBlob, decodeAudioData, blobToBase64 } from './audioUtils';
import { AppConfig, LogEntry, OutputPolicy, VoiceTransportEnded } from '../types';
import RobotBridge from './robotBridge';
import { ConversationOutputGate, NativeReceiptOutputGate, HANDSHAKE_JOKE, type ConversationOutput } from './conversationOutput';
import { normalizeOutputPolicy, operatorFeedback, nativeReceiptFeedback, toolLabel, hasMalformedOutcomeFlags, isOperatorReceiptNarration } from './operatorFeedback';
import { toolResponseStatus, acceptedJobIdentity, type ToolDispatch } from './executionStatus';
import { ReceiptAudioCache, serverReceiptAudioText, supportsPuckReceiptConsumer } from './receiptAudioCache';
import { preloadReceiptAudio, supportsQuickReceipts } from './receiptAudioPreload';
import { FOLLOW_HANDOFF_CUE, followHandoffTools, confirmedFollowHandoff } from './followHandoff';

/** A background permission request may never be presented by the browser. */
class MediaVisibilityError extends Error {
  constructor() {
    super('Bring this browser tab forward, then connect to enable microphone or camera access.');
    this.name = 'MediaVisibilityError';
  }
}

/** Robot tools come from GET /tools. TEXT models return ordinary completed text;
 * the application screens it and sends it to the configured TTS voice. */
export function isAudioOutputModel(modelName: string): boolean {
  const name = modelName.toLowerCase();
  if (name.includes('robotics') || name.includes('er-2') || name.includes('er2')) {
    return false;
  }
  return true;
}

/** Existing structured server catalog must advertise the complete handshake path. */
export function supportsPawClarification(tools: FunctionDeclaration[], tricks: any, actions: any): boolean {
  const names = new Set(tools.map(tool => tool.name));
  return ['robot_status', 'play_trick', 'interactive_action'].every(name => names.has(name))
    && !!tricks && Object.prototype.hasOwnProperty.call(tricks, 'sit_stay_shake')
    && !!actions && ['shake_left', 'shake_right'].every(name => Object.prototype.hasOwnProperty.call(actions, name)
      && actions[name]?.trick === 'sit_stay_shake');
}

/** Known status/gate failures alone do not fence motion; Stop/status remain usable when paused. */
function isModelAction(name: unknown): boolean {
  return !['stop', 'robot_status', 'unlock', 'lock_tricks', 'speak', 'say', 'narrate'].includes(String(name));
}

/** Optional diagnostics: no PCM, transcript text, tool arguments or credentials. */
export interface LiveTimingEvent {
  phase: string;
  atMs: number;
  metadata: Record<string, string | number | boolean | null>;
}

interface TurnTiming {
  id: number;
  began: number;
  audioReleaseSent?: number;
  handlers: number;
  tools: number;
  inputFirst?: number;
  inputEnd?: number;
  outputFirst?: number;
  audioFirst?: number;
  toolFirst?: number;
  toolReply?: number;
  receipt?: number;
  generated?: number;
  complete?: number;
  nativeScheduled?: number;
  expectsNative: boolean;
  logged?: boolean;
  playbackLogged?: boolean;
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
  private feedbackGeneration = 0;
  private feedbackRequest = 0;
  private pendingRobotTools = 0;
  private pendingModelActions = 0;
  private conversationOutput: ConversationOutputGate;
  private nativeReceiptOutput: NativeReceiptOutputGate;
  private apiKey = '';
  private pawClarificationAvailable = false;
  private followHandoffToolNames = new Set<string>();
  private cancelFollowHandoffAudio: (() => void) | null = null;
  private receiptPreloadController: AbortController | null = null;
  private cancelledToolIds = new Set<string>();
  private dispatchedActionIds = new Set<string>();
  // Session-lifetime admission tombstones, including refused/expired calls.
  // Turn resets must not revive a replay. A new service gets fresh sets.
  private claimedToolIds = new Set<string>();
  private claimedStopIds = new Set<string>();
  private automaticStop: Promise<void> | null = null;
  private stopRequestsPending = 0;
  private messageSequence = 0;
  // SDK callbacks overlap. Preserve ordinary packet order through admission
  // replies, not job completion. Stop bypasses this lane and expires old work.
  private ordinaryToolTail: Promise<void> | null = null;
  private toolAdmissionGeneration = 0;
  private timingSequence = 0;
  private currentTurnTiming: TurnTiming | null = null;
  private outputTurnTiming: TurnTiming | undefined;
  private audioSequence = 0;
  private audioInputActive = false;
  private handsFreeDisarmed = false;
  private audioInputGeneration = 0;
  private audioBoundaryFailed = false;
  private lastAudioReleaseSent?: number;
  private speechVisualInterval: number | null = null;
  private inputScriptProcessor: ScriptProcessorNode | null = null;
  private mediaStream: MediaStream | null = null;
  private cancelMediaAcquisition: (() => void) | null = null;
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

  public get voiceInputMode() {
    return this.config.voiceInputMode === 'hands_free' ? 'hands_free' : 'hold_to_talk';
  }

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
    private onTiming?: (event: LiveTimingEvent) => void,
    private onAudioInputChange: (active: boolean) => void = () => {},
    private receiptAudioCache?: ReceiptAudioCache,
    private onRobotToolsPaused: () => void = () => {},
    private onTransportEnded: (event: VoiceTransportEnded) => void = () => {}
  ) {
    this.config = config;
    this.outputPolicy = normalizeOutputPolicy(config.outputPolicy);
    this.conversationOutput = new ConversationOutputGate(
      output => { void this.releaseConversation(output, this.outputTurnTiming).catch(() => this.log('info', 'Conversation playback failed; no replacement narration was produced.')); }, message => {
        this.log('info', message);
      });
    this.nativeReceiptOutput = new NativeReceiptOutputGate(audio => { if (!isAudioOutputModel(this.config.modelName)) return; void this.releaseNativeReceipt(audio).catch(() => this.log('info', 'Receipt audio unavailable; the server receipt remains visible.')); });
    this.robotBridge = new RobotBridge(config.robotApiUrl);
    this.onLog = onLog;
    this.onAudioLevel = onAudioLevel;
    this.onDisconnect = onDisconnect;
    this.onToolDispatch = onToolDispatch;
    this.queuedSources = [];
  }

  /** Synchronous: later model motion calls cannot rearm; stop/status remain available. */
  public pauseRobotTools() {
    this.toolAdmissionGeneration++;
    this.endAudioInput();
    this.robotToolsPaused = true;
    try { this.onRobotToolsPaused(); } catch {}
    this.nativeReceiptOutput.resetContext();
    this.stopPlayback();
  }

  async connect(apiKey: string, config: AppConfig, videoElement: HTMLVideoElement) {
    if (this.disposed || this.connecting || this.connected) throw new Error('Create a new voice session to reconnect.');
    this.connecting = true;
    this.apiKey = apiKey;
    const needsMedia = config.enableAudio || (config.enableVideo && config.cameraSource === 'browser');
    try {
      // Avoid spending the permission deadline on a known-hidden document.
      // Text-only and server-camera sessions do not acquire local media.
      if (needsMedia) this.ensureMediaVisible();
      if (!this.receiptAudioCache && supportsQuickReceipts(config)) {
        const controller = new AbortController();
        this.receiptPreloadController = controller;
        this.receiptAudioCache = (await preloadReceiptAudio(config, controller.signal)) ?? undefined;
        this.receiptPreloadController = null;
        this.ensureOpen();
        this.log('info', this.receiptAudioCache ? 'Quick Puck acknowledgments ready.'
          : 'Quick acknowledgments unavailable; using the normal model reply.');
      }
      if (needsMedia) this.ensureMediaVisible(); // The preload may have yielded across a tab switch.
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

  private beginTurnTiming(message: LiveServerMessage): TurnTiming | undefined {
    const content = message.serverContent;
    if (this.disposed || (!content && !message.toolCall?.functionCalls?.length)) return;
    const now = performance.now();
    let turn = this.currentTurnTiming;
    if (!turn) {
      turn = { id: ++this.timingSequence, began: now, audioReleaseSent: this.lastAudioReleaseSent, handlers: 0, tools: 0, expectsNative: false };
      this.currentTurnTiming = turn;
    }
    turn.handlers++;
    if (content?.inputTranscription?.text) turn.inputFirst ??= now;
    if (content?.inputTranscription?.finished === true) turn.inputEnd = now;
    const parts = content?.modelTurn?.parts?.filter(part => !part.thought) ?? [];
    if (content?.outputTranscription?.text || parts.some(part => part.text || part.inlineData?.data)) turn.outputFirst ??= now;
    if (parts.some(part => part.inlineData?.data)) turn.audioFirst ??= now;
    if (content?.generationComplete) turn.generated ??= now;
    if (content?.turnComplete) {
      turn.complete = now; this.currentTurnTiming = null;
    }
    return turn;
  }

  private maybeFinishTurnTiming(turn?: TurnTiming) {
    if (turn && turn.complete !== undefined && turn.handlers === 0) {
      if (!turn.logged) this.finishTurnTiming(turn, 'complete');
      else if (turn.nativeScheduled !== undefined && !turn.playbackLogged) this.finishTurnTiming(turn, 'playback-update');
    }
  }

  private finishTurnTiming(turn: TurnTiming, reason: string) {
    if (turn.logged && (turn.nativeScheduled === undefined || turn.playbackLogged)) return;
    turn.logged = true;
    if (turn.nativeScheduled !== undefined) turn.playbackLogged = true;
    const offset = (value?: number) => value === undefined ? null : Math.round(value - turn.began);
    const gap = (first?: number, last?: number) => first === undefined || last === undefined || last < first ? null : Math.round(last - first);
    const firstEvent = Math.min(turn.outputFirst ?? Infinity, turn.toolFirst ?? Infinity);
    const summary = { turn: turn.id, basis: 'ms from first observed server callback', reason,
      latest_audio_release_sent_at_ms: turn.audioReleaseSent === undefined ? null : Math.round(turn.audioReleaseSent),
      latest_audio_release_to_first_callback_ms: gap(turn.audioReleaseSent, turn.began),
      input_transcript_first_ms: offset(turn.inputFirst), input_transcript_end_ms: offset(turn.inputEnd),
      first_model_output_ms: offset(turn.outputFirst), first_native_chunk_ms: offset(turn.audioFirst),
      tool_entry_ms: offset(turn.toolFirst), tool_reply_ms: offset(turn.toolReply), receipt_ms: offset(turn.receipt),
      tool_count: turn.tools, native_playback_expected: turn.expectsNative, generation_complete_ms: offset(turn.generated), turn_complete_ms: offset(turn.complete), native_playback_scheduled_ms: offset(turn.nativeScheduled),
      observed_input_end_to_model_ms: gap(turn.inputEnd, Number.isFinite(firstEvent) ? firstEvent : undefined),
      native_chunk_to_turn_complete_ms: gap(turn.audioFirst, turn.complete),
      generation_complete_to_playback_ms: gap(turn.generated, turn.nativeScheduled),
      turn_complete_to_playback_ms: gap(turn.complete, turn.nativeScheduled),
      note: 'Input transcript callbacks are not guaranteed to align with model turns or acoustic speech end. Audio release is an SDK send boundary, not acoustic end or guaranteed turn association. Native playback is scheduled, not measured sound. Null means unobserved.' };
    // Optional telemetry must never throw into dispatch or Stop.
    try { this.log('info', `Timing ${JSON.stringify(summary)}`); } catch {}
  }

  private ensureOpen() {
    if (this.disposed) throw new Error('Voice connection cancelled.');
  }

  private ensureMediaVisible() {
    if (typeof document !== 'undefined' && document.visibilityState === 'hidden') {
      throw new MediaVisibilityError();
    }
  }

  private async acquireMedia(constraints: MediaStreamConstraints) {
    this.ensureOpen();
    this.ensureMediaVisible();
    const device = constraints.audio ? 'Microphone' : 'Camera';
    this.log('info', `${device} permission: bring this browser tab forward and allow access. Cancel connection to stop waiting.`);
    // getUserMedia has no AbortSignal and may remain unresolved indefinitely.
    // Bound our wait, and immediately stop any tracks the browser grants late.
    await new Promise<void>((resolve, reject) => {
      let settled = false;
      const finish = (error?: Error) => {
        if (settled) return;
        settled = true;
        clearTimeout(timer);
        if (typeof document !== 'undefined') document.removeEventListener('visibilitychange', onVisibilityChange);
        if (this.cancelMediaAcquisition === cancel) this.cancelMediaAcquisition = null;
        if (error) reject(error); else resolve();
      };
      const cancel = () => finish(new Error('Voice connection cancelled.'));
      const onVisibilityChange = () => {
        if (document.visibilityState === 'hidden') finish(new MediaVisibilityError());
      };
      const timer = setTimeout(() => finish(new Error(`${device} access timed out after 30 seconds. Bring this tab forward, allow access in the browser site controls, then reconnect.`)), 30_000);
      this.cancelMediaAcquisition = cancel;
      // Scoped only to pending acquisition; ordinary tab switches after capture
      // succeeds keep the session open. Late grants still release their tracks.
      if (typeof document !== 'undefined') document.addEventListener('visibilitychange', onVisibilityChange);
      let request: Promise<MediaStream>;
      try { request = navigator.mediaDevices.getUserMedia(constraints); }
      catch (error) { request = Promise.reject(error); }
      request.then(stream => {
        if (settled || this.disposed) {
          stream.getTracks().forEach(track => { try { track.stop(); } catch {} });
          if (!settled) cancel();
          return;
        }
        try {
          // Own each track immediately: a later camera/SDK failure releases the mic.
          if (!this.mediaStream) this.mediaStream = new MediaStream();
          stream.getTracks().forEach(track => {
            if (track.kind === 'audio') track.enabled = false;
            this.mediaStream!.addTrack(track);
          });
          finish();
        } catch {
          stream.getTracks().forEach(track => { try { track.stop(); } catch {} });
          finish(new Error(`${device} capture could not be initialized. Close other capture apps and reconnect.`));
        }
      }, (error: any) => {
        const guidance = error?.name === 'NotAllowedError'
          ? `${device} permission denied. Allow access in the browser site controls and operating-system privacy settings, then reconnect.`
          : error?.name === 'NotFoundError'
          ? `${device} was not found. Connect or select an available device, then reconnect.`
          : error?.name === 'NotReadableError'
          ? `${device} could not open. Close other apps using it, check operating-system access, then reconnect.`
          : `${device} unavailable: ${error?.message || 'capture failed'}. Check browser permissions and reconnect.`;
        finish(new Error(guidance));
      });
    });
    this.ensureOpen();
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
          if (error instanceof MediaVisibilityError) throw error;
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
    const robotTools = offeredTools;
    this.followHandoffToolNames = followHandoffTools(spec?.follow_handoff, offeredTools);
    if (robotTools.length) {
      this.log('info', `Robot offers ${robotTools.length} tools: ${robotTools.map(t => t.name).join(', ')}`);
    } else {
      this.log('error', `Could not get tools from the robot (${spec?.message ?? 'no tools'}); no robot tools are available.`);
    }
    let systemInstruction = [spec?.prompt, config.extraInstructions].filter(Boolean).join('\n\n');
    // Selection is known locally; mounting/calibration and current frame delivery
    // are not. Do not grant body-side authority to a browser/external camera.
    const cameraContext = !config.enableVideo
      ? 'Camera source context: disabled. No camera images are sent by this connection.'
      : config.cameraSource === 'browser'
      ? 'Camera source context: browser. Images are selected from the browser device, not a verified robot-mounted camera. Camera orientation and mirroring are unverified. Do not use browser image position to choose a robot paw; ask which paw unless the person named it.'
      : 'Camera source context: server. Images are selected from the configured robot API camera endpoint. Camera mounting, orientation and mirroring are unverified. Do not infer robot body-side from image position; ask which paw unless the person named it.';
    systemInstruction += '\n' + cameraContext + ' Source selection does not establish a current frame. Describe visible image content when available without claiming calibrated robot-body directions.';

    if (isAudioOutput && this.outputPolicy === 'conversation') {
      systemInstruction += '\nComplete every tool call needed for the user request before acknowledging it; do not end a multi-step request just to speak. Remain silent while planning tools and waiting for their replies. After all needed replies arrive, if the latest response contains operator_acknowledgment.text, speak exactly that text and nothing else, unless its explicitly offered optional_clarification is needed for an unclear paw request. If absent, do not invent an acknowledgment. An explicit stand request uses stand_up by itself; do not add Stop unless the user requests cancellation. A busy-paws joke never replaces the requested handshake: still use the advertised handshake path to request the paw; use only the exact supplied acknowledgment for any joke.';
    }
    if (!isAudioOutput && this.outputPolicy === 'conversation') {
      systemInstruction += '\n\nReturn conversation directly as text. The application screens completed text and speaks it with the selected voice. Do not call a speech tool. Robot actions still require their robot tools.';
    }

    // Capability comes from the existing structured catalog, never model prose.
    this.pawClarificationAvailable = false;
    if (['robot_status', 'play_trick', 'interactive_action'].every(name => robotTools.some(tool => tool.name === name))) {
      try {
        const [tricks, actions]: any[] = await Promise.all([
          this.robotBridge.getAnimations(), this.robotBridge.getInteractiveActions(),
        ]);
        this.ensureOpen();
        this.pawClarificationAvailable = supportsPawClarification(robotTools, tricks, actions);
      } catch { this.ensureOpen(); }
    }
    if (this.pawClarificationAvailable) {
      systemInstruction += '\nA successful status reply may offer operator_acknowledgment.optional_clarification. Choose that exact question only when the person requests a handshake and the paw side is unclear. For an explicitly named paw, continue the requested action instead. For an ordinary status/count question, use the actual status receipt. Never add words or invented state to the offered question.';
    }

    const functionDeclarations = robotTools;
    systemInstruction += this.outputPolicy === 'receipts'
      ? '\n\nThe operator is using receipt-only control. Your free-form speech and text are muted; tool receipts are presented by the application. Continue using tools for requests.'
      : '\n\nConversation and robot control are available together. Use robot tools for actions. The application replaces tool-turn narration with fixed server receipts. On tool-free turns, converse naturally without inventing action acknowledgments or physical outcomes.';

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
      liveConfig.outputAudioTranscription = {};
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
    this.log('info', this.outputPolicy === 'receipts' ? 'Receipt-only control: model narration is muted.' : 'Conversation + control: native speech is buffered until generation finishes; tool turns use server receipts. Speech remains unverified.');
    this.log('info', `Connecting to ${modelName} (Audio Output: ${isAudioOutput ? 'Native' : 'Completed text via configured TTS'})...`);
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
            // Explicit Connect (or the one-use onboard launcher) arms once.
            // The same PCM stream uses provider VAD and browser AEC/NS.
            if (this.voiceInputMode === 'hands_free') this.beginAudioInput();
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
          const stop = this.dispatchedActionIds.size
            ? this.requestAutomaticStop('Voice connection closed after an action request') : null;
          this.reportTransportEnded({ kind: 'closed', code: event.code, reason: event.reason });
          if (stop) await stop;
          this.onDisconnect();
        },
        onerror: (err: ErrorEvent) => {
          if (this.disposed) return;
          const details = err.message || err.error?.message || String(err.error || 'WebSocket error');
          this.log('error', `Session error: ${details}`);
          if (this.voiceInputMode === 'hands_free') {
            this.pauseRobotTools();
            if (this.dispatchedActionIds.size) void this.requestAutomaticStop('Hands-free voice failed after an action request');
          }
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

  /** Deliberate audio transmission; the Live connection and camera stay open. */
  public beginAudioInput(): boolean {
    if (this.voiceInputMode === 'hands_free' && (this.handsFreeDisarmed || this.robotToolsPaused)) return false;
    if (!this.connected || this.disposed || !this.config.enableAudio || this.audioBoundaryFailed
        || !this.inputAudioContext || !this.mediaStream?.getAudioTracks().length) return false;
    if (this.audioInputActive) return true;
    // A fresh deliberate turn owns the output immediately; do not wait for a
    // provider interruption before cancelling older receipt decodes/replies.
    this.stopPlayback(false, false);
    this.conversationOutput.discardPending();
    this.nativeReceiptOutput.discardPending();
    const generation = ++this.audioInputGeneration;
    this.audioInputActive = true;
    this.mediaStream.getAudioTracks().forEach(track => { track.enabled = true; });
    try { this.onAudioInputChange(true); } catch { /* Optional UI. */ }
    if (this.inputAudioContext.state === 'suspended') {
      void this.inputAudioContext.resume().catch(() => {
        if (generation === this.audioInputGeneration) this.endAudioInput();
      });
    }
    this.trace(this.voiceInputMode === 'hands_free' ? 'audio_hands_free_start' : 'audio_hold_start');
    return true;
  }

  /** Close the local gate before any pending SDK callback can send more PCM. */
  public endAudioInput() {
    // Stop, pause, blur/hidden, settings or connection loss cannot silently
    // rearm an open microphone. A fresh explicit connection is required.
    if (this.voiceInputMode === 'hands_free') this.handsFreeDisarmed = true;
    const wasActive = this.audioInputActive;
    this.audioInputActive = false;
    ++this.audioInputGeneration;
    this.mediaStream?.getAudioTracks().forEach(track => { track.enabled = false; });
    try { this.onAudioInputChange(false); } catch { /* Optional UI. */ }
    if (!wasActive || !this.connected || this.disposed || !this.sessionPromise) return;
    const pending = this.sessionPromise;
    // Default automatic VAD remains enabled. Flush its cached input when a hold
    // ends; a later audio message reopens the stream. Promise callbacks preserve
    // release-before-repress ordering; never key this boundary to the new hold.
    // https://ai.google.dev/gemini-api/docs/live-api/capabilities#automatic-vad
    void pending.then(session => {
      if (!this.connected || this.disposed || this.sessionPromise !== pending) return;
      try {
        session.sendRealtimeInput({ audioStreamEnd: true });
        this.lastAudioReleaseSent = performance.now();
        this.trace('audio_hold_end_sent');
      } catch { this.failAudioBoundary(); }
    }).catch(() => this.failAudioBoundary());
  }

  private failAudioBoundary() {
    if (this.disposed) return;
    this.audioBoundaryFailed = true;
    this.audioInputActive = false;
    ++this.audioInputGeneration;
    this.mediaStream?.getAudioTracks().forEach(track => { track.enabled = false; });
    try { this.onAudioInputChange(false); } catch { /* Optional UI. */ }
    try { this.log('error', 'Audio release could not be sent. Reconnect before sending more speech.'); } catch {}
  }

  private setupAudioInput(ctx: AudioContext, stream: MediaStream) {
    const source = ctx.createMediaStreamSource(stream);
    this.inputSource = source;
    // 32 ms nominal at 16 kHz: reduce the partial held-audio tail discarded on
    // immediate release. Never send a later callback to try to recover that tail.
    const bufferFrames = 512;
    this.inputScriptProcessor = ctx.createScriptProcessor(bufferFrames, 1, 1);
    this.trace('audio_input_setup', { sample_rate: ctx.sampleRate ?? null, buffer_frames: bufferFrames, encoded_rate: 16000 });
    
    this.inputScriptProcessor.onaudioprocess = (e) => {
      if (!this.connected || !this.sessionPromise || !this.audioInputActive) return;
      const generation = this.audioInputGeneration;
      const pending = this.sessionPromise;

      const inputData = e.inputBuffer.getChannelData(0);
      const audioSequence = ++this.audioSequence;
      this.trace('audio_capture', { audio_sequence: audioSequence, samples: inputData.length,
        sample_rate: e.inputBuffer.sampleRate ?? ctx.sampleRate ?? null });
      const pcmBlob = createPcmBlob(inputData);
      
      void pending.then(session => {
        if (!this.connected || this.disposed || !this.audioInputActive
            || generation !== this.audioInputGeneration || this.sessionPromise !== pending) return;
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
           if (this.voiceInputMode === 'hands_free') {
             this.pauseRobotTools();
             this.log('error', 'Microphone sending failed. Reconnect before sending more speech.');
             if (this.dispatchedActionIds.size) void this.requestAutomaticStop('Hands-free microphone failed after an action request');
           }
           // Hold-to-talk retains its existing close/error handling.
        }
      }).catch(() => {});
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

  private async playAudioBase64(audioData: string, timing?: TurnTiming, isCurrent: () => boolean = () => true): Promise<void> {
    if (this.outputPolicy !== 'conversation') return;
    const ctx = this.audioContext, analyser = this.analyser, generation = this.playbackGeneration;
    if (!ctx || !analyser || this.disposed || !isCurrent()) return;
    if (ctx.state === 'suspended') await ctx.resume();
    const buffer = await decodeAudioData(audioData, ctx);
    if (this.disposed || generation !== this.playbackGeneration || ctx !== this.audioContext || !isCurrent()) return;
    this.nextStartTime = Math.max(this.nextStartTime, ctx.currentTime);
    const source = ctx.createBufferSource();
    source.buffer = buffer;
    source.connect(analyser);
    source.start(this.nextStartTime);
    if (timing) {
      timing.nativeScheduled ??= performance.now() + Math.max(0, this.nextStartTime - ctx.currentTime) * 1000;
      this.maybeFinishTurnTiming(timing);
    }
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

  /** Only the short code-owned takeover cue waits for playback, never ordinary
   * conversation/TTS streaming. Stop cancels this barrier synchronously. */
  private playFollowHandoffAudio(audio: string, current: () => boolean): Promise<boolean> {
    const ctx = this.audioContext, analyser = this.analyser, generation = this.playbackGeneration;
    if (!ctx || !analyser || ctx.state !== 'running' || !current()) return Promise.resolve(false);
    return new Promise(resolve => {
      let finished = false, source: AudioBufferSourceNode | null = null;
      const valid = () => !this.disposed && ctx === this.audioContext && ctx.state === 'running'
        && generation === this.playbackGeneration && current();
      const finish = (completed: boolean) => {
        if (finished) return;
        finished = true; clearTimeout(timer);
        if (this.cancelFollowHandoffAudio === cancel) this.cancelFollowHandoffAudio = null;
        if (source) {
          source.onended = null;
          const index = this.queuedSources.indexOf(source);
          if (index !== -1) this.queuedSources.splice(index, 1);
          if (!completed) { try { source.stop(); } catch { /* Already ended. */ } }
          try { source.disconnect(); } catch { /* Detached output. */ }
        }
        resolve(completed);
      };
      const cancel = () => finish(false);
      const timer = setTimeout(cancel, 10000);
      this.cancelFollowHandoffAudio = cancel;
      void (async () => {
        const buffer = await decodeAudioData(audio, ctx);
        if (finished || !valid() || !Number.isFinite(buffer.duration) || buffer.duration <= 0 || buffer.duration > 5) { finish(false); return; }
        source = ctx.createBufferSource(); source.buffer = buffer; source.connect(analyser);
        const startAt = Math.max(this.nextStartTime, ctx.currentTime);
        const endsAt = startAt + buffer.duration;
        source.onended = () => finish(valid() && ctx.currentTime >= endsAt - .025);
        this.queuedSources.push(source);
        this.nextStartTime = endsAt;
        source.start(startAt);
        this.trace('follow_handoff_audio_started', { duration_ms: buffer.duration * 1000 });
      })().catch(() => finish(false));
    });
  }

  private async prepareFollowHandoff(current: () => boolean, confirmedResponse?: any): Promise<{ announced: boolean; epoch?: string; refusal?: any }> {
    const refuse = (message: string) => ({ announced: false, refusal: { success: false, not_dispatched: true, message } });
    if (!current()) return refuse('Held motion not sent: follow handoff was superseded.');
    if (confirmedResponse === undefined) this.trace('follow_handoff_prepare_start');
    const response = confirmedResponse ?? await this.robotBridge.prepareFollowHandoff();
    const receipt = confirmedFollowHandoff(response);
    if (!current() || !receipt) return refuse('Held motion not sent: could not confirm that follow mode is disabled.');
    if (!receipt.changed) return { announced: false, epoch: receipt.epoch };
    const audio = this.outputPolicy === 'conversation'
      ? this.receiptAudioCache?.lookup(FOLLOW_HANDOFF_CUE, this.config.modelName, this.config.voiceName || 'Puck', this.config.ttsModelName) : null;
    if (!audio) return refuse('Follow mode is off. Held motion not sent: the handoff voice cue is unavailable.');
    const played = await this.playFollowHandoffAudio(audio, current);
    this.trace('follow_handoff_audio_finished', { completed: played });
    if (!played || !current()) return refuse('Follow mode is off. Held motion not sent: the handoff announcement was interrupted or failed.');
    return { announced: true, epoch: receipt.epoch };
  }

  private async synthesizeAndPlaySpeech(text: string, isCurrent: () => boolean = () => true): Promise<void> {
    if (this.outputPolicy !== 'conversation') return;
    const generation = this.playbackGeneration;
    const cancelled = () => this.disposed || generation !== this.playbackGeneration || !isCurrent();
    try {
      if (!this.ai || cancelled()) return;
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
            if (cancelled()) return;
            const audioData = part.inlineData?.data;
            if (audioData) {
              receivedAudio = true;
              await this.playAudioBase64(audioData, undefined, isCurrent);
            }
          }
        }
      } else {
        const response = await this.ai.models.generateContent(request);
        if (cancelled()) return;
        const parts = response.candidates?.[0]?.content?.parts || [];
        for (const part of parts) {
          if (cancelled()) return;
          const audioData = part.inlineData?.data;
          if (audioData) {
            receivedAudio = true;
            await this.playAudioBase64(audioData, undefined, isCurrent);
          }
        }
      }

      if (cancelled()) return;
      if (!receivedAudio) {
        this.log('error', 'Configured voice unavailable: no audio returned. The screened response remains visible.');
      }
    } catch (err: any) {
      if (cancelled()) return;
      const status = Number.isInteger(err?.status) ? err.status : null;
      this.trace('tts_request_failed', { http_status: status });
      this.log('error', status === 429
        ? 'Speech generation quota reached. The response remains visible; check Gemini API billing/quota before using text-to-speech.'
        : 'Speech synthesis unavailable. The screened response remains visible; no replacement voice was used.');
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
    // Context compression does not remove the provider's connection deadline.
    // Retire explicitly instead of ignoring GoAway until a policy-error close.
    // No session replay/resumption: robot requests must not run a second time.
    if (message.goAway) {
      await this.retireExpiringSession();
      return;
    }
    const timing = this.beginTurnTiming(message);
    this.outputTurnTiming = timing;
    try { await this.handleTimedMessage(message, timing); }
    finally {
      if (timing) { timing.handlers--; this.maybeFinishTurnTiming(timing); }
    }
  }

  private reportTransportEnded(event: VoiceTransportEnded) {
    // This updates presentation only; final onDisconnect retains Stop ownership.
    const reason = event.reason ? this.safeMessage(event.reason).replace(/[\r\n\t]+/g, ' ').slice(0, 240) : undefined;
    try { this.onTransportEnded({ ...event, reason }); } catch { /* UI cannot block Stop/cleanup. */ }
  }

  private async retireExpiringSession() {
    if (this.disposed) return;
    const sentActions = this.dispatchedActionIds.size > 0 || this.pendingModelActions > 0;
    this.log('info', 'Voice session is expiring. Reconnect for a new conversation; earlier requests will not be replayed.');
    this.trace('session_go_away', { sent_actions: sentActions });
    // disconnect() invalidates input/output/tools synchronously and closes even
    // a late SDK session. Its promise never waits for the connection to resolve.
    const closing = this.disconnect();
    const stop = sentActions
      ? this.requestAutomaticStop('Voice session expired after an action request') : null;
    this.reportTransportEnded({ kind: 'goaway' });
    try {
      await closing;
      if (stop) await stop;
    } finally {
      // Keep the existing shared Stop gate until its response settles. This is
      // explicit reconnect, never automatic motion rearming or command retry.
      this.onDisconnect();
    }
  }

  private async handleTimedMessage(message: LiveServerMessage, turnTiming?: TurnTiming) {
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
    // Mark the whole output turn before any tool/Stop await. Concurrent SDK
    // callbacks can close generation or the turn while HTTP is unresolved.
    const hasRobotTool = (message.toolCall?.functionCalls ?? [])
      .some(call => !['speak', 'say', 'narrate'].includes(call.name || ''));
    const calls = message.toolCall?.functionCalls ?? [];
    // Claim synchronously, before the ordinary lane or any HTTP await. The
    // original invocation owns its sole result/response; a repeated ID neither
    // retries HTTP nor replays an old success as a fresh acknowledgment.
    const admittedCalls = calls.filter(call => {
      if (!call.id) return true;
      // Stop alone may reuse an ordinary ID once. All later ordinary uses of
      // that ID remain closed; this is not general ID+tool-name deduplication.
      const claimed = call.name === 'stop' ? this.claimedStopIds : this.claimedToolIds;
      if (claimed.has(call.id)) {
        this.trace('duplicate_tool_call_ignored', { message_sequence: messageSequence,
          request_id: call.id, tool: call.name ?? 'unknown' });
        this.log('info', `Duplicate tool call ignored: ${toolLabel(call.name)}`);
        return false;
      }
      claimed.add(call.id);
      this.claimedToolIds.add(call.id);
      return true;
    });
    // Even a repeated Stop retains its conservative packet/queue precedence,
    // without another Stop HTTP. Observe the raw message to quarantine prose.
    const hasStop = calls.some(call => call.name === 'stop' && !(call.id && this.cancelledToolIds.has(call.id)));
    const deferPacketOutput = !!message.toolCall && !hasStop && this.ordinaryToolTail !== null;
    const invalidatePacketOutput = (deferred = false) => {
      if (!hasRobotTool) return;
      const interrupt = !!content?.interrupted || !!message.toolCallCancellation?.ids?.length || hasStop;
      if (this.nativeReceiptOutput.hasQueuedCue() && !interrupt) {
        this.feedbackGeneration++;
        if (!deferred) this.conversationOutput.discard();
      } else this.stopPlayback(!deferred, false);
    };
    if (!deferPacketOutput) invalidatePacketOutput();
    const receiptTurn = this.outputPolicy === 'conversation'
      ? this.nativeReceiptOutput.observe(message) : null;
    if (this.outputPolicy === 'conversation') {
      this.conversationOutput.observe(message, this.pendingRobotTools > 0, !isAudioOutputModel(this.config.modelName));
    }
    if (message.toolCall) {
      if (hasStop) this.toolAdmissionGeneration++;
      const admissionGeneration = this.toolAdmissionGeneration;
      let releaseOrdinary: (() => void) | undefined;
      let ownTail: Promise<void> | undefined;
      const previous = hasStop ? null : this.ordinaryToolTail;
      if (!hasStop) {
        ownTail = new Promise<void>(resolve => { releaseOrdinary = resolve; });
        this.ordinaryToolTail = ownTail;
      }
      // Reply once per provider packet after its ordered HTTP admissions settle.
      // Local server-confirmed cues remain immediate; Stop flushes independently.
      const responseBatch: { id?: string; build: () => any }[] = [];
      const flushResponses = () => {
        const batch = responseBatch.splice(0);
        const pending = this.sessionPromise;
        if (!batch.length || !this.connected || this.disposed || !pending) return;
        void pending.then(session => {
          if (!this.connected || this.disposed || this.sessionPromise !== pending) return;
          // Cancellation and receipt authorization may change while HTTP or the
          // SDK session is pending. Never freeze an acknowledgment at collection.
          const responses = batch.filter(row => !(row.id && this.cancelledToolIds.has(row.id)))
            .map(row => row.build());
          if (!responses.length) return;
          const traceResponses = (phase: string) => responses.forEach(response => this.trace(phase, {
            message_sequence: messageSequence, request_id: response.id ?? '',
            tool: response.name ?? 'unknown', batch_size: responses.length,
          }));
          try {
            traceResponses('function_response_send_start');
            session.sendToolResponse({ functionResponses: responses.length === 1 ? responses[0] : responses });
            traceResponses('function_response_send_return');
          } catch {
            traceResponses('function_response_send_failed');
            // Delivery is unconfirmed. Never replay an executed tool or receipt.
          }
        }).catch(() => {
          this.trace('function_response_session_unavailable', { message_sequence: messageSequence });
        });
      };
      try {
        if (cancelSentAction) await this.requestAutomaticStop('Live cancelled an already-dispatched action');
        if (previous) await previous;
        if (this.disposed || !this.connected) return;
        // This packet was marked tool-bearing at arrival. Its prior receipt
        // stays current until this lane is acquired; do not create a new output
        // tombstone if generation happened to close while it was waiting.
        if (deferPacketOutput && admissionGeneration === this.toolAdmissionGeneration) invalidatePacketOutput(true);
        if (hasStop) {
          this.endAudioInput();
          if (this.voiceInputMode === 'hands_free') {
            // No new microphone intent exists after disarming. Fence any later
            // provider motion packet without invalidating this Stop receipt.
            this.robotToolsPaused = true;
            try { this.onRobotToolsPaused(); } catch {}
          }
        }
        // Stop is an interrupt, never the tail of a queued sequence. Dispatch it
        // first and reject other actions in that same model batch before HTTP.
        const ordered = hasStop ? [...admittedCalls].sort((a, b) => Number(b.name === 'stop') - Number(a.name === 'stop')) : admittedCalls;
        for (const fc of ordered) {
          if (this.disposed) break;
          if (fc.name === 'stop') this.stopPlayback(false, false); // Packet already invalidated its output turn synchronously.
          const supersededBeforeDispatch = admissionGeneration !== this.toolAdmissionGeneration;
          let feedbackGeneration = this.feedbackGeneration;
          const feedbackRequest = ++this.feedbackRequest;
          if (fc.id && this.cancelledToolIds.has(fc.id)) {
            this.log('info', `Cancelled before dispatch: ${toolLabel(fc.name)}`);
            continue;
          }
          this.log('function', `Request: ${toolLabel(fc.name)}`);
          let result: any = { success: true };
          let receivedServerReply = false;
          let followHandoffAnnounced = false;
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
                const speechText = args?.text || args?.message || args?.content || '';
                result = typeof speechText === 'string' && speechText
                  ? { success: true, buffered: true, message: 'Speech buffered for the completed-turn output check; playback is not confirmed.' }
                  : { success: false, error: 'No text provided to speak' };
                break;
              default:
                // Every other tool is defined and run on the robot (the tools in lab/).
                if (admissionGeneration !== this.toolAdmissionGeneration && fc.name !== 'stop' && fc.name !== 'robot_status') {
                  result = { success: false, not_dispatched: true, message: 'Not dispatched: a Stop superseded this waiting request or model actions were paused.' };
                  break;
                }
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
                if (turnTiming) { turnTiming.toolFirst ??= performance.now(); turnTiming.tools++; }
                this.trace('tool_call_start', timing);
                this.pendingRobotTools++;
                const actionRequest = isModelAction(fc.name);
                if (actionRequest) this.pendingModelActions++;
                try {
                  const stopSerial = this.robotBridge.statusGeneration();
                  const handoffPlaybackGeneration = this.playbackGeneration;
                  const current = () => this.connected && !this.disposed && !this.robotToolsPaused
                    && admissionGeneration === this.toolAdmissionGeneration
                    && handoffPlaybackGeneration === this.playbackGeneration
                    && stopSerial === this.robotBridge.statusGeneration()
                    && !(fc.id && this.cancelledToolIds.has(fc.id));
                  if (this.followHandoffToolNames.has(fc.name) && fc.name !== 'stop') {
                    const prepared = await this.prepareFollowHandoff(current);
                    followHandoffAnnounced = prepared.announced;
                    if (prepared.refusal) result = prepared.refusal;
                    else if (!current()) result = { success: false, not_dispatched: true, message: 'Held motion superseded before dispatch.' };
                    else { this.trace('follow_handoff_motion_dispatch', timing); result = await this.robotBridge.callTool(fc.name, args ?? {}, prepared.epoch); receivedServerReply = true; }
                  } else {
                    result = await this.robotBridge.callTool(fc.name, args ?? {}); receivedServerReply = true;
                    // A model may explicitly request follow(false) before the
                    // next action. The same confirmed cue still owns this lane.
                    if (this.followHandoffToolNames.size && fc.name === 'follow' && args?.enabled === false && result?.success === true) {
                      const prepared = await this.prepareFollowHandoff(current, result);
                      followHandoffAnnounced = prepared.announced;
                      if (prepared.refusal) {
                        result = { success: false, message: 'Follow-disable request returned, but its announcement failed or was superseded. Later motion is cancelled.' };
                        receivedServerReply = false;
                      }
                    }
                  }
                }
                catch (error) { this.trace('tool_call_failed', timing); throw error; }
                finally {
                  if (turnTiming) turnTiming.toolReply ??= performance.now();
                  this.pendingRobotTools--; if (actionRequest) this.pendingModelActions--; if (stopping) this.reportStopPending(false);
                }
                const status = toolResponseStatus(result);
                this.trace('tool_call_resolved', { ...timing, status });

            }
          } catch (e: any) {
            result = { success: false, outcome_unknown: true, error: e.message };
          }

          const currentFeedback = () => !supersededBeforeDispatch && this.connected && !this.disposed && feedbackGeneration === this.feedbackGeneration
            && feedbackRequest === this.feedbackRequest && !(fc.id && this.cancelledToolIds.has(fc.id));
          if (dispatch) {
            const status = toolResponseStatus(result);
            const failedAction = isModelAction(fc.name) && result?.success === false;
            const actionFailureOrUnknown = isModelAction(fc.name) && (result?.success === false || status === 'unknown');
            if (status === 'unknown' || failedAction || (result?.success === false
                && (result?.command_epoch_required === true || fc.name === 'stop'))) {
              const wasCurrent = currentFeedback();
              const firstPause = !this.robotToolsPaused;
              // A known current refusal may keep its exact post-server receipt;
              // it never rearms the model or admits arbitrary tool-turn prose.
              const knownFailureReceipt = wasCurrent && receiptTurn && receivedServerReply && failedAction
                && !hasMalformedOutcomeFlags(result) && !result.outcome_unknown
                && !result.superseded && !result.not_dispatched && this.pendingModelActions === 0;
              if (knownFailureReceipt) {
                this.endAudioInput(); this.robotToolsPaused = true;
                try { this.onRobotToolsPaused(); } catch {}
                this.stopPlayback(true, false);
              } else this.pauseRobotTools();
              if (failedAction && firstPause && this.connected && !this.disposed) this.log('error',
                `Model actions paused after a ${toolLabel(fc.name)} request failed. Reconnect voice to resume; Stop and status remain available.`);
              // SDK callbacks can overlap. Stop's existing shared HTTP epoch
              // fences another already-sent action; no stale request is retried.
              if (actionFailureOrUnknown && this.pendingModelActions > 0 && this.connected && !this.disposed) {
                void this.requestAutomaticStop('An action failed or had an unknown result while another model action was in flight');
              }
              // Keep this current unknown/error receipt, while invalidating older speech.
              if (wasCurrent) feedbackGeneration = this.feedbackGeneration;
            }
            if (currentFeedback()) this.onToolDispatch({ ...dispatch, status, message: operatorFeedback(fc.name, result), timestamp: Date.now(), job: acceptedJobIdentity(result), serverUrl: this.robotBridge.baseUrl });
          }
          const speechAlias = ['speak', 'say', 'narrate'].includes(fc.name || '');
          if (!speechAlias && currentFeedback()) {
            const feedback = operatorFeedback(fc.name, result);
            if (turnTiming) turnTiming.receipt ??= performance.now();
            this.log('receipt', feedback);
          }
          const actualReceipt = receivedServerReply && dispatch && currentFeedback()
            && typeof result?.success === 'boolean' && !hasMalformedOutcomeFlags(result) && !result.outcome_unknown && !result.superseded && !result.not_dispatched
            ? nativeReceiptFeedback(fc.name, result) : null;
          const receiptAuthorization = actualReceipt ? this.nativeReceiptOutput.authorize(receiptTurn, actualReceipt, fc, result) : null;
          const textReceiptModel = !isAudioOutputModel(this.config.modelName)
            && supportsPuckReceiptConsumer(this.config.modelName, this.config.voiceName || 'Puck', this.config.ttsModelName);
          const statusChoice = this.pawClarificationAvailable && fc.name === 'robot_status'
            && result?.success === true && actualReceipt && currentFeedback()
            ? this.nativeReceiptOutput.offerPawClarification(receiptTurn, fc, actualReceipt, currentFeedback,
              textReceiptModel ? text => {
                if (currentFeedback()) void this.synthesizeAndPlaySpeech(text, currentFeedback)
                  .catch(() => this.log('info', 'Receipt speech unavailable; the server receipt remains visible.'));
              } : undefined) : null;
          // The immutable clip is code-owned: its own current actual HTTP reply
          // is sufficient even when a multi-tool model turn cannot be narrated.
          const canUseReceiptAudio = actualReceipt && currentFeedback() && this.outputPolicy === 'conversation'
            && supportsPuckReceiptConsumer(this.config.modelName, this.config.voiceName || 'Puck', this.config.ttsModelName);
          const cacheText = canUseReceiptAudio ? serverReceiptAudioText(fc.name, fc.args, result) : null;
          // Extensions use the same queue contract without a frontend tool-name
          // entry. Known tools retain their stricter typed receipt requirements.
          // The generic cue says only that this server accepted a job;
          // it never speaks the extension's name, arguments or result prose.
          const genericQueuedAcceptance = canUseReceiptAudio && toolLabel(fc.name) === 'robot'
            && isModelAction(fc.name) && acceptedJobIdentity(result);
          // Keep an explicitly composed native receipt (the fourth-handshake joke)
          // intact instead of replacing it with the shorter generic cached cue.
          const composedNativeReceipt = receiptAuthorization && receiptAuthorization !== actualReceipt
            && receiptAuthorization !== cacheText;
          // This turn already heard the confirmed takeover, not another Okay.
          if (followHandoffAnnounced && receiptTurn) this.nativeReceiptOutput.claimQueuedCue(receiptTurn);
          const neutralAudio = (cacheText || genericQueuedAcceptance) && result?.success === true && fc.name !== 'stop'
            ? this.receiptAudioCache?.lookup('Okay.', this.config.modelName, this.config.voiceName || 'Puck', this.config.ttsModelName) : null;
          // Existing cached phrases require their typed submission; generic
          // extensions additionally require a valid server/job identity. "Okay"
          // marks first acceptance, never completion or acceptance of later steps.
          const neutralQueued = !!neutralAudio && !!receiptTurn;
          const firstQueuedCue = neutralQueued && this.nativeReceiptOutput.claimQueuedCue(receiptTurn);
          const queuedCueUsed = this.nativeReceiptOutput.hasQueuedCue(receiptTurn);
          if (queuedCueUsed && result?.success === false && actualReceipt && currentFeedback()) {
            this.stopPlayback(false, false);
            feedbackGeneration = this.feedbackGeneration;
          }
          const cachedAudio = firstQueuedCue ? neutralAudio : neutralQueued ? null
            : cacheText && !composedNativeReceipt
            ? this.receiptAudioCache?.lookup(cacheText, this.config.modelName, this.config.voiceName || 'Puck', this.config.ttsModelName) : null;
          const cachedReceipt = !!cachedAudio;
          if (cachedReceipt) {
            // Neutral cues preserve accepted-handshake bookkeeping while muting
            // native duplicates. Stop/failure clips retain immediate interruption.
            if (!firstQueuedCue) this.nativeReceiptOutput.invalidate(false);
            this.trace('cached_receipt_admitted', { bytes: Math.floor(cachedAudio.length * 3 / 4) });
            void this.playAudioBase64(cachedAudio, turnTiming).catch(() => this.log('info', 'Cached receipt audio unavailable; the server receipt remains visible.'));
          }
          const acceptedFourthJoke = queuedCueUsed && [
            `Left paw command queued. ${HANDSHAKE_JOKE}`, `Right paw command queued. ${HANDSHAKE_JOKE}`,
          ].includes(receiptAuthorization || '');
          if (acceptedFourthJoke && currentFeedback()) {
            // Same existing exact accepted-chain authorization; only the repeated
            // action prefix is omitted after the neutral acceptance cue.
            this.nativeReceiptOutput.offerQueuedCueJoke(receiptTurn, receiptAuthorization!, currentFeedback, () => {
              void this.synthesizeAndPlaySpeech(HANDSHAKE_JOKE, currentFeedback)
                .catch(() => this.log('info', 'Receipt speech unavailable; the server receipt remains visible.'));
            });
          }
          const suppressSuccessNarration = queuedCueUsed && result?.success === true && fc.name !== 'stop';
          // TEXT dialogue cannot supply native receipt audio. Speak only the exact
          // code-owned current authorization, never the model's tool-turn prose.
          // A later request in this same batch invalidates outstanding TTS chunks.
          if (!cachedReceipt && !suppressSuccessNarration && !statusChoice && receiptAuthorization && currentFeedback()
              && !isAudioOutputModel(this.config.modelName)
              && supportsPuckReceiptConsumer(this.config.modelName, this.config.voiceName || 'Puck', this.config.ttsModelName)) {
            void this.synthesizeAndPlaySpeech(receiptAuthorization, currentFeedback)
              .catch(() => this.log('info', 'Receipt speech unavailable; the server receipt remains visible.'));
          }
          responseBatch.push({ id: fc.id, build: () => ({
            id: fc.id,
            name: fc.name,
            response: { result, ...(receiptAuthorization && !cachedReceipt && !suppressSuccessNarration && currentFeedback() ? { operator_acknowledgment: { text: receiptAuthorization, ...(statusChoice?.question ? { optional_clarification: { id: 'paw_side', text: statusChoice.question } } : {}) } } : {}) },
          }) });
          if (fc.name === 'stop') flushResponses();
        }
      } finally {
        flushResponses();
        if (this.ordinaryToolTail === ownTail) this.ordinaryToolTail = null;
        releaseOrdinary?.();
      }
    } else if (cancelSentAction) {
      await this.requestAutomaticStop('Live cancelled an already-dispatched action');
    }

  }

  private async releaseNativeReceipt(audio: string[]) {
    if (this.disposed || !this.connected || this.outputPolicy !== 'conversation') return;
    this.stopPlayback(false, false);
    const generation = this.playbackGeneration;
    this.trace('native_receipt_admitted', { audio_parts: audio.length });
    for (const part of audio) {
      if (this.disposed || !this.connected || generation !== this.playbackGeneration) return;
      await this.playAudioBase64(part);
    }
  }

  private async releaseConversation(output: ConversationOutput, timing?: TurnTiming) {
    if (this.disposed || !this.connected || this.outputPolicy !== 'conversation') return;
    // A receipt-shaped sentence can only speak through the current exact
    // authorization path, including older receipts after later tool turns.
    if ([output.transcript, output.text.join(''), ...output.speech].some(isOperatorReceiptNarration)) {
      this.log('info', 'Unbound receipt narration omitted; the actual server receipt remains visible.'); return;
    }
    if (timing && output.audio.length) timing.expectsNative = true;
    // Replace playback without quarantining the next genuine conversation turn.
    this.stopPlayback(false);
    const generation = this.playbackGeneration;
    const current = () => !this.disposed && this.connected && generation === this.playbackGeneration;
    const text = output.text.join('');
    if (text) this.log('model', text);
    if (output.transcript) this.log('model', `Output transcript: ${output.transcript}`);
    for (const audio of output.audio) {
      if (!current()) return;
      this.trace('output_audio_handle_start');
      await this.playAudioBase64(audio, timing);
      this.trace('output_audio_handle_return');
    }
    // A completed TEXT turn is already screened by ConversationOutputGate.
    // Keep one configured voice; do not synthesize transcripts of native audio,
    // or repeat a response already supplied through a speak/say/narrate alias.
    const speech = output.speech.length ? output.speech : (!isAudioOutputModel(this.config.modelName) && !output.audio.length && text ? [text] : []);
    for (const words of speech) {
      if (!current()) return;
      if (words !== text) this.log('model', words);
      this.trace('tts_handle_start');
      await this.synthesizeAndPlaySpeech(words);
      this.trace('tts_handle_return');
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

  private stopPlayback(invalidateTurn = true, invalidateReceipt = true) {
    this.cancelFollowHandoffAudio?.();
    if (invalidateReceipt) this.nativeReceiptOutput.invalidate();
    if (invalidateTurn) this.conversationOutput.discard();
    this.feedbackGeneration++;
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
    this.receiptPreloadController?.abort();
    this.receiptPreloadController = null;
    this.cancelMediaAcquisition?.();
    this.connected = false;
    this.endAudioInput();
    this.videoGeneration++;
    this.stopPlayback();
    this.conversationOutput.close();
    this.nativeReceiptOutput.close();
    if (this.currentTurnTiming) this.finishTurnTiming(this.currentTurnTiming, 'disconnected-before-boundary');
    this.currentTurnTiming = null; this.outputTurnTiming = undefined;
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
