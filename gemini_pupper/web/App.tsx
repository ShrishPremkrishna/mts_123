import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import { ConnectionState, AppConfig, LogEntry, SystemStatus } from './types';
import { DEFAULT_CONFIG } from './constants';
import DogFace from './components/DogFace';
import DebugPanel from './components/DebugPanel';
import Settings from './components/Settings';
import { GeminiLiveService } from './services/geminiLive';
import RobotBridge from './services/robotBridge';
import { normalizeOutputPolicy, operatorFeedback } from './services/operatorFeedback';
import ExecutionStatus from './components/ExecutionStatus';
import { takeOnboardBootstrap, consumeOnboardBootstrap } from './services/onboardBootstrap';
import { requestIndependentStop, updateLatestDispatch, type ToolDispatch } from './services/executionStatus';
import type { StatusReport } from './services/acceptedJobs';

// Capture once per page load, outside React StrictMode render retries. The URL
// is scrubbed before any asynchronous bootstrap, media or provider operation.
const onboardStartup = takeOnboardBootstrap();

// Icons
const CogIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M12.22 2h-.44a2 2 0 0 0-2 2v.18a2 2 0 0 1-1 1.73l-.43.25a2 2 0 0 1-2 0l-.15-.08a2 2 0 0 0-2.73.73l-.22.38a2 2 0 0 0 .73 2.73l.15.1a2 2 0 0 1 1 1.72v.51a2 2 0 0 1-1 1.74l-.15.09a2 2 0 0 0-.73 2.73l.22.38a2 2 0 0 0 2.73.73l.15-.08a2 2 0 0 1 2 0l.43.25a2 2 0 0 1 1 1.73V20a2 2 0 0 0 2 2h.44a2 2 0 0 0 2-2v-.18a2 2 0 0 1 1-1.73l.43-.25a2 2 0 0 1 2 0l.15.08a2 2 0 0 0 2.73-.73l.22-.39a2 2 0 0 0-.73-2.73l-.15-.09a2 2 0 0 1-1-1.74v-.47a2 2 0 0 1 1-1.74l.15-.09a2 2 0 0 0 .73-2.73l-.22-.38a2 2 0 0 0-2.73-.73l-.15.08a2 2 0 0 1-2 0l-.43-.25a2 2 0 0 1-1-1.73V4a2 2 0 0 0-2-2z"/><circle cx="12" cy="12" r="3"/></svg>
);
const TerminalIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><polyline points="4 17 10 11 4 5"/><line x1="12" y1="19" x2="20" y2="19"/></svg>
);
const FaceIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><circle cx="12" cy="12" r="10"/><path d="M8 14s1.5 2 4 2 4-2 4-2"/><line x1="9" y1="9" x2="9.01" y2="9"/><line x1="15" y1="9" x2="15.01" y2="9"/></svg>
);
const PowerIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M18.36 6.64a9 9 0 1 1-12.73 0"/><line x1="12" y1="2" x2="12" y2="12"/></svg>
);
const MoonIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"/></svg>
);

const App: React.FC = () => {
  const [view, setView] = useState<'face' | 'debug'>('face');
  const [showSettings, setShowSettings] = useState(false);
  const [connState, setConnState] = useState<ConnectionState>(ConnectionState.DISCONNECTED);
  const [systemStatus, setSystemStatus] = useState<SystemStatus | null>(null);
  
  // Load API Key from localStorage
  const [apiKey, setApiKey] = useState(() => {
    return onboardStartup.present ? '' : localStorage.getItem('pupster_api_key') || '';
  });

  // Load Config from localStorage
  const [config, setConfig] = useState<AppConfig>(() => {
    const saved = localStorage.getItem('pupster_config');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        // The system prompt moved to the robot (lab/prompt.md); drop the old saved one.
        delete parsed.systemInstruction;
        return { ...DEFAULT_CONFIG, ...parsed, outputPolicy: normalizeOutputPolicy(parsed.outputPolicy) };
      } catch (e) {
        console.error("Failed to parse saved config", e);
      }
    }
    return DEFAULT_CONFIG;
  });
  
  // Persist API Key changes
  useEffect(() => {
    if (!onboardStartup.present) localStorage.setItem('pupster_api_key', apiKey);
  }, [apiKey]);

  // Persist Config changes
  useEffect(() => {
    localStorage.setItem('pupster_config', JSON.stringify(config));
  }, [config]);

  // Runtime State
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [audioLevel, setAudioLevel] = useState(0);
  const [audioInputActive, setAudioInputActive] = useState(false);
  const talkOwner = useRef<{ pointer: number } | { key: string } | null>(null);
  const [lastDispatch, setLastDispatch] = useState<ToolDispatch | null>(null);
  const [modelActionsPaused, setModelActionsPaused] = useState(false);
  const [stopMessage, setStopMessage] = useState('');
  const stopSequence = useRef(0);
  const pendingStops = useRef(0);
  const [stopPending, setStopPending] = useState(false);
  const isStopPending = useCallback(() => pendingStops.current > 0, []);
  const beginStop = () => { pendingStops.current += 1; setStopPending(true); };
  const endStop = () => { pendingStops.current -= 1; setStopPending(pendingStops.current > 0); };
  
  // Create a persistent video element
  const [videoElement, setVideoElement] = useState<HTMLVideoElement | null>(null);
  const hiddenVideoContainerRef = useRef<HTMLDivElement>(null);

  // UI Visibility State (for auto-hiding footer)
  const [isUiVisible, setIsUiVisible] = useState(true);
  const idleTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  
  useEffect(() => {
    const v = document.createElement('video');
    v.autoplay = true;
    v.muted = true;
    v.playsInline = true;
    setVideoElement(v);
    return () => {
        v.srcObject = null;
    };
  }, []);

  // When in Face mode, park the video element in the hidden container so it stays active
  useEffect(() => {
    if (view === 'face' && videoElement && hiddenVideoContainerRef.current) {
        hiddenVideoContainerRef.current.appendChild(videoElement);
    }
  }, [view, videoElement]);

  // Activity Monitor for Auto-Hiding UI
  useEffect(() => {
    const resetIdleTimer = () => {
      setIsUiVisible(true);
      if (idleTimerRef.current) clearTimeout(idleTimerRef.current);
      
      // Only set auto-hide timer if we are in face mode
      if (view === 'face') {
        idleTimerRef.current = setTimeout(() => {
          setIsUiVisible(false);
        }, 5000); // 5 seconds
      }
    };

    // Attach listeners
    window.addEventListener('mousemove', resetIdleTimer);
    window.addEventListener('mousedown', resetIdleTimer);
    window.addEventListener('touchstart', resetIdleTimer);
    window.addEventListener('keydown', resetIdleTimer);

    // Initial trigger
    resetIdleTimer();

    return () => {
      window.removeEventListener('mousemove', resetIdleTimer);
      window.removeEventListener('mousedown', resetIdleTimer);
      window.removeEventListener('touchstart', resetIdleTimer);
      window.removeEventListener('keydown', resetIdleTimer);
      if (idleTimerRef.current) clearTimeout(idleTimerRef.current);
    };
  }, [view]);

  const geminiRef = useRef<GeminiLiveService | null>(null);
  const onboardStarted = useRef(false);
  const onboardAllowed = useRef(onboardStartup.present);
  const onboardAbort = useRef<AbortController | null>(null);
  const onboardResult = useRef<ReturnType<typeof consumeOnboardBootstrap> | null>(null);
  const onboardSubscriber = useRef<object | null>(null);
  const [onboardPending, setOnboardPending] = useState(onboardStartup.present);
  const cancelOnboardStart = () => {
    onboardAllowed.current = false;
    onboardAbort.current?.abort();
    onboardAbort.current = null;
    setOnboardPending(false);
  };
  const disconnectingRef = useRef<GeminiLiveService | null>(null);
  const endedTransportRef = useRef<GeminiLiveService | null>(null);
  const releaseTalk = useCallback(() => {
    talkOwner.current = null;
    geminiRef.current?.endAudioInput();
    setAudioInputActive(false);
  }, []);
  useEffect(() => {
    const pointerReleased = (event: PointerEvent) => {
      const owner = talkOwner.current;
      if (owner && 'pointer' in owner && owner.pointer === event.pointerId) releaseTalk();
    };
    const hidden = () => { if (document.visibilityState !== 'visible') releaseTalk(); };
    window.addEventListener('blur', releaseTalk);
    window.addEventListener('pointerup', pointerReleased);
    window.addEventListener('pointercancel', pointerReleased);
    document.addEventListener('visibilitychange', hidden);
    return () => {
      releaseTalk();
      window.removeEventListener('blur', releaseTalk);
      window.removeEventListener('pointerup', pointerReleased);
      window.removeEventListener('pointercancel', pointerReleased);
      document.removeEventListener('visibilitychange', hidden);
    };
  }, []);
  const beginTalk = (owner: { pointer: number } | { key: string }) => {
    if (config.voiceInputMode === 'hands_free' || talkOwner.current || showSettings || isStopPending() || connState !== ConnectionState.CONNECTED) return;
    if (geminiRef.current?.beginAudioInput()) {
      talkOwner.current = owner;
      setAudioInputActive(true);
    }
  };
  // Stable for polling; changing the configured server immediately replaces it.
  const robotBridge = useMemo(() => new RobotBridge(config.robotApiUrl), [config.robotApiUrl]);

  // The existing status poll feeds late job evidence only to the current
  // session that owns that exact bridge; no second polling loop.
  const forwardStatusReport = useCallback((report: StatusReport) => {
    const service = geminiRef.current;
    if (service && service.robotBridge === report.bridge) service.observeQueueStatus(report);
  }, []);
  const observeControllers = useCallback(() => !geminiRef.current?.hasTrackedJobs(), []);

  const addLog = useCallback((entry: LogEntry) => {
    setLogs(prev => [...prev.slice(-99), entry]); // Keep last 100 logs
  }, []);

  // Background System Polling Loop (Slow, for critical battery alerts)
  useEffect(() => {
    const interval = setInterval(async () => {
        const status = await robotBridge.getSystemStatus();
        setSystemStatus(status);
        // Do not log here to avoid spam. DebugPanel will handle its own fast polling.
    }, 60000); // Poll every 60s
    return () => clearInterval(interval);
  }, [addLog, robotBridge]);

  const [connectionError, setConnectionError] = useState('');

  const connectVoice = async (keyToUse = apiKey, configToUse = config) => {
    // Ref is synchronous, so repeated clicks cannot create overlapping sessions.
    if (geminiRef.current || isStopPending()) return;
    if (!keyToUse) {
      addLog({ id: 'err-key', timestamp: Date.now(), type: 'error', message: 'API Key missing' });
      setShowSettings(true);
      return;
    }
    
    if (!videoElement) {
        addLog({ id: 'err-vid', timestamp: Date.now(), type: 'error', message: 'Video initialization failed' });
        return;
    }

    let service: GeminiLiveService | null = null;
    let automaticStopSequence: number | null = null;
    try {
      setConnectionError('');
      endedTransportRef.current = null;
      setConnState(ConnectionState.CONNECTING);
      setLastDispatch(null);
      setLogs(current => current.filter(entry => entry.type !== 'receipt'));
      
      service = new GeminiLiveService(
        configToUse,
        entry => { if (geminiRef.current === service) addLog(entry); },
        level => { if (geminiRef.current === service) setAudioLevel(level); },
        () => {
          if (geminiRef.current !== service || disconnectingRef.current === service) return;
          talkOwner.current = null;
          setAudioInputActive(false);
          geminiRef.current = null;
          setConnState(ConnectionState.DISCONNECTED);
        },
        (entry) => {
          if (geminiRef.current !== service) return;
          setLastDispatch(current => updateLatestDispatch(current, entry));
          if (entry.status === 'unknown') setModelActionsPaused(true);
        },
        message => {
          if (automaticStopSequence === null) automaticStopSequence = ++stopSequence.current;
          if (geminiRef.current !== service || automaticStopSequence !== stopSequence.current) return;
          setModelActionsPaused(true);
          setStopMessage(message);
        },
        pending => {
          // Model-issued and automatic Stops use the same typed lifecycle;
          // an old service must still release its pending request after closing.
          if (pending) beginStop(); else endStop();
        },
        undefined,
        active => {
          if (geminiRef.current !== service) return;
          if (!active) talkOwner.current = null;
          setAudioInputActive(active);
        },
        undefined,
        () => { if (geminiRef.current === service) setModelActionsPaused(true); },
        event => {
          if (geminiRef.current !== service) return;
          endedTransportRef.current = service;
          talkOwner.current = null;
          setAudioInputActive(false);
          setModelActionsPaused(true);
          const detail = event.kind === 'goaway' ? 'Provider requested a new connection.'
            : [event.code ? `code ${event.code}` : '', event.reason || ''].filter(Boolean).join(': ');
          setConnectionError(`Voice connection ended${detail ? ` (${detail})` : ''}. Reconnect starts a fresh conversation; previous requests are not replayed.`);
          // Keep geminiRef until the existing final callback releases ownership.
          setConnState(ConnectionState.RECOVERING);
          setIsUiVisible(true);
        }
      );

      // Store before awaiting so Stop can pause dispatch even while connecting.
      geminiRef.current = service;
      setModelActionsPaused(false);
      await service.connect(keyToUse, configToUse, videoElement);
      if (geminiRef.current === service && endedTransportRef.current !== service) setConnState(ConnectionState.CONNECTED);
    } catch (err: any) {
      // Explicit disconnect owns this session until its independent Stop settles.
      if (geminiRef.current !== service || disconnectingRef.current === service) return;
      geminiRef.current = null;
      setConnectionError(err.message);
      addLog({ id: 'conn-err', timestamp: Date.now(), type: 'error', message: err.message });
      setConnState(ConnectionState.ERROR);
    }
  };

  const handleConnect = async () => {
    if (onboardAllowed.current && !onboardStarted.current || onboardAbort.current) return;
    await connectVoice();
  };

  useEffect(() => {
    if (!videoElement || !onboardStartup.present || !onboardAllowed.current) return;
    const subscriber = {};
    onboardSubscriber.current = subscriber;
    if (!onboardStarted.current) {
      onboardStarted.current = true;
      onboardAbort.current = new AbortController();
      onboardResult.current = onboardStartup.nonce
        ? consumeOnboardBootstrap(onboardStartup.nonce, onboardAbort.current.signal)
        : Promise.reject(new Error('Invalid onboard launch.'));
    }
    const controller = onboardAbort.current;
    const result = onboardResult.current;
    if (!controller || !result) return;
    const current = () => onboardSubscriber.current === subscriber;
    void (async () => {
      try {
        // A replayed effect subscribes to the same one-use request; it never
        // consumes the grant twice or starts a connection from stale cleanup.
        const bootstrap = await result;
        if (!current() || !onboardAllowed.current || controller.signal.aborted) return;
        // apiKey is intentionally never put into config, logs or localStorage.
        const selected = { ...config, ...bootstrap.config };
        setApiKey(bootstrap.apiKey);
        setConfig(selected);
        onboardAbort.current = null;
        onboardAllowed.current = false;
        setOnboardPending(false);
        await connectVoice(bootstrap.apiKey, selected);
      } catch {
        if (current() && !controller.signal.aborted) {
          onboardAllowed.current = false;
          setConnectionError('Onboard setup failed or expired. Start a new onboard session, or enter a key in Settings.');
          setConnState(ConnectionState.ERROR);
        }
      } finally {
        if (current()) {
          if (onboardAbort.current === controller) onboardAbort.current = null;
          setOnboardPending(false);
        }
      }
    })();
    return () => {
      if (current()) onboardSubscriber.current = null;
      // React's immediate cleanup/setup replay (or a replacement video element)
      // keeps the one request alive. A real unmount has no new subscriber and
      // aborts before any late response is allowed to Connect.
      queueMicrotask(() => {
        if (!onboardSubscriber.current && onboardAbort.current === controller) controller.abort();
      });
    };
  }, [videoElement]);

  const handleStop = async () => {
    cancelOnboardStart();
    releaseTalk();
    // This does not wait for a model turn and future model calls remain paused.
    setModelActionsPaused(true);
    beginStop();
    const request = ++stopSequence.current;
    setStopMessage('Request sent; waiting for server response');
    const bridge = geminiRef.current?.robotBridge ?? robotBridge;
    try {
      const result = await requestIndependentStop(geminiRef.current, bridge);
      if (request !== stopSequence.current) return;
      setStopMessage(operatorFeedback('stop', result));
    } catch {
      if (request === stopSequence.current) setStopMessage('No confirmed response. Physical stopping is unknown; use R3/power.');
    } finally {
      endStop();
    }
  };

  const handleDisconnect = async () => {
    cancelOnboardStart();
    releaseTalk();
    const service = geminiRef.current;
    if (service) {
      if (disconnectingRef.current === service) return;
      disconnectingRef.current = service;
      // Pause/stop immediately, and release local media without waiting for HTTP.
      const stop = handleStop();
      await service.disconnect();
      await stop;
      disconnectingRef.current = null;
      if (geminiRef.current !== service) return;
      geminiRef.current = null;
    }
    
    if (videoElement) {
        videoElement.srcObject = null;
    }
    
    setConnState(ConnectionState.DISCONNECTED);
  };

  return (
    <div className="h-screen w-screen bg-black flex flex-col font-sans text-gray-100 overflow-hidden relative">
      
      <div 
        ref={hiddenVideoContainerRef} 
        style={{ position: 'absolute', top: 0, left: 0, width: '1px', height: '1px', opacity: 0, overflow: 'hidden', pointerEvents: 'none' }} 
      />

      <div role="status" className="shrink-0 px-3 py-2 text-xs bg-gray-900 text-amber-200">
        {(geminiRef.current?.outputPolicy ?? normalizeOutputPolicy(config.outputPolicy)) === 'receipts'
          ? 'Receipt-only control — model narration is muted. Feedback describes server responses, not observed motion.'
          : 'Conversation + control — action feedback follows server replies, not observed motion. Conversation and camera descriptions remain unverified.'}
        {logs.filter(entry => entry.type === 'receipt').at(-1)?.message && <div aria-live="polite">
          Latest response: {logs.filter(entry => entry.type === 'receipt').at(-1)?.message}
        </div>}
      </div>
      <ExecutionStatus
        robotBridge={geminiRef.current?.robotBridge ?? robotBridge}
        lastDispatch={lastDispatch}
        modelActionsPaused={modelActionsPaused}
        onStop={handleStop}
        stopMessage={stopMessage}
        onStatusReport={forwardStatusReport}
        observeControllers={observeControllers}
      />

      <div className="shrink-0 flex flex-wrap items-center justify-center gap-3 px-3 py-2 bg-gray-900 border-t border-gray-800">
        {(connState === ConnectionState.DISCONNECTED || connState === ConnectionState.ERROR || connState === ConnectionState.RECOVERING) && (
          <button
            aria-label="Connect voice"
            title="Start a fresh voice conversation. Previous requests will not be replayed."
            disabled={stopPending || showSettings || onboardPending || connState === ConnectionState.RECOVERING}
            onClick={handleConnect}
            className="rounded-full bg-cyan-500 px-4 py-2 font-semibold text-black hover:bg-cyan-400 disabled:opacity-40">
            Connect voice
          </button>
        )}
        {(geminiRef.current?.voiceInputMode ?? config.voiceInputMode) !== 'hands_free' && <button
          aria-label="Hold to talk" aria-pressed={audioInputActive}
          disabled={connState !== ConnectionState.CONNECTED || showSettings || stopPending || !config.enableAudio}
          onPointerDown={event => {
            if (event.button !== 0 || !event.isPrimary) return;
            event.preventDefault();
            beginTalk({ pointer: event.pointerId });
            if (talkOwner.current && 'pointer' in talkOwner.current) {
              try { event.currentTarget.setPointerCapture(event.pointerId); } catch { /* Window release is the fallback. */ }
            }
          }}
          onPointerUp={event => {
            const owner = talkOwner.current;
            if (owner && 'pointer' in owner && owner.pointer === event.pointerId) releaseTalk();
          }}
          onPointerCancel={event => {
            const owner = talkOwner.current;
            if (owner && 'pointer' in owner && owner.pointer === event.pointerId) releaseTalk();
          }}
          onLostPointerCapture={event => {
            const owner = talkOwner.current;
            if (owner && 'pointer' in owner && owner.pointer === event.pointerId) releaseTalk();
          }}
          onKeyDown={event => {
            if (event.key !== ' ' && event.key !== 'Enter') return;
            event.preventDefault();
            if (!event.repeat) beginTalk({ key: event.key });
          }}
          onKeyUp={event => {
            const owner = talkOwner.current;
            if (owner && 'key' in owner && owner.key === event.key) { event.preventDefault(); releaseTalk(); }
          }}
          onBlur={releaseTalk}
          onContextMenu={event => event.preventDefault()}
          className={`touch-none select-none rounded-full px-5 py-2 font-semibold disabled:opacity-40 ${audioInputActive ? 'bg-red-500 text-white' : 'bg-cyan-500 text-black'}`}>
          {audioInputActive ? 'Listening — release to finish' : 'Hold to talk'}
        </button>}
        {(geminiRef.current?.voiceInputMode ?? config.voiceInputMode) === 'hands_free' && <div role="status" data-hands-free className="text-sm text-cyan-200">
          {audioInputActive ? 'Hands-free listening — no wake word. Say Stop to stop and disarm.'
            : connState === ConnectionState.CONNECTED ? 'Hands-free disarmed. Disconnect, then Connect to listen again.'
            : 'Hands-free selected. Connect to start listening.'}
          {connState === ConnectionState.CONNECTED && !audioInputActive && <button
            aria-label="Disconnect disarmed voice" onClick={handleDisconnect} className="ml-2 underline">Disconnect voice</button>}
        </div>}
        {onboardPending && <span role="status" data-onboard-pending>Preparing the one-time onboard voice connection…</span>}
        <span role="status" className="text-xs text-gray-300">
          {audioInputActive ? 'Microphone sending' : 'Microphone not sending'}
        </span>
      </div>

      {connState === ConnectionState.RECOVERING && (
        <p role="status" data-voice-recovery className="shrink-0 px-3 py-2 text-center text-sm text-amber-200">
          Voice disconnected. {stopPending ? 'Waiting for the Stop response before reconnect.' : 'Finishing connection cleanup before reconnect.'}
        </p>
      )}
      {(view === 'debug' || connState === ConnectionState.RECOVERING) && connectionError && (
        <p role="alert" className="shrink-0 px-3 py-2 text-center text-sm text-amber-200">{connectionError}</p>
      )}

      <main 
        className="flex-1 relative overflow-hidden"
      >
        {view === 'face' ? (
          <DogFace 
            isListening={audioInputActive}
            isSpeaking={audioLevel > 0.05} 
            audioLevel={audioLevel}
            systemStatus={systemStatus}
            config={config}
            videoElement={videoElement}
          />
        ) : (
          <DebugPanel 
            logs={logs} 
            videoElement={videoElement}
            robotBridge={geminiRef.current ? (geminiRef.current as any).robotBridge : robotBridge}
            systemStatus={systemStatus}
            onStatusUpdate={setSystemStatus}
            cameraSource={config.cameraSource}
            onStop={handleStop}
            stopPending={stopPending}
            isStopPending={isStopPending}
          />
        )}

        {(connState === ConnectionState.DISCONNECTED || connState === ConnectionState.ERROR) && view === 'face' && (
           <div className="absolute inset-0 flex flex-col gap-4 items-center justify-center bg-black/50 backdrop-blur-sm z-10 pointer-events-auto">
              <button 
                onClick={handleConnect}
                disabled={stopPending || showSettings || onboardPending}
                title="Start a fresh voice conversation. Previous requests will not be replayed."
                className="bg-cyan-500 hover:bg-cyan-400 text-black font-bold py-4 px-8 rounded-full shadow-[0_0_30px_rgba(34,211,238,0.5)] transform hover:scale-105 transition-all text-xl flex items-center gap-2"
              >
                <PowerIcon /> WAKE PUPPER
              </button>
              {connectionError && <p role="alert" className="max-w-lg px-6 text-center text-amber-200">{connectionError}</p>}
           </div>
        )}
        {connState === ConnectionState.CONNECTING && (
          <div role="status" className="absolute bottom-20 left-1/2 -translate-x-1/2 z-20 max-w-lg rounded bg-gray-900/95 px-4 py-3 text-center text-sm text-gray-100">
            Connecting. If permission is pending, bring this tab forward and allow microphone or camera access in the browser site controls. Device permission waits end after 30 seconds.
            <button onClick={handleDisconnect} className="ml-2 underline text-cyan-300">Cancel connection</button>
          </div>
        )}
      </main>

      <footer className={`
        h-16 flex items-center justify-between px-4 z-20 transition-all duration-500 ease-in-out
        ${view === 'face' ? 'absolute bottom-0 left-0 right-0 border-t-0 bg-gray-900/60 backdrop-blur-sm' : 'relative bg-gray-900 border-t border-gray-800'}
        ${view === 'face' && !isUiVisible ? 'translate-y-full opacity-0 pointer-events-none' : 'translate-y-0 opacity-100 pointer-events-auto'}
      `}>
        <div className="flex items-center space-x-3 w-32">
           <div className={`w-3 h-3 rounded-full ${
             connState === ConnectionState.CONNECTED ? 'bg-green-500 shadow-[0_0_10px_#22c55e]' : 
             connState === ConnectionState.CONNECTING ? 'bg-yellow-500 animate-pulse' : 'bg-red-500'
           }`} />
           <span className="text-xs font-mono text-gray-400 uppercase">{connState}</span>
        </div>

        <div className="flex bg-gray-800 rounded-lg p-1">
          <button 
            onClick={() => setView('face')}
            aria-label="Conversation"
            title="Conversation"
            className={`p-2 rounded ${view === 'face' ? 'bg-gray-700 text-cyan-400' : 'text-gray-500'}`}
          >
            <FaceIcon />
          </button>
          <button 
            onClick={() => setView('debug')}
            aria-label="Robot controls"
            title="Robot controls without voice"
            className={`p-2 rounded ${view === 'debug' ? 'bg-gray-700 text-cyan-400' : 'text-gray-500'}`}
          >
            <TerminalIcon />
          </button>
        </div>

        <div className="flex items-center space-x-2 w-32 justify-end">
          {(connState === ConnectionState.CONNECTED || connState === ConnectionState.CONNECTING) ? (
             <button 
               onClick={handleDisconnect}
               title="Sleep / Disconnect"
               className="bg-red-900/50 hover:bg-red-900 text-red-200 p-2 rounded-full border border-red-800 transition-colors"
             >
               <PowerIcon />
             </button>
          ) : (
            <div className="text-gray-500 flex items-center gap-1 mr-2" title="Sleeping">
              <MoonIcon />
              <span className="text-xs font-medium">SLEEPING</span>
            </div>
          )}
          <button 
            onClick={() => {
              cancelOnboardStart();
              releaseTalk();
              setShowSettings(true);
              if (document.fullscreenElement) {
                document.exitFullscreen().catch(e => console.error(e));
              }
            }}
            className="p-2 text-gray-400 hover:text-white"
            title="Settings"
          >
            <CogIcon />
          </button>
        </div>
      </footer>

      {showSettings && (
        <Settings 
          apiKey={apiKey} 
          setApiKey={setApiKey} 
          config={config} 
          setConfig={setConfig} 
          sessionActive={!!geminiRef.current}
          onClose={() => setShowSettings(false)} 
        />
      )}
    </div>
  );
};

export default App;