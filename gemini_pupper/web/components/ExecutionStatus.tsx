import React, { useEffect, useState } from 'react';
import { toolLabel } from '../services/operatorFeedback';
import type RobotBridge from '../services/robotBridge';
import { parseQueueStatus, queueSummary, runtimeSummary, motionCapabilitySummary, matchJobEvidence, type QueueStatus, type ToolDispatch } from '../services/executionStatus';

interface Props {
  robotBridge: RobotBridge;
  lastDispatch: ToolDispatch | null;
  modelActionsPaused: boolean;
  onStop: () => Promise<void>;
  stopMessage: string;
}

export default function ExecutionStatus({ robotBridge, lastDispatch, modelActionsPaused, onStop, stopMessage }: Props) {
  const [report, setReport] = useState<{ status: QueueStatus; received: number; started: number; generation: number; bridge: RobotBridge } | null>(null);
  const [error, setError] = useState('Waiting for server status');
  const [checking, setChecking] = useState(false);
  const [now, setNow] = useState(Date.now());

  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout> | undefined;
    setReport(null);
    setError('Waiting for server status');
    const clock = setInterval(() => setNow(Date.now()), 1000);
    const poll = async () => {
      setChecking(true);
      const started = Date.now();
      const generation = robotBridge.statusGeneration();
      try {
        const result = await robotBridge.getQueueStatus();
        if (!active) return;
        const status = parseQueueStatus(result);
        if (status) {
          setReport({ status, received: Date.now(), started, generation, bridge: robotBridge });
          setNow(Date.now());
          setError('');
        } else {
          setReport(null);
          setError(result?.message || 'Server status unavailable or invalid');
        }
      } catch {
        if (!active) return;
        setReport(null);
        setError('Server status unavailable');
      } finally {
        if (active) {
          setChecking(false);
          // Schedule only after the bounded request finishes; no overlapping polls.
          timer = setTimeout(poll, 1000);
        }
      }
    };
    void poll();
    return () => { active = false; if (timer) clearTimeout(timer); clearInterval(clock); };
  }, [robotBridge]);

  // The previous bridge can still occupy React state until its effect cleans up.
  const currentReport = report?.bridge === robotBridge ? report : null;
  const fresh = !!currentReport && currentReport.generation === robotBridge.statusGeneration()
    && now - currentReport.received >= 0 && now - currentReport.received <= 2500;
  const job = matchJobEvidence(lastDispatch, currentReport?.status ?? null, {
    serverUrl: robotBridge.baseUrl, pollStartedAt: currentReport?.started ?? 0, fresh,
  });

  return (
    <section aria-label="Execution status" className="shrink-0 border-b border-gray-700 bg-gray-950 px-3 py-2 text-xs text-gray-200 flex gap-3 items-start">
      <div className="flex-1 min-w-0" aria-live="polite">
        <div className="font-semibold">Execution status {checking ? '· checking' : ''}</div>
        <div className={error ? 'text-amber-300' : 'text-cyan-200'}>
          {currentReport ? `${fresh ? 'Server report' : 'Stale server report'} ${new Date(currentReport.received).toLocaleTimeString()}: ${queueSummary(currentReport.status)}` : error}
        </div>
        {currentReport && <div className={currentReport.status.runtime === 'fake' ? 'font-bold text-amber-300' : 'text-gray-300'}>{runtimeSummary(currentReport.status)}</div>}
        <div className="text-gray-300">{motionCapabilitySummary(currentReport?.status ?? null, fresh)}</div>
        {currentReport?.status.controller_fault && <div className="text-red-300">Controller fault: {currentReport.status.controller_fault}</div>}
        <div>Latest Gemini request: {lastDispatch ? `${toolLabel(lastDispatch.name)} — ${lastDispatch.status}: ${lastDispatch.message}` : 'none sent in this voice session'}</div>
        {lastDispatch && <div className={job.state === 'unknown' ? 'text-amber-300' : 'text-cyan-200'}>
          Latest Gemini job: {job.text}
        </div>}
        {currentReport?.status.last_outcome && <div>
          Latest server result, not matched to this request: {currentReport.status.last_outcome.name} — {currentReport.status.last_outcome.success ? 'reported success' : 'failed/cancelled'}: {currentReport.status.last_outcome.message}
        </div>}
        {modelActionsPaused && <div className="text-amber-300">Model motion requests paused. Stop/status remain available; reconnect voice to enable motion requests.</div>}
        {stopMessage && <div className="text-amber-200">Stop response: {stopMessage}</div>}
        <div className="text-gray-400">Server feedback is separate from speech. Observe physical stopping; use R3/power if needed.</div>
      </div>
      <button type="button" onClick={() => { void onStop(); }} className="relative z-[60] shrink-0 rounded bg-red-700 hover:bg-red-600 px-5 py-3 font-bold text-white" title="Pause model actions and directly request stop from the server">
        STOP
      </button>
    </section>
  );
}
