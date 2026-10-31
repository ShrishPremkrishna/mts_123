import type { MoveCommand, SystemStatus } from "../types";
import { robotBaseUrl } from "../constants";

interface CommandState {
  commandEpoch: string | null;
  stopSerial: number;
  pendingStops: number;
  stopUnconfirmed: boolean;
}
// Manual controls and Live sessions may own distinct bridges for one server.
// Stop fencing survives bridge replacement/reconnect in this page.
const commandStates = new Map<string, CommandState>();

// This service talks to the Python FastAPI backend running on the robot
class RobotBridge {
  readonly baseUrl: string;
  private readonly state: CommandState;

  constructor(baseUrl: string) {
    // Canonicalize host casing/default ports while retaining API path prefixes.
    this.baseUrl = new URL(robotBaseUrl(baseUrl)).toString().replace(/\/+$/, '');
    const existing = commandStates.get(this.baseUrl);
    this.state = existing ?? { commandEpoch: null, stopSerial: 0, pendingStops: 0, stopUnconfirmed: false };
    if (!existing) commandStates.set(this.baseUrl, this.state);
  }

  private async request(endpoint: string, method: 'GET' | 'POST', body?: any, returnBlob: boolean = false) {
    const path = endpoint.replace(/\/+$/, '');
    const stopping = method === 'POST' && ['/stop/immediate', '/queue/stop', '/tools/stop'].includes(path);
    if (stopping) {
      // Invalidate synchronously, before any HTTP work. No old intent may adopt
      // a newer epoch merely because its health request finishes after Stop.
      const serial = ++this.state.stopSerial;
      this.state.pendingStops += 1;
      this.state.commandEpoch = null;
      this.state.stopUnconfirmed = true;
      try {
        const result = await this.sendRequest(endpoint, method, body, returnBlob);
        if (serial === this.state.stopSerial) {
          this.state.stopUnconfirmed = !(result?.success === true && !result?.outcome_unknown
            && result?.command_epoch_required === true && typeof result?.command_epoch === 'string' && result.command_epoch);
        }
        return result;
      } finally { this.state.pendingStops -= 1; this.state.commandEpoch = null; }
    }
    if (method === 'GET' || path === '/tools/robot_status') {
      // Unrelated late status responses never update the command epoch cache.
      return this.sendRequest(endpoint, method, body, returnBlob);
    }
    const serial = this.state.stopSerial;
    const refused = (message: string) => ({ success: false, outcome_unknown: false, not_dispatched: true, message });
    if (this.state.pendingStops) return refused('Not dispatched: a Stop response is pending.');
    if (this.state.stopUnconfirmed) return refused('Not dispatched: the previous Stop failed or its outcome is unknown. Use Stop again and confirm its response before new motion.');
    let epoch = this.state.commandEpoch;
    if (!epoch) {
      const health = await this.sendRequest('/health', 'GET');
      if (serial !== this.state.stopSerial || this.state.pendingStops) return refused('Not dispatched: Stop occurred while preparing this request.');
      if (health?.command_epoch_required !== true || typeof health.command_epoch !== 'string' || !health.command_epoch) {
        return refused('Not dispatched: server command epoch is unavailable. Check the updated server; no command was retried.');
      }
      // Concurrent preparation cannot replace a different already selected epoch.
      if (this.state.commandEpoch && this.state.commandEpoch !== health.command_epoch) {
        return refused('Not dispatched: command epoch changed while preparing this request.');
      }
      epoch = health.command_epoch;
      this.state.commandEpoch = epoch;
    }
    if (serial !== this.state.stopSerial || this.state.pendingStops) return refused('Not dispatched: Stop occurred before this request was sent.');
    const result = await this.sendRequest(endpoint, method, body, returnBlob, epoch);
    if (serial !== this.state.stopSerial) {
      return { success: false, outcome_unknown: !result?.not_dispatched, superseded: true,
        message: result?.not_dispatched
          ? 'Superseded by Stop; the server rejected this earlier request without dispatch.'
          : 'Superseded by Stop. This earlier request may have executed before Stop; its late response is not a new acceptance.',
        original_response: result };
    }
    if (result?.not_dispatched && result?.command_epoch_required === true) this.state.commandEpoch = null;
    // Never retry a rejected motion or adopt the newer epoch from its response.
    return result;
  }

  private async sendRequest(endpoint: string, method: 'GET' | 'POST', body?: any, returnBlob: boolean = false, epoch?: string) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const options: RequestInit = {
        signal: controller.signal,
        method,
        headers: { ...(body && method === 'POST' ? { 'Content-Type': 'application/json' } : {}),
          ...(epoch ? { 'X-Pupper-Command-Epoch': epoch } : {}) },
      };

      if (body && method === 'POST') {
        options.body = JSON.stringify(body);
      }

      const response = await fetch(`${this.baseUrl}${endpoint}`, options);

      if (!response.ok) {
        if (method === 'POST' && [409, 428].includes(response.status)) {
          const rejected = await response.json().catch(() => null);
          if (rejected?.success === false && rejected?.command_epoch_required === true) {
            return { ...rejected, outcome_unknown: false, not_dispatched: true };
          }
        }
        throw new Error(`robot server returned HTTP ${response.status} for ${endpoint}`);
      }
      
      if (returnBlob) {
        return await response.blob();
      }
      return await response.json();
    } catch (e) {
      // Suppress log spam for polling
      if(endpoint !== '/system/status' && endpoint !== '/camera/image' && endpoint !== '/queue/status') {
         console.error(`[RobotBridge] Error calling ${endpoint}`, e);
      }
      // Aborting a client request does not establish whether a POST ran.
      return { success: false, outcome_unknown: method === 'POST', message: `robot response unavailable: ${(e as Error)?.message || e}` };
    } finally {
      clearTimeout(timeout);
    }
  }

  async getSystemStatus(): Promise<SystemStatus | null> {
    const res = await this.request('/system/status', 'GET');
    if (res && res.success !== false && res.battery && res.cpu && res.services) {
      return res as SystemStatus;
    }
    return null;
  }
  
  async getCameraImage(): Promise<Blob | null> {
      try {
          const blob = await this.request('/camera/image', 'GET', undefined, true);
          if (blob instanceof Blob) {
              return blob;
          }
      } catch (e) {}
      return null;
  }

  async queueActivateWalking() {
    return this.request('/walking/activate', 'POST');
  }

  async queueDeactivate() {
    return this.request('/walking/deactivate', 'POST');
  }

  async queueMove(cmd: MoveCommand) {
    // Map internal MoveCommand to API MoveVelocityRequest
    const body = {
      forward_backward_velocity: cmd.vx,
      right_left_velocity: cmd.vy,
      turning_velocity: cmd.wz,
      duration: cmd.duration
    };
    return this.request('/move/velocity', 'POST', body);
  }

  async moveDirection(heading: number, speed: number, duration: number) {
    return this.request('/move/direction', 'POST', { heading, speed, duration });
  }

  async queueStop() {
    return this.request('/queue/stop', 'POST');
  }

  async queueWait(duration: number) {
    return this.request('/queue/wait', 'POST', { duration });
  }

  async queueReset() {
    return this.request('/queue/reset', 'POST');
  }

  async immediateStop() {
    return this.request('/stop/immediate', 'POST');
  }

  async queueAnimation(name: string) {
    return this.request('/animation', 'POST', { animation_name: name });
  }

  /** Gemini's system prompt and tool declarations (lab/prompt.md, the tools in lab/). */
  async getTools() {
    return this.request('/tools', 'GET');
  }

  /** Run one of Gemini's tools on the robot. Returns {success, message}. */
  async callTool(name: string, args: Record<string, unknown>) {
    return this.request(`/tools/${encodeURIComponent(name)}`, 'POST', args);
  }

  async getLimits() {
    return this.request('/limits', 'GET');
  }

  async getQueueStatus() {
    return this.request('/queue/status', 'GET');
  }

  async interactiveAction(action: string) {
    return this.request('/interactive_action', 'POST', { action });
  }

  async getInteractiveActions() {
    return this.request('/interactive_actions', 'GET');
  }

  async getAnimations() {
    return this.request('/animations', 'GET');
  }

  async setVolume(volume: number) {
    return this.request('/system/volume', 'POST', { volume });
  }

  async checkMode() {
    return this.request('/system/mode', 'GET');
  }

  async setFollowMode(enabled: boolean) {
    const endpoint = enabled ? '/following/activate' : '/following/deactivate';
    return this.request(endpoint, 'POST');
  }

  async analyzeCamera(prompt: string) {
    return this.request('/camera/analyze', 'POST', { prompt });
  }
}

export default RobotBridge;