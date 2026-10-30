import type { MoveCommand, SystemStatus } from "../types";
import { robotBaseUrl } from "../constants";

// This service talks to the Python FastAPI backend running on the robot
class RobotBridge {
  readonly baseUrl: string;

  constructor(baseUrl: string) {
    this.baseUrl = robotBaseUrl(baseUrl);
  }

  private async request(endpoint: string, method: 'GET' | 'POST', body?: any, returnBlob: boolean = false) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const options: RequestInit = {
        signal: controller.signal,
        method,
        headers: body && method === 'POST' ? { 'Content-Type': 'application/json' } : {},
      };

      if (body && method === 'POST') {
        options.body = JSON.stringify(body);
      }

      const response = await fetch(`${this.baseUrl}${endpoint}`, options);

      if (!response.ok) {
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