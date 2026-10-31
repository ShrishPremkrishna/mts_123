

export enum ConnectionState {
  DISCONNECTED = 'DISCONNECTED',
  CONNECTING = 'CONNECTING',
  CONNECTED = 'CONNECTED',
  ERROR = 'ERROR',
}

export enum RobotMode {
  IDLE = 'IDLE',
  WALKING = 'WALKING',
  ANIMATING = 'ANIMATING',
}

export interface LogEntry {
  id: string;
  timestamp: number;
  type: 'info' | 'error' | 'function' | 'user' | 'model' | 'receipt';
  message: string;
}

export type EyeTrackingSource = 'mouse' | 'server' | 'mediapipe';
export type CameraSource = 'browser' | 'server';

export type OutputPolicy = 'receipts' | 'conversation';

export interface AppConfig {
  outputPolicy: OutputPolicy;
  modelName: string;
  ttsModelName?: string;
  voiceName: string;
  extraInstructions: string; // added after the robot's prompt (lab/prompt.md)
  enableVideo: boolean;
  enableAudio: boolean;
  robotApiUrl: string; // URL for the FastAPI bridge
  eyeTrackingSource: EyeTrackingSource;
  cameraSource: CameraSource;
  enableAffectiveDialog: boolean;
  enableProactiveAudio: boolean;
}

export interface MoveCommand {
  vx: number; // forward/back
  vy: number; // left/right
  wz: number; // turn
  duration: number;
}

export interface SystemStatus {
  battery: {
    percentage: number | null;
    is_charging: boolean;
    voltage: number | null;
  };
  cpu: {
    usage: number | null;
    temperature: number | null;
  };
  services: {
    robot: 'active' | 'inactive' | 'unknown' | 'simulated';
    internet: 'online' | 'offline' | 'unknown' | 'not_checked';
  };
}