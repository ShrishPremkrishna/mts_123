

export enum ConnectionState {
  DISCONNECTED = 'DISCONNECTED',
  CONNECTING = 'CONNECTING',
  CONNECTED = 'CONNECTED',
  RECOVERING = 'RECOVERING',
  ERROR = 'ERROR',
}

export interface VoiceTransportEnded {
  kind: 'closed' | 'goaway';
  code?: number;
  reason?: string;
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

export type VoiceInputMode = 'hold_to_talk' | 'hands_free';

export type OutputPolicy = 'receipts' | 'conversation';

export interface AppConfig {
  outputPolicy: OutputPolicy;
  voiceInputMode?: VoiceInputMode; // Reconnect to arm hands-free; unknown/absent means hold-to-talk.
  quickReceipts?: boolean; // Four human-approved Puck clips; reconnect to change.
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