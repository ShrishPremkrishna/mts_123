import type { MoveCommand } from '../types';

// Stock walking rejects lower nonzero rates; shorten duration for smaller moves.
export const MANUAL_SPEEDS = [0.2];
export const MANUAL_TURNS = [20];
export const MANUAL_DURATIONS = [0.25, 0.5, 1];
export const MANUAL_DEFAULTS = { speed: 0.2, turn: 20, duration: 0.5 };
export type ManualDirection = 'forward' | 'backward' | 'left' | 'right';

export function manualMove(direction: ManualDirection, settings: typeof MANUAL_DEFAULTS): MoveCommand {
  if (!MANUAL_SPEEDS.includes(settings.speed) || !MANUAL_TURNS.includes(settings.turn)
      || !MANUAL_DURATIONS.includes(settings.duration)) throw new Error('Choose a listed manual movement preset');
  // The HTTP API is right/clockwise-positive; it converts to ROS signs itself.
  switch (direction) {
    case 'forward': return { vx: settings.speed, vy: 0, wz: 0, duration: settings.duration };
    case 'backward': return { vx: -settings.speed, vy: 0, wz: 0, duration: settings.duration };
    case 'left': return { vx: 0, vy: 0, wz: -settings.turn, duration: settings.duration };
    case 'right': return { vx: 0, vy: 0, wz: settings.turn, duration: settings.duration };
    default: throw new Error('Unknown manual direction');
  }
}
