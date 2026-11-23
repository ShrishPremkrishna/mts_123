import { hasMalformedOutcomeFlags } from './operatorFeedback';

export const FOLLOW_HANDOFF_CUE = "I've stopped following.";

/** Explicit server capability, never inferred from a tool name or prompt prose. */
export function followHandoffTools(value: any, offered: readonly { name?: string }[]): Set<string> {
  if (value === undefined) return new Set(); // Older API has no announcement protocol.
  const names = new Set(offered.map(tool => tool.name));
  if (value?.schema !== 'pupper_follow_handoff_v1' || value.prepare_if_active !== true
      || !Array.isArray(value.tools) || value.tools.length > 256
      || value.tools.some((name: unknown) => typeof name !== 'string' || !names.has(name))) {
    throw new Error('Invalid follow handoff capability from robot server.');
  }
  return new Set(value.tools);
}

/** Disabled follower controller, not measured physical stillness. */
export function confirmedFollowHandoff(result: any): { changed: boolean; epoch: string } | null {
  if (result?.success !== true || hasMalformedOutcomeFlags(result) || result.outcome_unknown
      || result.superseded || result.not_dispatched) return null;
  const state = result.details?.following;
  if (state?.schema !== 'pupper_follow_state_v1' || state.enabled !== false || typeof state.changed !== 'boolean'
      || typeof state.server_instance !== 'string' || !/^[a-f0-9]{32}$/.test(state.server_instance)
      || typeof state.command_epoch !== 'string' || !new RegExp('^' + state.server_instance + ':[0-9]+$').test(state.command_epoch)) return null;
  return { changed: state.changed, epoch: state.command_epoch };
}
