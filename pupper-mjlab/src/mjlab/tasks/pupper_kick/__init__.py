"""Pupper v3 header kick: sprint at a ball and hit it with the head bumper."""

from mjlab.tasks.pupper.rl import PupperOnPolicyRunner
from mjlab.tasks.pupper_kick.kick_env_cfg import pupper_header_kick_env_cfg
from mjlab.tasks.pupper_kick.rl_cfg import pupper_header_kick_ppo_runner_cfg
from mjlab.tasks.registry import register_mjlab_task

register_mjlab_task(
  task_id="Mjlab-HeaderKick-Flat-Pupper-v3",
  env_cfg=pupper_header_kick_env_cfg(),
  play_env_cfg=pupper_header_kick_env_cfg(play=True),
  rl_cfg=pupper_header_kick_ppo_runner_cfg(),
  runner_cls=PupperOnPolicyRunner,
)
