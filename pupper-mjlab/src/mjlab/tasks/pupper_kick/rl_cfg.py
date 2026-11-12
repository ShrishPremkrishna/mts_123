"""PPO settings for the header kick (the Pupper walking network, longer horizon)."""

from mjlab.tasks.pupper.rl_cfg import pupper_ppo_runner_cfg


def pupper_header_kick_ppo_runner_cfg():
  cfg = pupper_ppo_runner_cfg()
  cfg.experiment_name = "pupper_header_kick"
  cfg.max_iterations = 3_000
  cfg.save_interval = 100
  # An approach takes 1-2 s (50-100 steps) before the kick pays out; gamma 0.99
  # gives a ~2 s horizon where the walking tasks' 0.97 gives ~0.7 s.
  cfg.algorithm.gamma = 0.99
  # At 0.01 (the walking tasks' value) the action std grew from 1.0 to 2.06 over
  # 3,000 CPU iterations while returns stayed flat: the per-step rewards of this
  # task are small, so the entropy bonus dominated.
  cfg.algorithm.entropy_coef = 0.003
  return cfg
