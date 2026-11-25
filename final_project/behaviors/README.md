# behaviors (pre-v1 remnant)

Not current work — see [../fc/](../fc/STATUS.md). What remains is the old single-robot walk+kick runner,
command guard, deadline and bridge that the `fc_walk_kick` profile pins (`gemini_pupper/robot/fc_release_profile.py`)
and `foundation/hardware/runtime_identity.py` verifies. Kick nodes, ball detector, movement-trial tools,
launch files, policies and the robot runbook were removed on 2026-11-24; restore them with
`git checkout archive/pre-fc-v1-cleanup -- final_project/behaviors`.
