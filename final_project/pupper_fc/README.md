# pupper_fc (pre-v1 remnant)

Not current work — see [../fc/](../fc/STATUS.md). What remains is the gameplay library that other packages
still import: the `fc_walk_kick` session mode in `gemini_pupper/robot/` (`fc_walk_kick.py`,
`fc_release_profile.py`), `foundation/hardware/fc_walk_kick_release.py` and `runtime_identity.py`, the
remnant in [../behaviors/](../behaviors/README.md), and `overhead_perception/fieldcam/gameplay.py`.
The lobby, replay viewer, camera setup and laptop package were removed on 2026-11-24; restore them with
`git checkout archive/pre-fc-v1-cleanup -- final_project/pupper_fc`.

Tests: `.venv-api/bin/python -m pytest -q final_project/pupper_fc/tests final_project/behaviors` from the repo root.
