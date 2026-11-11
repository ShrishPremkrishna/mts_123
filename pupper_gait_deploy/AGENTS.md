# pupper_gait_deploy

This package deploys trained gait policies to the Pupper. Shared principles are in the [root AGENTS.md](../AGENTS.md).

- `src/neural_controller.cpp`: controller that runs the policy.
- `policies/`: policy files.
- `launch.py`, `config.yaml`: ROS launch definition and parameters.
- `test/`: host-side checks.

Command effects

- `./deploy.sh` installs, rebuilds and launches.
- `./deploy.sh --no-launch` still installs and rebuilds; it only skips launching.
- `python3 download_policy.py --validate-only FILE` validates an existing policy JSON.
- From the team root, `bash pupper_gait_deploy/test/run_host_test.sh` runs the host checks.

The [dated state record](../foundation/results/interruption-recovery.json) identifies inspected releases and deployed binaries. `install.py` copies source, policies and configuration into its selected platform workspace, defaulting to `/home/pi/pupperv3-monorepo`. For work on the physical robot, use the [hardware checklist](../foundation/HARDWARE_CHECKLIST.md). Physical acceptance is recorded from supervised trials.
