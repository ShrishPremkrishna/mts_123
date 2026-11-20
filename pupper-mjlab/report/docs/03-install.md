# 3 · Install + build

```bash
./install.sh ~/policies/23pt1499.json
```

The script runs these steps, as in [scripts/install.sh](scripts/install.sh):
1. Validates the policy and copies it to `pupper_gait_deploy/policies/test_policy.json`, which is gitignored. That file is the **Square** slot.
2. Runs `install.py --monorepo ~/pupperv3-monorepo --no-build`. This copies our controller, `config.yaml`, `launch.py`, the shipped policies and `joy_utils` into the monorepo. Any file it overwrites is saved as `*.backup`.
3. Rebuilds only `neural_controller` and `joy_utils`, with `-DBUILD_TESTING=OFF`. This takes about 3 min on a Pi 5.

Why not `./deploy.sh`: at `main` 454d3ef, `pupper_gait_deploy/test/test_neural_controller_safety.cpp` doesn't compile on the Pi because it is missing `#include "rclcpp/executors.hpp"`. That fails the whole package build, and the monorepo `build.sh` still exits 0. Turning tests off avoids it. The host tests still pass:

```bash
cd ~/mts_123 && bash pupper_gait_deploy/test/run_host_test.sh     # 2724 checks, 0 failures on 2026-11-19
```

Switch policy (bumpy ↔ flat): rerun `./install.sh <other.json>`. Restart the launch afterwards.
