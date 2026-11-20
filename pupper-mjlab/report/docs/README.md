# Deploy Opt Lab 2 Part 3 policies to Pupper

A recipe for running the Part 3 policies (bumpy, flat control) on a Pupper v3.
Commands run **on the robot** (`ssh pi@<pupper>`), unless marked *Mac*.

## Quick recipe (already set up)

```bash
cd ~/mts_123/pupper-mjlab/report/docs/scripts
./get_policy.sh bumpy                     # W&B -> ~/policies/23pt1499.json, validated
./check_imu.sh                            # stack stopped: IMU must answer on I2C
./install.sh ~/policies/23pt1499.json     # Square slot + controller, build (~3 min)
./launch.sh                               # home, then gamepad: X baseline, Square = policy
```

E-stop: **R3** (right-stick press). Resume: **Options** or `./start_motion.sh`.

## Files

| Step | Doc | Scripts |
|---|---|---|
| 1 | [01-pi-setup.md](01-pi-setup.md): repo, W&B login, gamepad | — |
| 2 | [02-get-policy.md](02-get-policy.md): download + verify | `get_policy.sh` |
| 3 | [03-install.md](03-install.md): install + build | `install.sh` |
| 4 | [04-run.md](04-run.md): launch, buttons, test order | `check_imu.sh`, `launch.sh`, `start_motion.sh` |
| 5 | [05-measure.md](05-measure.md): record + standing check | `record.sh`, `stand_check.py` |
| — | [06-troubleshooting.md](06-troubleshooting.md) | — |
| — | [07-loaner-log-2026-11-19.md](07-loaner-log-2026-11-19.md): first hardware attempt | — |

## Policies

| Name | W&B run (`photon_mz-stanford-university/mjlab/…`) | sha256 |
|---|---|---|
| bumpy (Part 3 final) | `23pt1499` | `42d4863b1934436ce42c70394b15bdd8c823ff46694321b20c6f9b7b0d702c93` |
| flat control | `13m4vcia` | `5a94a14938b6f28922f9b734c640c8a5cf56609bbda4a3d2ba8a7c3aaee42d90` |

Both run on the stock course controller with no controller changes: kp 5.5, kd 0.25, action_scale 0.75, command clip vx ±1.5 / vy ±0.5 / wz ±2.0 m/s·rad/s.
Gait follows the stick speed: trot forward/back, lift-in-place gait for turns and sidesteps, gallop above 0.5 m/s.
Bumpy's known sim defect: while standing it tucks the left back leg about 0.38 rad and puts 59% of its weight on the left. Sim numbers: [../decisions/2026-11-18-w4-p3-candidate.md](../decisions/2026-11-18-w4-p3-candidate.md).
