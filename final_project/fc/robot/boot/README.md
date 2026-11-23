# fc/robot/boot — power-on service (prepared 2026-11-23, NOT installed)

Goal: robot powered on → playing with as few hands-on steps as possible. Installing this changes how the
Pi boots, so it waits for the overseer/operator's approval.

## Recommended flow

| Step | Who | What happens | Phase (`fcctl.sh status`) |
| --- | --- | --- | --- |
| 1 | operator | Robot **on the stand**, power on, gamepad on | `off` |
| 2 | `fc-boot.service` | after boot + `FC_BOOT_DELAY_S` (15 s): `fcctl.sh start <profile> --confirm` = homing, controllers loaded, mux checked. Ends there: no standing, no bridge | `starting` → `homed` |
| 3 | operator | lift the robot from the stand to the floor | `homed` |
| 4 | operator | **Ready** in the UI (`fcctl.sh ready`): software stand + bridge (about 10-15 s; tej_u6 asks for Options) | `standing` → `ready` |

Budget: Pi boot ~30-40 s + 15 s delay + start ≤ 100 s (RUNBOOK step 2 pass criterion), then Ready ~10-15 s (mostly ROS graph queries).
Power on during setup, not at the start of the 2-minute slot. Between matches: `fcctl.sh down --keep-stack`
lies the robot down and keeps it homed, so the next Ready needs no stand and no homing.

Why stop at `homed` instead of standing at boot: standing needs the robot on the floor and the operator
watching, and homing needs it on the stand (below). Something has to move the robot between the two;
a person doing that and pressing Ready is the simplest safe split. `FC_BOOT_MODE=idle` boots without
homing (for days when the robot cannot be powered on on the stand); Ready then reports `needs_confirm`
and homes only with `--confirm`.

## What the service does, exactly

- `fc-boot.service` (template here; `install.sh` writes `/etc/systemd/system/fc-boot.service` with the
  user and home filled in). `Type=simple`, `RemainAfterExit=yes`, `User=pi`, `ExecStart=bash
  ~/fc-v1/robot/fcctl.sh boot`, `ExecStop=bash ~/fc-v1/robot/fcctl.sh stop` (power-off lies the robot down
  through fcctl's normal stop), `KillMode=process` (systemd never SIGKILLs the stack), realtime limits as
  the course `robot.service`, output appended to `~/fc-v1/logs/boot.log`.
- `fcctl.sh boot` reads `~/fc-v1/boot.env` (outside `robot/`, so `stage.sh --delete` keeps it; only
  `FC_BOOT_PROFILE`, `FC_BOOT_MODE` = `home`|`idle`, `FC_BOOT_DELAY_S` = 0-300 are accepted, other lines
  are ignored, nothing is sourced). One attempt, no retry. It skips if a profile is already running, refuses
  at ≥ 80 °C, and records its result in `~/fc-v1/run/boot.json`, which `status` shows as `"boot"`
  (`waiting|starting|homed|idle|skipped|failed`).
- It uses the same `start` as the UI, so every existing gate applies: refuses next to a foreign stack,
  topology must be exactly one mux, `stop` works during homing.
- `fcctl` records the boot id in `run/active`; state left over from an earlier boot reads as `off`, not
  as a crashed stack.

## Install / roll back (on the Pi, after `stage.sh`)

```bash
bash ~/fc-v1/robot/boot/install.sh print   --profile stock --mode home   # review; changes nothing
bash ~/fc-v1/robot/boot/install.sh install --profile stock --mode home --delay 15   # sudo; next power-on
bash ~/fc-v1/robot/boot/install.sh status
bash ~/fc-v1/robot/boot/install.sh uninstall    # roll back: disable + remove the unit (running stack untouched)
```

`install` refuses while the course `robot.service` is enabled or running. It only writes
`/etc/systemd/system/fc-boot.service`, `~/fc-v1/boot.env` and `~/fc-v1/run/fc-boot.service.rendered`, and
runs `systemctl daemon-reload` + `enable` (not `start`). `uninstall` is the complete rollback; to also
forget the settings, `rm ~/fc-v1/boot.env`. Emergency rollback without the script:
`sudo systemctl disable --now fc-boot.service` (this also runs fcctl stop) and remove the unit file.

Before relying on it, check once (read-only) that the unit gets the same realtime limits as an ssh login:
`ssh mts 'ulimit -r -l'` vs. `systemctl show fc-boot.service -p LimitRTPRIO -p LimitMEMLOCK`, and that
Tej's `tej_u6` session (camera, recorder) starts without a login session if that profile is chosen.
Not yet verified on hardware: the first installed boot should be supervised.

## Homing posture: what the stock stack needs (evidence, 2026-11-23)

**Answer: homing needs every leg hanging free, with nothing in any joint's homing path — the stand. A
"lying flat on the floor" start is not supported by evidence and has a silent failure mode. Do not home on
the floor without the acceptance test below. Homing and joint limits are unchanged.**

How homing works (`control_board_hardware_interface.cpp` `do_homing()`, called from `on_activate`; the
same in the Pi's installed baseline URDF `foundation/hardware/floor_stock/baseline-expanded.urdf`):

1. Three stages: knees (`_3`, stage 0), then hip pitch (`_2`, stage 1), then abduction (`_1`, stage 2). Each
   moving joint's target advances at ±1.5 rad/s with kp 5.5, kd 0.2. **Joints not yet in their stage are
   limp (kp = kd = 0).**
2. A joint counts as "at its hard stop" the first time its filtered PD torque reaches **2.0 N·m** (about
   0.36 rad of tracking lag). Its zero is then set unconditionally:
   `zero = measured_position - homed_position` (±0.71 knee, ±0.42 hip, ±1.22 abduction).
3. Then every joint returns to its post-homing pose (0 / 0 / ±0.71, clamped to ±0.61) and is held at
   kp 5.5 until a controller takes over. No timeout or plausibility check (the supervised homing review,
   `foundation/results/supervised-homing-20261029-reviewed.json`, also notes no bounded timeout/cancel).

Consequences for a floor start:

- Any resistance before the real stop — a foot on the floor, the body's weight on a limp neighbouring
  joint, a leg folded under the body — is indistinguishable from the stop. That joint is then calibrated
  wrong by the travel it had left, **silently**: `/joint_states` reads correct in the wrong frame, so the
  software cannot detect it, and the position limits move with the bad zero (they no longer protect the
  real hard stop). The walker would then run on a mis-zeroed leg.
- Lying flat puts body weight on the legs while some joints are limp and others sweep through up to
  ~1.2 rad (abduction). Whether the feet stay clear of the floor in every stage depends on the exact
  resting posture; the source cannot answer that, and no floor-homing run exists in this repo's evidence.
- The driver also requires 3 s of stillness before enabling motors (IMU startup check), which a stand or a
  resting robot satisfies; a hand-held robot may not.
- The course documentation says only that on power-on the legs move until they hit their endstops, then
  return to rest; it does not state a posture (pupper-v3-documentation, "Operation"). The course image
  autostarts `robot.service` (homing at boot) wherever the robot is. That is not evidence that floor
  homing calibrates correctly.

Acceptance test that would make a floor start supported (supervised; not done): pick one exact resting
posture; home 5× on the stand and 5× in that posture. After each, lift the robot onto the stand, make the
legs limp (no leg controller active, e.g. `fcctl.sh rest` after a stand, or stop the stack while keeping
`/joint_states` recording), let them hang still, and record `/joint_states`. A physically repeatable
pose is the external reference: the held post-homing pose is not, because a mis-zeroed joint still
reads its commanded value. Floor-vs-stand difference per joint within ~0.03 rad → floor start accepted for
that posture only; then `FC_BOOT_MODE=home` could run with the robot lying on the field.
