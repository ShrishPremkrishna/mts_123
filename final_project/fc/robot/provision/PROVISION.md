# Second robot: from box to game-ready

Goal: the second Pupper (`red`, tag 11) passes `provision.sh` and its calibration in under an hour.

**State (Nov 28, device-free):** red was made by cloning mts (path A below). It boots (Pi 5), answers `ssh pupper-red`
(LAN 10.5.6.37, Tailscale node `pupper-red` 100.92.219.105), and `provision.sh --check pupper-red red red 11` gives 20 pass,
1 fail (no paired DualSense), all Pi OS items ok incl. `imu_v4`; I2C shows the IMU (0x4B) and ADC (0x48). `robots.json`
red: host `pupper-red`, `udp_host` 10.5.6.37. Next: the [RUNBOOK robot 2 block](../../RUNBOOK.md#robot-2-red-first-motion--shrish-present)
(DualSense, `imu-level`, tag board + `robotcal red`, step tests, first motion with Shrish present).

## Why cloning is safe

No joint calibration is stored per robot: every start homes each joint against its hard stop, and the homing values in
`pupper_v3_description/description/components.xacro` are the same for every Pupper. In `~/fc-v1` only `robot/robot_id`,
`robot/link.key` and `run/` (IMU cal, replay state, pidfiles) are robot-specific. Per-robot calibration lives on the
laptop (`fc/config/robots/<id>.json`). What a clone must not keep: hostname, machine-id (also the DHCP client id), ssh
host keys, Tailscale state, `robot_id`, and `~/fc-v1/run` (the IMU accel bias belongs to mts's chip).

## Path A: clone mts (used Nov 28)

mts boots from a 128 GB SanDisk USB drive, so the clone goes to a second USB drive plugged into **mts** (mts stays up).
`clone_live.sh` (runs on mts as root) copies mts's filesystems at file level (a live copy; Nov 28: 25.6 GB in 13 min),
leaving out old foundation/FC run data and `~/fc-v1/run` (the IMU cal), then resets the identity offline before the drive
ever boots (new partition ids, hostname + `/etc/hosts`, empty machine-id, new ssh host keys, no Tailscale state,
`robot_id`, no `identity.json`) and verifies it by remounting read-only. It picks the target only by serial and refuses
the boot drive or any mounted disk. `--print-excludes` lists the excludes; `--exclude` / `--excludes-file` change them.

1. Plug in the blank drive and read its serial: `ssh mts lsblk -dno NAME,SERIAL,MODEL,SIZE`.
2. Plan (changes nothing, no root): `ssh mts 'bash -s -- --dry-run --serial <SERIAL> --hostname pupper-red --robot-id red' < fc/robot/provision/clone_live.sh`
3. Clone: `scp fc/robot/provision/clone_live.sh mts:/tmp/` then
   `ssh mts 'sudo nohup bash /tmp/clone_live.sh --yes --serial <SERIAL> --hostname pupper-red --robot-id red >/dev/null 2>&1 &'`. Poll with
   `ssh mts 'cat /var/log/fc-clone-pupper-red.log.done 2>/dev/null || tail -c 300 /var/log/fc-clone-pupper-red.log'`:
   `.done` holds the exit code (0 cloned + verified, 1 failed, 2 refused). Not `pgrep -f`: it matches the ssh command itself.
4. Move the drive to robot 2 and boot it on the stand. It joins the saved Wi-Fi networks ([RUNBOOK Wi-Fi](../../RUNBOOK.md#wi-fi-in-a-new-place-no-touchscreen)).
   Laptop: add `Host pupper-red` to `~/.ssh/config`. Optional: `sudo tailscale up --hostname=pupper-red` (browser login).
5. `fc/robot/provision/provision.sh --check pupper-red red red 11`, set red's `host` / `udp_host` in `robots.json`, then
   `stage.sh` + `sha_check.sh` and the per-robot steps below.

Tests: `fc/robot/tests/test_clone_live.py` (guards and plan against fake `lsblk`/`findmnt`/`df`/`du`; no disk touched).
Checked on mts (2026-11-28, read-only `bash -s`): refuses mts's own drive, an unknown serial, mts's own hostname/robot id;
with a fake second disk the plan estimates a 26.7 GB copy. The erase/copy/verify path has not run since it became a script
(Nov 28's run was the same steps by hand).
For a card cloned another way (e.g. `dd`), `provision.sh --fix-clone-identity pupper-red pi@<ip> red red 11` does the
identity reset on the booted clone (needs passwordless sudo; keep mts off until it has run, since the clone answers with mts's keys).

## Path B: the robot's own class card (alternative; not run on a real card)

| # | Step | Time |
|---|---|---|
| 1 | Robot on the stand, power on. Join Wi-Fi with a keyboard (`nmcli dev wifi connect <SSID> --ask`). Note `hostname -I` | 5–10 min |
| 2 | Laptop: `ssh-copy-id pi@<ip>` (image default password: `pupperv3-monorepo/infra/pupper_image_builder/readme.md`). Add a `Host` alias with the IP; several class robots may answer to `pupper.local` | 3 min |
| 3 | `fc/robot/provision/provision.sh --check <host> red red 11` (read-only) | 1 min |
| 4 | Fix FAILs. Gamepad: `bash ~/pupperv3-monorepo/robot/utils/pair_ps5_controller.sh` on the Pi. `launch.py mux` (newer monorepo launches its own mux), `packages` or `ros`: use path A instead | 0–5 min |
| 5 | `provision.sh --apply-os <host> red red 11` (owner approves: boot config and IMU driver). Stages `fc/robot` and the link key, writes `robot_id`, applies the [Pi OS items](#pi-os-items---apply-os) with undo lines, then reboot and rerun `--check` | 10–15 min (estimate) |
| 6 | Per-robot steps below | 30 min |

## provision.sh

`provision.sh [--check] [--apply-os] [--fix-clone-identity NEW_HOSTNAME] [--yes] <ssh-host> <robot-id> <team> <tag>`

- There is no default host. Before any ssh, it refuses when the arguments disagree with `robots.json`, or when the host, tag or team already belongs to another robot. For example, `provision.sh mts red red 11` exits 2.
- `--check` runs `probe.sh` over ssh. The probe is read-only: no files, no git index refresh, no `.pyc`, no device opened, no ROS node. The report checks jazzy, the monorepo install and stock packages, `config.yaml`, the `launch.py` mux and teleop arg, the policy sha against mts, Python ≥ 3.10, rclpy, disk ≥ 2 GB, motor SPI and IMU I2C device nodes, a paired DualSense, temperature, running stacks, `robot_id`, `link.key` mode, staged-file shas against this laptop, `identity.json`, and clone signs. FAIL means `fcctl.sh start` cannot work. After the report it lists the Pi OS items (`os_items.sh check`, read-only) as ok/missing; they do not change READY or the exit code. Exit codes: 0 ready, 1 not ready, 2 refused, 3 unreachable.
- A full run writes only under `~/fc-v1/` on the Pi, plus red's `host` in `robots.json` on the laptop. Only `--fix-clone-identity` and `--apply-os` change system files, and each asks first (`--yes` skips the question).
- Tests: `fc/robot/tests/test_provision.py`. These cover the config guards, the probe rules and the Pi OS items, and run end to end against a fake `ssh` whose Pi is a temp home. They check that `--check` writes nothing, that a full run writes only `fc-v1/`, that a second run re-sends no files, that `--apply-os` turns a fresh-card `config.txt` into mts's state (backup kept, `imu_cal.json` untouched), that a second apply changes nothing, and that it refuses without sudo or while a stack runs.

## Pi OS items (`--apply-os`)

`provision/os_items.sh` runs on the Pi (`check` and `plan` are read-only; `apply` changes only missing items). The source is [fc/robot/README.md](../README.md#pi-os-changes-on-mts-2026-11-24-operator-approved-for-heat), where each change on mts is recorded. A fresh class card has the stock state: `infra/pupper_image_builder/provision_pios_base.sh` adds the three touch overlays, and `pupper-rs/install_service.sh` links and enables `pupper-gui`.

| Item | ok when | apply | Undo (one line) |
| --- | --- | --- | --- |
| `hailo` | `config.txt` has `dtparam=pciex1=off` | append it in a marked `# --- Pupper FC heat reduction` block (`[all]`) | `sudo sed -i '/^# --- Pupper FC heat reduction/,/^# --- end Pupper FC ---$/{/^dtparam=pciex1=off$/d}' /boot/firmware/config.txt && sudo reboot` |
| `fan` | mts's 4 `dtparam=fan_temp*` lines (full speed from 55–60 °C) | the same block | `sudo sed -i '/^# --- Pupper FC heat reduction/,/^# --- end Pupper FC ---$/{/^dtparam=fan_temp[0-3]=/d}' /boot/firmware/config.txt && sudo reboot` |
| `gui` | `pupper-gui` not enabled, not running | `sudo systemctl disable --now pupper-gui` | `sudo systemctl link /home/pi/pupperv3-monorepo/pupper-rs/pupper-gui.service && sudo systemctl enable --now pupper-gui` |
| `touch` | no active `dtoverlay=waveshare-4dpic-*` (GT911 off the IMU's I2C-1 bus) | prefix those lines with `#fc-off(<date> i2c touch) ` | `sudo sed -i 's/^#fc-off([^)]*i2c touch) //' /boot/firmware/config.txt && sudo reboot` |
| `imu_v4` | the stock driver `.so` is a link to `~/fc-v1/build/imu-filter-v4/...` and that build contains `FC_IMU_CAL` | stack stopped: copy this card's `control_board_hardware_interface` source, apply the staged `imu/stock-imu-filter.patch` (refuses if it does not apply, or differs from the laptop's), colcon build + `test_fc*` tests (same recipe as on mts), then move the old `.so` to `.so.stock-link` and link v4 | `~/fc-v1/robot/fcctl.sh stop; cd ~/pupperv3-monorepo/ros2_ws/install/control_board_hardware_interface/lib && rm libcontrol_board_hardware_interface.so && mv libcontrol_board_hardware_interface.so.stock-link libcontrol_board_hardware_interface.so` |

- The `config.txt` items need passwordless sudo and a reboot. `apply` first copies `config.txt` to `config.txt.bak-pre-fc-os-<date>`. `imu_v4` needs no sudo.
- `imu_cal.json` is never copied or touched. The accel bias belongs to one chip, so each robot runs `fcctl.sh imu-level` itself (`--check` shows `info imu_cal`).
- The v4 build on red will have a different sha256 from mts's (`f849f254…`), so the check looks for the `FC_IMU_CAL` marker instead of a sha.
- Verified on a fake Pi (tests) and with a read-only `--check mts blue blue 10` on 2026-11-27: all five items read `ok` on mts, and the v4 sha is `f849f2547da2`. `apply` has not yet run on a real card.

## Per-robot calibration (red, ~20 min; repeat for blue if its tag is ever remounted)

**Easiest: copy blue's tag mount exactly.** Use the same height, the same forward offset, and the tag's
top edge toward the tail. Then red needs no `eye_*` overrides. This also matters because the eye currently uses one mount for all
tags until `app.py`/`eye.py` are wired (below). With the robot **standing** (gamepad X), measure:

| Knob (`fc/config/robots/red.json`) | Measure | Check in the UI |
| --- | --- | --- |
| `eye_tag_height` (m) | Floor to the tag's top surface | Put the robot on a taped mark at a far corner. The pose shown should be within 3 cm of the mark. Height error shows up as an error that grows toward the edge of the frame |
| `eye_tag_offset_x` (m) | Along the body axis, from the tag centre to the body centre (midpoint of the front and rear hips). Positive means the tag sits ahead of the body centre. Centre the tag left-to-right within 1 cm, because there is no lateral knob | **Step** turn ±0.5. The reported `dist_m` should be under 3 cm for a ~90° turn. Offset error ≈ `dist_m / (2·sin(yaw_deg/2))` |
| `eye_yaw_offset_deg` | From the tag's rotation on the robot: top edge toward the nose = 0, toward the tail = −180, toward the left side = −90, toward the right side = +90 | Face east and read yaw ≈ 0°. Then **Step** fwd 0.1/0.2 facing east and again facing west. If `heading_bias_deg` has the same sign in both, add its mean to `eye_yaw_offset_deg`. If the sign flips, the gait is drifting and the mount is fine |
| `act_lag_s` | — | **Step** fwd 0.1/0.2/0.3 and turn ±0.5 (preset `slow`, then `play`). Set it to the median `latency_s` |
| (sim, global) `sim_gain_vx`, `sim_gain_wz`, `sim_lag_s`, `sim_drift_deg_per_m` | — | From the same steps: `gain_lin`, `gain_wz`, `latency_s`, `yaw_drift_deg_per_m`. The sim has one set for both robots, so use the worse robot's values |

Pass: `latency_s` < 0.35, `gain_lin` repeats within ±10%, and |`heading_bias_deg`| < 5.
To record a value, move each measured key from `_to_measure` to the top level of `red.json` and fill
`_measured` (date, who). Then run `python -m pytest -q fc/tests/test_core_robot_params.py`, which rejects
unknown keys and wrong types, and commit.

## Per-robot params: how the app should load them (not wired yet)

`fc/core.py` provides `load_robot_overrides(robot_id, root=ROBOTS_DIR, known=None)` and
`robot_params(base, robot_id)`. A missing file gives `{}`. Keys starting with `_` are notes. With `known`,
any unknown key or wrong value type raises `ValueError`. `params.json` stays the shared base, so nothing
changes until these functions are called. The wiring in `app.py` is for the overseer:

```python
from .core import load_robot_overrides
# __init__, right after self.base_params: read once, fail fast at startup
self.robot_over = {r.id: load_robot_overrides(r.id, known=self.params) for r in self.all_robots}
def params_for(self, rid): return {**self.params, **self.robot_over.get(rid, {})}   # presets < robot
# _decide: params = self.params_for(r.id) for brain.decide and shape(); record self.robot_over in run meta.
```

The eye also needs per-tag mounts. Pass `robot_over` to `Eye`, and in `Eye.process` merge
`self.robot_over.get(rid, {})` into the params before building `mount` for that robot, both in
`process` and in `cfg["robots"]` in `__init__`. Until then, `eye_*` overrides are ignored, so copy blue's
mount. Overrides win over presets and over live UI edits of the same key, so the UI should show each
robot's overridden keys.
