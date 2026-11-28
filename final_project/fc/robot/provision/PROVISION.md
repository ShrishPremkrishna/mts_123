# Second robot: from box to game-ready

Goal: the second Pupper (`red`, tag 11) passes `provision.sh` and its calibration in under an hour.
Written 2026-11-23. mts was offline that day (Tailscale last seen 10 h earlier, LAN unreachable), so the
facts about mts below come from the read-only audit of 2026-11-23 and the monorepo source, not a live check.

## Decision: keep the new robot's own class card (path B). Clone mts only as a fallback (path A)

| Evidence | What it means |
| --- | --- |
| No joint calibration is stored per robot. Every start homes each joint against its hard stop; the `homing_*` and `post_homing_position` values in `pupper_v3_description/description/components.xacro` are the same for every Pupper. The IMU mount is fixed in the URDF. In `~/fc-v1`, only `robot/robot_id`, `robot/link.key` and `run/` (replay state, pidfiles) are robot-specific | Cloning copies no calibration worth keeping. Per-robot calibration lives on the laptop: `fc/config/robots/<id>.json` |
| mts uses about 60 GB of a ~113 GB root filesystem. 49 GB of that is old foundation and FC run data (audit §4). The laptop had ~28 GB free on 2026-11-23 | A raw copy of mts's card needs a card-to-card `dd` with two readers, taking about 45–70 min (estimate, not timed), and mts is off the floor the whole time |
| A cloned card carries mts's Tailscale node key, ssh host keys, machine-id (also used as the DHCP client id), hostname, Codex appliance markers and `~/fc-v1/run` replay state | Each one must be reset before both robots are on the network together. Running `tailscale logout` on the clone would log out mts too |
| mts's `stock` neural_controller has uncommitted changes (+1105/−273 lines, rebuilt 2026-10-12). Policy sha is `ecf3a4ab` | A class card may walk a little differently. The step tests measure that anyway, because motors and legs differ between robots |
| Newer monorepo revisions start their own `cmd_vel_mux` in `launch.py` (the laptop's copy does, line 234). mts's installed launch does not | On such a card, `stock.sh` would start a second mux and `start` would refuse READY. `provision.sh --check` reports this as `FAIL launch.py mux` within a minute |

**Recommendation.** Run `provision.sh --check` on the card the robot arrives with. If its only FAILs are
our files, the gamepad, or `robot_id`, provision it (path B, about 40–50 min including calibration).
Fall back to cloning mts (path A, about 1.5–2 h) only when the card is missing or broken, or
`launch.py mux` fails and no mux-less profile variant exists yet. Writing that variant before the robot
arrives removes the main risk on path B. It would be a `profiles/` change, so it needs the profile owner.

## What to have physically

- **Both paths:** a second DualSense (a controller pairs with one Pi at a time, and each robot needs its own R3 e-stop); a printed tag 11 with a rigid mount that copies blue's geometry (see calibration); a tape measure; the stand; a charged battery; a USB keyboard for the first Wi-Fi join.
- **Path A only:** two USB microSD readers and a blank microSD **larger** than mts's card (cards with the same nominal size differ by a few MB, and a raw copy onto a smaller card cuts off the end of the filesystem). Run `provision.sh --check mts blue blue 10` once to see `root_dev` and `disk_total_gb`, which show whether mts boots from SD and how big the card is. Path A also needs mts powered off for about an hour.

## Path B: the robot's own class card (recommended)

| # | Step | Time |
|---|---|---|
| 1 | Robot on the stand, power on. Join the floor Wi-Fi on the touchscreen or with a keyboard (`nmcli dev wifi connect <SSID> --ask`). Note `hostname -I` | 5–10 min |
| 2 | Laptop: `ssh-copy-id pi@<ip>`. The image default password is in `pupperv3-monorepo/infra/pupper_image_builder/readme.md`, or the previous team may have changed it. Add `Host pupper-red` / `HostName <ip>` / `User pi` to `~/.ssh/config`. Several class robots may all answer to the default `pupper.local`, so use the IP | 3 min |
| 3 | `fc/robot/provision/provision.sh --check pupper-red red red 11` (from `final_project/`). This only reads | 1 min |
| 4 | Fix FAILs. Gamepad: `bash ~/pupperv3-monorepo/robot/utils/pair_ps5_controller.sh` on the Pi (hold PS+Create). `launch.py mux`, `packages` or `ros`: stop and switch to path A, or get the profile variant written | 0–5 min |
| 5 | `fc/robot/provision/provision.sh --apply-os pupper-red red red 11` (the owner approves this run: it changes boot config and the IMU driver). This fills red's empty `host` in `robots.json`, stages `fc/robot` and the link key, writes `robot_id` and `~/fc-v1/identity.json`, then applies the Pi OS items mts has ([below](#pi-os-items---apply-os)): it prints each missing item with what it will change and its undo, and asks you to type the Pi's hostname. Then `ssh pupper-red sudo reboot` and rerun `--check`: every Pi OS item should read `ok`. Set red's `udp_host` in `robots.json` to the LAN IP | 10–15 min (driver build: estimate, not timed) |
| 6 | [RUNBOOK](../../RUNBOOK.md) steps 0–3 for red (`--robots red --udp-host red=<ip>`): status, IMU test, start/stop on the stand, stop paths. Then, robot level on the stand: `~/fc-v1/robot/fcctl.sh imu-level` (red's own accel bias; never copied from mts) | 15 min |
| 7 | Calibration below | 15–20 min |

## Path A: clone mts (fallback)

1. On mts: `~/fc-v1/robot/fcctl.sh stop`, then `sudo poweroff`, and take out the microSD. Keep mts **off** until step 5. The clone answers with mts's Tailscale key, machine-id and host keys.
2. Mac, with both cards in readers: `diskutil list`, and identify both disks by size and name. Then
   `diskutil unmountDisk /dev/diskSRC; diskutil unmountDisk /dev/diskDST; sudo dd if=/dev/rdiskSRC of=/dev/rdiskDST bs=4m status=progress`.
   Check the disk numbers twice: `dd` onto the wrong disk erases it. (45–70 min)
3. Put mts's card back in mts, but leave mts off. Put the clone in the new robot and boot it on the stand. Read `hostname -I` from its screen, then `ssh pi@<that ip>`.
4. `provision.sh --fix-clone-identity pupper-red pi@<ip> red red 11` needs passwordless sudo. It sets the hostname and `/etc/hosts`, creates a new machine-id and new ssh host keys, deletes the copied Tailscale state, clears `~/fc-v1/run`, and leaves a marker so a second run does nothing. Without passwordless sudo, do the same by hand in this order: `sudo hostnamectl set-hostname pupper-red`, edit the `127.0.1.1` line in `/etc/hosts`, `sudo rm /etc/machine-id && sudo systemd-machine-id-setup`, `sudo rm /etc/ssh/ssh_host_* && sudo ssh-keygen -A`, `sudo systemctl stop tailscaled && sudo rm /var/lib/tailscale/tailscaled.state`, `rm -rf ~/fc-v1/run`. Then `sudo reboot`. (10 min)
5. Laptop: run `ssh-keygen -R <old ip>`, then add the `pupper-red` alias. Power mts back on. Optionally run `sudo tailscale up --hostname=pupper-red` on red, which needs your browser login.
6. `provision.sh pupper-red red red 11`, then path B steps 5–7. The report fails `identity` if red still shows mts's ssh host key (compared against the laptop's `known_hosts`; mts is never contacted).

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
| `eye_yaw_offset_deg` | From the tag's rotation on the robot: top edge toward the nose = 0, toward the tail = −180, toward the left side = −90, toward the right side = +90 | RUNBOOK step 0: face east and read yaw ≈ 0°. Then **Step** fwd 0.1/0.2 facing east and again facing west. If `heading_bias_deg` has the same sign in both, add its mean to `eye_yaw_offset_deg`. If the sign flips, the gait is drifting and the mount is fine |
| `act_lag_s` | — | **Step** fwd 0.1/0.2/0.3 and turn ±0.5 (preset `slow`, then `fast`). Set it to the median `latency_s` |
| (sim, global) `sim_gain_vx`, `sim_gain_wz`, `sim_lag_s`, `sim_drift_deg_per_m` | — | From the same steps: `gain_lin`, `gain_wz`, `latency_s`, `yaw_drift_deg_per_m`. The sim has one set for both robots, so use the worse robot's values |

Pass, the same as the RUNBOOK: `latency_s` < 0.35, `gain_lin` repeats within ±10%, and |`heading_bias_deg`| < 5.
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
