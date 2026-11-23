# Pupper FC laptop lobby

Open **Pupper FC.command** from the extracted ZIP on an Apple silicon Mac running macOS 15 or newer with Python 3.12 on its command path. The ZIP includes the pinned Python wheels. Keep the extracted directory together and writable. First launch creates its Python environment there. Later launches reuse it and open the same local lobby. Close its Terminal window to shut down the server.

From a source checkout, run `./pupper-fc`. Source setup downloads dependencies when needed. The existing simulator remains available via `python -m final_project.pupper_fc`.

## Device-free synthetic rehearsal

From the active checkout on this Mac, reuse the verified laptop environment:

```sh
PUPPER_FC_PYTHON=/Users/shrish/.codex/worktrees/fc-teleop/cs123/.venv-fc/bin/python ./pupper-fc --mode replay
```

`PUPPER_FC_PYTHON` selects an existing environment, checks the pinned packages and imports,
and never installs into that environment. Without it, the launcher uses its managed
`.venv-fc` as before. Add `--no-open` to print the local address.

Replay is a **synthetic I/O rehearsal**, not playback of a real recording. Home,
Field, Robot, Settings and Play use the same lobby routes with isolated synthetic
robot/field transports. The overhead image uses the maintained rendered scene and
perception pipeline. No camera device, robot discovery, SSH, Home or network field
publisher is opened. Connect one or two synthetic teams, open the field, start the
rehearsal, then End or Stop. After Stop, explicitly disconnect and reconnect a team
before another browser rehearsal. These acknowledgments do not test the native
executor, physical standing, kicks, contact or Stop. The selected native pair and
physical qualification remain pending. The Settings identity comes from the source
selection manifest plus the actual checkout revision; source selection is not a
deployment receipt. Live remains the default launch mode and never falls back to
replay.

## Camera and teams

Open camera settings to select the overhead camera, camera source, field dimensions and calibration. Six mapped floor tags provide automatic field calibration. The single-corner option requires a tag of known size, rotation and offsets, measured field dimensions and verification of the opposite corner. Body tags give team identity and heading. The experimental team-colour mode requires coloured team and nose patches and their height above the floor. The bare-body appearance overlay supplies candidate centres only. Use **Run offline check** in camera settings, or run `./pupper-fc --self-test`, to exercise a synthetic field without a camera or Pi.

For live team controls, a separately enrolled Pi must already run the matching Pupper FC session helper at `/home/pi/pupper-fc/bin/fc_session.py` and accept SSH from the laptop with a trusted host key. This ZIP installs the laptop component only. Connect checks the selected team and match-pitch protocol version 1 with a read-only profile request. An incompatible or missing helper leaves the team offline. Hardware testing of Home, standing, physical Stop and Wi-Fi-loss recovery belongs to the matching Pi release.

Use team setup for a trusted SSH alias or use Wi-Fi discovery. Select autonomous play or keyboard teleop per team. Home moves all twelve joints and requires physical support. This lobby's standing flow requires the Pi's Stop gamepad and a fresh standing cue. In a match, Escape requests Stop through the connected sessions. Physical Stop is independent of the browser.

## Developer checks and packaging

Run `./pupper-fc-check` from the checkout for the laptop lobby, mock robot sessions, and overhead perception tests. These tests use local fixtures. They do not exercise Pi hardware.

Run `./pupper-fc-package-laptop` from the checkout to build `outputs/pupper-fc-release/pupper-fc-laptop.zip` and its adjacent SHA-256 file. The builder fetches and verifies 19 locked wheels on its first run and reuses the verified wheel cache thereafter. It requires the laptop package sources to match the committed revision. Verify from the output directory with `shasum -a 256 -c pupper-fc-laptop.zip.sha256`. The ZIP records its Git revision in `release-source.json`.

## Integration handoff

The desktop source was extracted from `codex/pupper-fc-lobby-perception` at `69d8a73`. It builds on the overhead appearance and team-patch detectors already on main. The laptop owns team selection, camera calibration, referee decisions, score, keyboard input and the coordinator that exchanges messages with enrolled teams. The robot owns gait, Home, balance recovery, motor authority and Stop enforcement. The Pi installer, controller and walker stay on their separate release path.

The laptop test suite exercises Blue alone, Red alone, both teams, rejection of kickoff, operator disconnect, and score/resume against local peers. The synthetic image check measures field calibration and pose on generated scenes. Neither qualifies physical Home, standing, two-robot identity through occlusion, real match performance, or disconnected Stop on a deployed Pi. The November 20 bare-body camera experiment found six labelled centres across eight archived views. It supplies no team identity or signed heading. [Overhead experiment handoff](../handoffs/FC_OVERHEAD_PREVIEW.md)
