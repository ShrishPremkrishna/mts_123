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

Replay is a **synthetic rehearsal of the selected match and runner code**, not playback
of a real recording. Only external I/O is substituted; nothing in replay opens a camera
device, robot discovery, SSH, Home or a network field publisher. Robot-process
links use local socket pairs only.

- **Field.** The rendered scene and perception pipeline produce each referee frame. The
  real fieldcam `StatePublisher` encodes it and delivers it in memory.
- **Field producer** (stand-in for `fc_session.py field`, one per autonomous team). The
  selected `IngressCore` admits each datagram (measured clock probes, capture age,
  calibration revision). The selected `LiveMatchPlay` then turns the referee directive
  into that robot's frame and attack, defend or reset lease, and tracks the runner's kick
  receipts.
- **Robot** (`replay_robot.py`, one child interpreter per connected team, in place of
  `fc_session.py match --runner-only`). It speaks the same stdin/stdout protocol and runs
  the selected `RunnerSession`, `OperatorDrive`, `GuardedPath`, `CombinedRuntime`,
  `BridgeCore`/`Executor`, `MatchImu`/`BoardReceipt`, the `software_start` Stop caller,
  `robot_api.Robot` and `kick_in_place`. Only that process substitutes ROS and what lies
  beyond it: the controller manager, leg_kick clip playback, Stop gateway latch, guard and
  native-authority reports, board record, IMU, and a walker that forwards each command it
  would execute. A per-team socket pair stands in for the Pi's ROS topics: normalized
  events in, kick receipts and walker commands out.
- **Plant.** A kinematic model moves the rendered robots exactly as commanded and rolls
  the ball. A kick moves the ball only if the synthetic clip reaches contact with the
  ball in a fixed reach. There is no gait, contact, latency or braking model.

This is an **ideal-capture timing comparison**: the selected camera's 0.12 s delay
is not applied. It does not qualify the selected camera/native-entry timing
contract. That timing work remains pending; freshness limits are unchanged.

Tickets, operator deadlines, mode changes, board holds, kick receipts, goal reset, Resume,
End and Stop are therefore the production rules. Stop is terminal: the robot process
reports its synthetic gateway latch (`stop_result`, labelled synthetic) and exits. A team
then needs an explicit reconnect, which starts a new runner. End match returns the runner
to standby and leaves it connected.

**Kickoff readiness** (live and replay) includes what every field play refuses: the ball
and each connected robot must be measured inside the field. Start fails at once with that
reason rather than waiting at "Connecting the field". In replay, **Place ball at centre**
is the operator's hand: it is available between matches, at a goal reset, or whenever the
ball has left the field, and never happens automatically.

The Play screen shows:

- which lease the field play chose for each robot;
- the executor's decision, and its own reset check while it waits for Resume;
- each kick receipt by source: the runner's request versus simulated native receipts.

Replay skips Home, standing and the laptop Stop link: the stand-in starts with the
walker active. None of this tests native code, the body-command guard, physical
standing, kicks, contact or Stop.

The Settings identity comes from the source selection manifest plus the actual checkout
revision; source selection is not a deployment receipt. Live remains the default launch
mode and never falls back to replay. The robot child imports the robot release sources
(`gemini_pupper`, `follow_me_pupper`, `foundation`); the laptop ZIP carries that closure.

## Screens

- **Home**: the three connection actions (Blue, Red, field camera), Settings and a
  one-line kickoff checklist. The header always shows Live or Replay and Stop (Esc).
- **Field**: camera source, calibration, snapshot, corner check and goalposts. Finish
  returns Home.
- **Robot**: each robot's ordered steps: Connect, Home on the stand, Stand with laptop
  Stop, Match controls.
- **Settings**: per-robot autonomous/teleop and onboard ball mode, app and build
  identity, selection limits and the offline software check.
- **Play**: overhead feed, one onboard panel per participant, score, actions, and per
  robot the planner phase and reason, overhead pose and heading, target and error,
  command, input age, admitted frames, field-play lease and kick receipts. Onboard panels state that no
  onboard stream is available; the selected robot software has no preview contract yet.
  Robots whose runner reports no executor telemetry say so instead of showing values.

## Camera and teams

Open camera settings to select the overhead camera, camera source, field dimensions and calibration. Six mapped floor tags provide automatic field calibration. The single-corner option requires a tag of known size, rotation and offsets, measured field dimensions and verification of the opposite corner. Body tags give team identity and heading. The experimental team-colour mode requires coloured team and nose patches and their height above the floor. The bare-body appearance overlay supplies candidate centres only. Use **Run offline check** in camera settings, or run `./pupper-fc --self-test`, to exercise a synthetic field without a camera or Pi.

For live team controls, a separately enrolled Pi must already run the matching Pupper FC session helper at `/home/pi/pupper-fc/bin/fc_session.py` and accept SSH from the laptop with a trusted host key. This ZIP installs the laptop component only. Connect checks the selected team and match-pitch protocol version 1 with a read-only profile request. An incompatible or missing helper leaves the team offline. Hardware testing of Home, standing, physical Stop and Wi-Fi-loss recovery belongs to the matching Pi release.

Use the Robot screen for a trusted SSH alias or Wi-Fi discovery, and Settings to select autonomous play or keyboard teleop per robot before kickoff. Home moves all twelve joints and requires physical support.

On a Pi whose gateway reports `fc_operator_control`, Home finishes on the stand and this window becomes the robot's laptop Stop (`fc_session.py operator --owner`). The window echoes each fresh gateway challenge; the Pi rejects echoes older than 0.30 s and owns the watchdog. **Enable standing** sends the displayed challenge once with `supported: true` after the operator confirms support again. Match controls then attach with the standing owner (`--operator-owner`), which the Pi rechecks. Stop, Esc, hiding or closing the window, or a silent link requests Stop on both the operator link and match controls. A standing receipt reports controller state, not observed posture. Pis without that gateway keep the earlier gamepad standing cue. Physical Stop is independent of the browser.

## Developer checks and packaging

Run `./pupper-fc-check` from the checkout for the laptop lobby, mock robot sessions, and overhead perception tests. These tests use local fixtures. They do not exercise Pi hardware.

Run `./pupper-fc-package-laptop` from the checkout to build `outputs/pupper-fc-release/pupper-fc-laptop.zip` and its adjacent SHA-256 file. The builder fetches and verifies 19 locked wheels on its first run and reuses the verified wheel cache thereafter. It requires the laptop package sources to match the committed revision. Verify from the output directory with `shasum -a 256 -c pupper-fc-laptop.zip.sha256`. The ZIP records its Git revision in `release-source.json`.

## Integration handoff

The desktop source was extracted from `codex/pupper-fc-lobby-perception` at `69d8a73`. It builds on the overhead appearance and team-patch detectors already on main. The laptop owns team selection, camera calibration, referee decisions, score, keyboard input and the coordinator that exchanges messages with enrolled teams. The robot owns gait, Home, balance recovery, motor authority and Stop enforcement. The Pi installer, controller and walker stay on their separate release path.

The laptop test suite exercises Blue alone, Red alone, both teams, rejection of kickoff, operator disconnect, and score/resume against local peers. The synthetic image check measures field calibration and pose on generated scenes. Neither qualifies physical Home, standing, two-robot identity through occlusion, real match performance, or disconnected Stop on a deployed Pi. The November 20 bare-body camera experiment found six labelled centres across eight archived views. It supplies no team identity or signed heading. [Overhead experiment handoff](../handoffs/FC_OVERHEAD_PREVIEW.md)
