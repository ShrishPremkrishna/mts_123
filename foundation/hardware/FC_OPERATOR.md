# Pupper FC operator commands

Open **Pupper FC.command** on the laptop. For each cold, enrolled robot, open its team setup and click **Prepare**. Support that robot on its stand, confirm support and click **Home**. When the Pi reports that homing is complete, **Enable standing** sends a fresh standing cue through the same terminal used by Home. Gamepad Options also works. The dashboard connects the match runner after the Pi confirms an active walker. A prepared, standing robot can use **Connect** directly.

The terminal equivalents below use `mts` as the robot’s SSH host.

```sh
ssh mts python3 /home/pi/pupper-fc/bin/fc_session.py prepare
ssh -t mts python3 /home/pi/pupper-fc/bin/fc_session.py home --supported
```

**Prepare** pins the installed FC release and existing native platform and reuses the current run on this boot. It opens no motor, ROS or camera processes. **Home** starts one supported hardware session, the onboard camera, the headless detector and the recorder. The terminal standing cue is Enter after homing, or Options on the gamepad. The gateway requires a connected gamepad for software Start. Keep Pupper supported through that cue. A repeated home checks the original manager and live hardware state. The standing cue is skipped while the walker is already active.

For controller-free homing, choose **Finish homed · no gamepad** before clicking Home, or run `home --supported --home-only`. It initializes the supported hardware once and queries the board state. The command requests no walker activation. Later `home --supported` reuses that home and accepts Enter to stand while the connected gamepad provides the independent Stop path. Preparing and homing do not wait for joystick messages.

Closing setup revokes its pending software standing cue. If mechanical homing has started, keep the robot on its stand while the Pi finishes its readiness checks. The physical gamepad Stop remains available during setup.

**Start match** is the third action. In the lobby, connect each team and the live overhead camera once. Check that the ball and each team's body tag are visible. Choose autonomous or keyboard play per team and click **Start match**. After a normal **End match** the walker stays standing and the connections stay open. Click **Start match** again. After a goal, replace the ball, let the autonomous players reach home, then click **Resume play**.

**Escape** requests hard Stop on the connected teams. During a native kick or another active job, End match also takes the hard Stop path. On disconnect, the affected runner gets EOF and stops. The lobby ends the match for the other participants. Support the robot before using `home --supported` to stand again after a hard Stop. A healthy driver reuses the existing calibration. A hardware-interface fault requires separate recovery.

For a diagnostic terminal trial with a separately supplied field stream, `ssh -t mts python3 /home/pi/pupper-fc/bin/fc_session.py match` prints an arm command for that attempt. Use this standalone runner on its own. Its default duration is 60 seconds. The `--one-shot` option selects the original single-kick trial.

The laptop console uses `ssh -T mts python3 /home/pi/pupper-fc/bin/fc_session.py match --runner-only --robot-id pupper-1`. This opens the selected operator-controls runner without enabling walking or starting a field producer. It preserves the runner's JSON input and output. Configure `pupper-2` on the second Pi. Each ROS graph must use localhost-only discovery. Only SSH and the camera's UDP stream cross the network. The command checks the local manager's identity and process environment before attaching. Physical two-Pupper isolation and control still require a field check.

## Laptop camera and field inputs

The Pi sends its team identity, measured field dimensions, attack directions, UDP port and socket addresses over the runner's existing SSH connection. The lobby keeps that profile for the life of the connection, including subsequent kickoffs. Older runners use one separate `describe --robot-id ID` query. Its field process uses a separate connection:

```sh
ssh -T mts python3 /home/pi/pupper-fc/bin/fc_session.py field --robot-id pupper-1 --epoch GAME --source-session CAMERA --revision 1 --tracks '{"striker":"pupper-1","keeper":"pupper-2"}'
```

Keep standard input open. The process exits on EOF or termination and releases its UDP port. By default the laptop source host is the SSH client, and the field service runs for up to one hour. Each Pi uses its local boot clock and the same camera session, team map and match epoch. The first JSON event, `source_open`, confirms the socket and publisher opened. The periodic `latest.paired` and `latest.reason` show whether a current frame produced an intent. A defender can produce a guard intent. Missing or coasting participant tracks withdraw field authority until observations recover. A matched native kick edge pins the attacking role until the walker returns.

The laptop opens a pending kickoff across both Pis. Only fresh synchronized ball and participant observations set `latest.ready_to_arm`. While pending, the field publishes observations with zero intents, even if one Pi has accepted its arm. Once every Pi accepts, the referee commits and issues tactics on the next capture. The camera referee adjudicates full-ball crossings between calibrated posts and latches each goal. For a new rally, return the ball to the field and wait for the robots to reach their home poses. The laptop waits for every autonomous runner to accept Resume, then releases the shared referee for all teams. The previous rally cannot start another kick during a partial reset.

## Enroll or roll back a Pi

Use the Pi tarball’s `READ-ME.md` for the one-time board build on an existing FC platform. The enrollment below assumes that fixed driver is already selected in the installed environment. Pass `--imu-prefix /absolute/path/to/native/install/control_board_hardware_interface` when adding it to a previous platform. A new enrollment requires per-report IMU publication in both strict and recovery modes. Use the other ID on the second robot.

```sh
python3 foundation/hardware/fc_walk_kick_release.py enroll --robot-id pupper-1
```

Enrollment stages and verifies this source, the stock walker, both leg-kick clips, the headless detector and the canonical mounted field. It verifies that the Pi's ROS overlay actually resolves the target controller, board driver and software Stop gateway, imports the ROS/Hailo Python dependencies without opening them, and checks the installed detector model digest. It selects one profile and creates the stable `/home/pi/pupper-fc/bin/fc_session.py` entry. Avahi advertisement is attempted once using noninteractive sudo. If the account cannot install the announcement, Connect still accepts an SSH host configured on the laptop.

A previously qualified Pi supplies its latest completed FC platform selection. On a new Pi use the installed FC ROS environment, or pass `--platform-env /path/to/environment.sh`. The stock coursework image needs the target AArch64 deadline controller, compatible board driver, updated Start/Stop gateway and Hailo model installed first. Enrollment reports any missing component. Use `--detector-model FILE` when the approved HEF is outside the known paths. A different venue can use `--field-config FILE` for measured geometry and attack directions. The default is the recorded November 20 field with blue attacking west and red east. The laptop can correct goal-post locations in camera settings.

Normal play records clip requests, kick lifecycle changes and gateway/native Stop requests in MCAP. Add `--recording full` to enrollment for complete joint, IMU, guard, native authority and field streams, the onboard detector timing log and the normalized per-frame source trace. The normal match omits the ROS joint-state broadcaster and its dynamic transforms. Both recording modes keep the IMU broadcaster for the operator client. The native walker and kick controller read joint and IMU hardware interfaces directly.

The default board profile is strict. For a separately qualified new driver, add `--imu-prefix /path/to/control_board_hardware_interface --imu-recovery supported_rehome` to the same enrollment command. All enrolled libraries have recorded path and SHA-256 identities. A healthy warm session never silently changes these selections.

To restore the preceding enrollment, first end the physical native session with the ordinary supported shutdown. Then run:

```sh
python3 foundation/hardware/fc_walk_kick_release.py rollback
```

This checks the previous staged release's manifest, atomically selects the earlier settings/release and refreshes its discovery announcement. On a first installation with no preceding selection, rollback disables the managed profile and requests removal of its advertisement. It leaves the saved settings and staged build available on disk. No enrollment action starts a motor or ROS graph.

Keep the transport receiving the laptop operator input open for the match. To end the physical session with Pupper supported, use the existing `persistent_session.py shutdown --run-dir RUN` from the selected release. The next prepare creates a new run after verified shutdown. If startup or cleanup failed, inspect the existing receipt before restarting. Reconnecting to a stopped session never issues a motor command.

## Recover a diagnosed IMU fault

The default profile uses `"imu_recovery": "strict"`. A separately qualified target driver with the FC recovery contract and an enrolled `supported_rehome` profile can attempt one supported full rehome in its original native process. With the field source and match stopped, Stop visibly latched, and Pupper on the stand, use:

```sh
ssh -t mts python3 /home/pi/pupper-fc/bin/fc_session.py home --supported --recover-imu
```

The command checks the mapped board library, new fault in this run, 60 unavailable motor handles, inactive leg owners and Stop state, then requests the board's supported full homing transition. After activation it verifies all command handles remain unclaimed and restores only the selected read-only state broadcasters if needed. Standing still uses a fresh operator cue. A timeout leaves a consumed recovery marker so a retry cannot repeat a motor transition with an unknown result. This route has resource-manager/fake-device and software evidence. The Pi controller manager and physical homing remain unqualified until a supervised check.
