# Pupper FC operator commands

Click **Connect** on the laptop to prepare Blue or Red. Support the robot with all feet clear for **Home**. Check support again and select **Enable standing**. Keep the team's browser tab visible. A current FC installation accepts this cue without a gamepad. Once the cameras and teams are ready, click **Start match**. A normal **End match** keeps a healthy walker standing for another kickoff.

The Pi issues fresh challenges to the visible browser. If replies stop, its native gateway revokes standing and requests damping and controller deactivation. The lease lasts 750 ms from issue on the Pi. Scheduling and controller cleanup add time. **Stop all** and **Escape** explicitly request Stop. Reconnecting alone cannot activate a walker. Support the robot, reconnect, and give a new standing cue. A healthy board can reuse its existing Home. Keep the physical power control within reach during supported qualification.

The Pi starts with its original Pupper eyes and touch Wi-Fi controls. The laptop runs the game. Course motion services stay idle at boot.

The Pi archive supplies `./pupper-fc-install blue` (or `red`) for initial appliance installation on a prepared FC platform, and `sudo python3 foundation/hardware/fc_appliance.py normal` to restore its previous boot target on the next boot.


Open **Pupper FC.command** on the laptop. Select **Home only** to finish setup with the walking controller inactive. With the controller-free gateway selected, the dashboard opens a separate operator connection for **Enable standing** and keeps it open through match setup and play. Changing away from the tab releases standing authority.

The terminal equivalents below use `mts` as the robot’s SSH host.

```sh
ssh mts python3 /home/pi/pupper-fc/bin/fc_session.py prepare
ssh -t mts python3 /home/pi/pupper-fc/bin/fc_session.py home --supported
```

**Prepare** pins the installed FC release and existing native platform and reuses the current run on this boot. It opens no motor, ROS or camera processes. **Home** starts one supported hardware session and the recorder. With event recording, the first request to stand for a match or open play starts the onboard camera and detector after Home succeeds. Full recording starts them before Home to capture the whole session. The terminal standing cue is Enter after homing, or Options on the gamepad. The terminal Enter path requires a connected Stop gamepad. The FC dashboard has its own Pi-enforced operator connection. Keep Pupper supported through that cue. A repeated home checks the original manager and live hardware state. The standing cue is skipped while the walker is already active.

For controller-free homing, choose **Finish homed · no gamepad** before clicking Home, or run `home --supported --home-only`. It initializes the supported hardware once and queries the board state. The command requests no walker activation. Later `home --supported` reuses that home and accepts Enter to stand while the connected gamepad provides the independent Stop path. Preparing and homing do not wait for joystick messages.

If the board refuses its IMU check before motor enable, the setup screen shows the native reason while the Pi closes the processes from that attempt. Once setup confirms their exit, select **Prepare** for a fresh run, keep Pupper supported and still, and explicitly select **Home** again. If setup reports unconfirmed cleanup, inspect the retained run and complete the existing owned shutdown procedure first.

Closing setup revokes its pending software standing cue. If mechanical homing has started, keep the robot on its stand while the Pi finishes its readiness checks. A connected physical gamepad can also request Stop.

**Start match** is the third action. In the lobby, connect each team and the live overhead camera once. Check that the ball and each team's body tag are visible. Choose autonomous or keyboard play per team and click **Start match**. After a normal **End match** the walker stays standing and the connections stay open. Click **Start match** again. After a goal, replace the ball, let the autonomous players reach home, then click **Resume play**.

**Escape** requests hard Stop on the connected teams. During a native kick or another active job, End match also takes the hard Stop path. On disconnect, the affected runner gets EOF and stops. The lobby ends the match for the other participants. Support the robot before giving a fresh **Enable standing** cue after a hard Stop. A healthy driver reuses the existing calibration. A hardware-interface fault requires separate recovery.

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

## Match pause after loss of balance feedback

Select the candidate with `./pupper-fc-install blue --field-imu-hold` or `red` during installation. It uses the ordinary FC walker. A newly observed report fault revokes the active match for the roster. An eligible robot continues closed-loop balance at zero requested velocity while the other player pauses. An interrupted native kick takes Stop. Watch the robot and keep the overhead camera connected. When the lobby confirms both players and the camera are ready, click **Resume after balance hold**. This preserves the score and Home. A goal already awaiting ball replacement still needs the ordinary goal-reset operation.

On the stock BNO08x, Game Rotation Vector and bias-corrected uncalibrated gyro can bridge the loss of the primary reports only while both alternates remain fresh and agree with recent primary measurements. The initial agreement requirement is 500 ms. The 3 s ceiling starts at the older primary measurement and includes detection and requalification. A stopped chip or bus supplies neither alternate. On a terminal board fault, support Pupper, end the match, shut down the selected native session and run Connect and supported Home again. The connected Stop gamepad still supplies the independent stop during standing.

For the first hardware check, put one Pupper in a load-bearing sling with feet touching and a spotter at Stop. Verify Home and stand, start a one-player match and observe the report cadence before asking for motion. Exercise a delivery-only dropout first, then a primary-only fault with an instrumented report feed. Confirm zero requested motion, remaining torso support, unchanged board and Home IDs, fresh post-resume intent, and immediate physical Stop. Use a separate supported trial for common-sensor loss. Floor walking, contact, braking distance and balance in a kick require physical qualification.

On a prepared Blue Pi with target alias `mts`, start a one-robot keyboard match in the sling and release the keys. From the laptop run:

```sh
ssh -t mts python3 /home/pi/pupper-fc/bin/fc_session.py test-imu-delivery --supported
```

The probe suspends the read-only ROS IMU broadcaster for two seconds and verifies restored delivery. Native balance sensing continues. Check that the lobby pauses, then use **Resume after balance hold** and one fresh key press. Verify Stop while the sling carries the body. Testing true report loss on a powered robot is a separate instrumented trial.

## Supported bench damping

For a supervised bench experiment, rebuild the bundled board package against the Pi's installed ROS environment and use the ordinary enrollment command with `--imu-prefix "$PWD/native/install/control_board_hardware_interface" --imu-recovery supported_damping`. Enrollment and Prepare are motor-inactive. Keep the torso supported and the legs clear for the entire trial. `home --supported --home-only` homes and leaves the bench controller inactive. A connected Stop gamepad and a fresh supported cue through `home --supported` enable light joint damping. The legs can swing. The dashboard and Pi report `match_capable=false` for this profile.

A short IMU loss keeps the drives in the same damping mode. Three fresh pairs restore balance telemetry. Three seconds without qualification, invalid joint feedback or a failed SPI exchange disables all drives on the next controller read. R3 releases the damping owner and submits neutral independently of fresh balance. A terminal fault needs a supported full home. Run `home --supported --recover-imu --home-only` in the supervised terminal after a verified fault and latched Stop. Before field use, shut down the bench session, restore the strict enrollment and perform supported Home. These are candidate procedures for the first hardware qualification. [Native behavior and test evidence](../../pupperv3-monorepo/ros2_ws/src/control_board_hardware_interface/README.md)
