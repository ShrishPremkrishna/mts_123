# Pupper FC release and hardware qualification

This candidate shortens the path from a powered robot to repeated soccer attempts. Keep the Pi, board driver, calibrated joint coordinates and walker alive between attempts. The laptop owns the overhead camera, field geometry, game roles and operator controls. The Pi owns its 12 actuator commands, balance feedback, onboard Hailo detector and physical Stop.

## Dashboard and runtime update · November 20

The laptop now runs the enrolled Pi's Prepare and Home helpers from each team card. A cold session requires support during Home and a fresh cue to enable standing. A connected Stop gamepad is still required for standing. Full Home attaches the runner. End match keeps a healthy session standing and the next Start match uses the existing connections. Camera settings and `./pupper-fc --self-test` run the same motor-free check of six floor tags, both body tags, ball tracking and kickoff using synthetic images. Developers can run `./pupper-fc-check --ros` for the quick checks plus Blue, Red and paired ROS rehearsals.

The selected Pi application dropped from 79 files to 56. The old wildcard packaged 23 unused files (315,083 source bytes), including the conversational server, chat client, classroom exercises and fake-server tests. The match profile selects its required files explicitly. It retains a shared controller-switching client, leg-kick implementation, safety gateway, native ROS controller, board driver and Hailo detector. Only the selected walk and kick controllers own the legs.

Normal match configuration now has six sections instead of 14. It omits the unused joint broadcaster, derived dynamic joint transforms and per-frame detector timing file. Full recording still provides those diagnostics. The FC runner removes six unused ROS connections for conversational images, person following, legacy velocity and reference tricks. The balance watchdog and direct native joint feedback remain active. In seven alternating software trials, median cold Prepare fell from 268.6 to 221.6 ms. Warm Prepare stayed near 22 ms. Preparing the Home launch description fell from 133.9 to 126.9 ms. These are ARM64 Jazzy measurements with no motors. [Runtime inventory and comparison](../../foundation/results/fc-runtime-path-20261120.json)

A separate guard and native walker rehearsal withheld front-camera detections for 1.0, 2.0 and 3.0 seconds. Walking-policy input reached zero after 276–298 ms and stayed zero through the dropout. All three trials required new clearance, a source zero and a new drive. The same walker activation then received 0.16 m/s again 192–224 ms after frames returned. This measured the actual ROS guard and compiled policy on memory interfaces. Balance-sensor loss still trips the separate 100 ms watchdog. Physical braking and floor recovery remain open.

## Previous verified release · November 20

The downloadable source release is `7236bc6c3899ec8265c46a318defd61ed1d126e9` on `codex/pupper-fc-match`. A fresh clone at that commit rebuilt both archives byte for byte. SHA-256 verification covered the delivered files. [Laptop ZIP](../../outputs/pupper-fc-release/pupper-fc-laptop.zip) · [Pi tarball](../../outputs/pupper-fc-release/pupper-fc-pi.tar.gz) · [SHA256SUMS](../../outputs/pupper-fc-release/SHA256SUMS) · [Portable installation](PACKAGING.md)

Two clean extractions on the Apple Silicon development Mac reached HTTP-ready in **3.79 s** and **6.83 s**, the latter during native compilation. A warm process restart took **0.26–0.28 s**. Reopening an already running lobby took **0.05–0.06 s**. Both cold launches used the ZIP’s 19 bundled wheels with no package index or external cache. Each extracted app served its lobby and a JPEG from the synthetic camera showing all six floor tags, two robots and the ball.

In an isolated ARM64 Ubuntu/Jazzy container, the independently extracted Pi source staged and verified all **79 application files**. All three native packages compiled in **37.6 s**. Motor-inactive Prepare from the final staged release took **0.219 s cold** and **0.022 s warm** with a temporary profile and the approved archived HEF. Constructing the Home launch description from the pinned profile took **0.126 s**. None of these operations started a hardware process. The target Debian Pi needs its own board build.

In MuJoCo, both nominal kicks scored. Both offset cases made contact with the intended paw and restored walking, and the left offset shot missed. Three of the four selected trials scored. A separate native ROS wire test delivered 0.16 m/s to the walker model, observed the kick controller apply frame 50, and delivered 0.16 m/s to the restored walker under a new activation. The staged inputs matched the downloadable release. [Verification receipt and scope](../../foundation/results/fc-offline-release-20261120.json)

### Reproduce the offline checks

From the release checkout, run the quick suite and the network-isolated ROS rehearsal. The ROS image must already be available locally.

```sh
./pupper-fc-check
docker run --rm --network none -v "$PWD:/repo:ro" -w /repo \
  -e PUPPER_ISOLATED_ROS_TEST=1 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e PYTHONDONTWRITEBYTECODE=1 ros:jazzy-ros-base \
  python3 -u final_project/behaviors/tools/ros_fc_match_smoke.py
./pupper-fc-package
(cd outputs/pupper-fc-release && shasum -a 256 -c SHA256SUMS)
```

For the motor-inactive Linux timing, supply the staged release directory, compiled ROS install directory and approved detector HEF to [benchmark_fc_prepare.py](../../foundation/scripts/benchmark_fc_prepare.py) using `--release`, `--install` and `--detector-model`.

## Where the ten minutes went

Two archived Pi runs on November 20 show how much time was spent after homing. Times in the table use the same robot log clock for native milestones. The last column measures launch to the first body-command request, including session setup. A request can be zero.

| Archived run | Stationary check through finished home | Of that, mechanical homing | Home finished to first walker activation | Launch to first body request |
| --- | ---: | ---: | ---: | ---: |
| Field run, boot 74798bf0 | 8.19 s | 5.08 s | 252.07 s | 609.8 s |
| Final run, same boot | 10.66 s | 3.85 s | 113.64 s | 178.99 s |

The driver moves each actuator toward its configured reference, detects contact from tracking error and motor gains, and records an offset that maps controller coordinates to the mechanical joint angle. It then returns the joints to the configured pose. The stand supports the body during that unloaded leg movement and the preceding stationary balance check. FC has no validated persistent encoder origin, actuator reboot generation or cold-start pose measurement. A match reset, camera reconnect, game Stop or agent handoff has no inherent need to redo this procedure while the calibrated board session remains valid.

In the first archived run, an initial walker activation was stopped after about two seconds. Following its second activation, the walker stayed active for 190.23 seconds before the driver reported an expired balance-sensor receipt and disabled the motors. The two nearest CPU samples were 78.75 °C and 77.65 °C, with firmware flags `0x0`. That older fault message did not identify the expired component. The separate final run peaked at 84.8 °C and showed throttling. All 4,176 source commands were zero and there was no kick. Those observations account for two different symptoms. Neither recording establishes a general idle timer or a heat-triggered shutdown at 84 °C.

The hardware archive is `2026-11-20/final-offense-74798bf0` under the project hardware archive root. The two run directories are `fc-field-74798bf0` and `fc-final-74798bf0-0051`. Each contains `robot.log`, thermal samples, the session manifest and a timestamped telemetry bag. The archive contains 48 and 45 verified files respectively. The previous physical standby receipt records selected library mapping and inactive leg owners. [Previous physical standby](../../foundation/results/pi-combined-walk-kick-20261119.json)

## What the soccer runtime needs

| Keep in the match path | Remove from repeated setup or keep outside the match |
| --- | --- |
| One controller manager, one board driver, joint and IMU feedback, one gait policy, one kick policy, one physical Stop owner | The FC launch already excludes course trick controllers, the ordinary Gemini HTTP server, conversational prompts, the LLM velocity mux and the Lab 5 person-following service. A legacy configuration or source file in the checkout does not imply an active process. |
| One onboard camera owner and a headless Hailo detector with capture timestamps | The FC session starts its own detector. The former manual attachment, display, rendered masks and interactive browser camera controls have no role in routine matches. |
| One overhead camera producer, calibrated floor tag coordinates, robot tags and ball observations | Known floor tags determine metric geometry. Manual points are a fallback for a setup missing usable reference tags. A cloud vision call is unnecessary in the motion loop. |
| Exactly one authority for body requests and an independently current camera observation | The guard now reports each new camera capture and permission transition immediately. Repeated unchanged descriptions retain the existing heartbeat. Old images and old commands still expire. |
| Controller ownership checks across walk, kick, return and Stop | Cache the two independently valid checks. A 50-tick idle harness went from 1,300 ROS graph calls to 234. This measures API calls in a fixture. |
| Run identity, pinned native libraries and field decisions needed for postmortem | The final archive had 163,906 messages. Joint and raw IMU broadcasters accounted for 139,184, or 84.9%. The motion path can consume these streams without recording every sample. The enrolled match profile now records four kick/Stop event topics. Full-rate capture is available with `--recording full`. Prepare generates the session manifest and controller YAML once. Home checks the pinned inputs and consumes those files. Pi CPU and storage savings still need measurement. |

Replay of the last recorded camera and zero-command traffic raised camera-ready time from **15.33 to 31.70 seconds** within a 52.37-second attack window. Median remaining freshness rose from 89 to 168 ms. Both replays emitted zero nonzero motor commands. Recorder arrival times approximated callbacks and the replay used a 20 ms timer. This is evidence of shorter software latency. It does not establish a walk, a goal or reduced Pi temperature. [Body guard](../behaviors/body_command_guard.py) · [Offline graph and feedback tests](../behaviors/test_fc_feedback_rates.py)

The markerless body preview found the six labeled bodies across eight stored views with no extra proposals. Darkening those views by 35% produced one extra proposal. On the development Mac, the appearance detector alone took 6.23 ms median and 7.28 ms at the 95th percentile at 896 pixels wide. The capture set covers one robot in a single room. Team identity and heading still come from the robot tags. In the local camera-worker rehearsal the lobby recognized six field tags, two robot tags and the ball in synthetic frames. [Appearance replay](../../foundation/results/markerless-appearance-20261120.json) · [Camera calibration](../overhead_perception/README.md)

## Operator path

From a cold boot on an enrolled Pi, run **Prepare**, support the robot and run **Home**, then click **Start match** in the laptop lobby. Enter after homing or gamepad Options gives the standing cue. Keyboard standing requires a connected Stop gamepad. For a supported home without a gamepad, `home --supported --home-only` measures the joints and leaves walker activation to a later cue. [Three operator actions](../../foundation/hardware/FC_OPERATOR.md)

For the next match keep the camera, runner, board, calibration and healthy walker. **End match** requests standby. **Start match** begins a new epoch. Each runner reports its field profile over the existing connection. Field readiness publishes on transition. The camera holds tactics and keyboard drive until every participating runner accepts kickoff. A hard Stop, live native kick or board fault has its own recovery path. [Lobby](lobby.py) · [Field source](../behaviors/live_field_attack.py)

The enrolled `robot_id` in `~/.config/pupper-fc/session.json` determines which team the Pi advertises through Avahi. The laptop verifies the remote session identity on connection. Give the second robot its own identity, platform template, target-native library digests and field-side assignment. Confirm it can be connected and stopped independently before paired floor play.

## Match rehearsal on the final sources

| Connected teams | First successful kickoff to nonzero command | Warm kickoff to nonzero command | End to standby |
| --- | ---: | ---: | ---: |
| Blue | 0.276 s | 0.325 s | 0.198 s |
| Red | 0.334 s | 0.323 s | 0.264 s |
| Blue and Red | 3.127 s | 0.376 s | 0.260 s |

Each cell is one observation with prepared peers. The paired trial began with an injected rejection. The fixture ran the production lobby, field subprocesses, UDP clock probes, ROS Jazzy DDS in separate domains, executor and kick handoff. Local adapters supplied camera, SSH peer, guard and native completion feedback. Timings end at the first nonzero executor command. Pi inference, network latency and physical movement require measurement on the field. [Commands and per-roster results](../../foundation/results/fc-match-ros-20261120.json)

Red rejected kickoff after Blue had already armed. Both stayed still and returned to standby. Each solo robot completed two separate kick attempts, saw a fresh ball measurement after walker return and covered a new 24 cm approach. A controlled full-ball crossing scored once. An early Resume preserved the match. Both teams entered the second rally after replacement, including a Red runner that missed the original goal frames. Field-source EOF withdrew play. In Chrome, W before kickoff produced zero nonzero requests, W after kickoff commanded forward motion, and Escape stopped both mixed-mode team adapters. The final quick suite passed 55 checks.

## What can recover after lost feedback

- **Camera or match traffic** can expire independently. With valid body feedback, issue zero through the walker and preserve the motor session. Renewed data needs a live operator/run authority and an admissible new capture. Discard expired motion and queued kicks.
- **On a permanently supported bench**, `supported_standby` homes once, inhibits all leg-controller claims and sends disabled packets. A 1.005-second simulated gyro dropout tripped at 104 ms and invalidated the exported balance fields. The same process restored telemetry after 3.057 seconds of fresh stationary readings. All 12 calibration offsets were preserved and leg claims remained inhibited. Moving this robot requires a separate field session.
- **During field movement**, `supported_rehome` sends a full neutral packet before reporting the fault. The real ROS Jazzy ResourceManager in the offline fixture moved the board to unconfigured and withdrew all 60 command interfaces. Returning balance alone left it disabled. The explicit supported home discarded all 12 previous offsets and measured new ones, including a simulated encoder reset. The manager stayed alive. An independent Stop canceled a pending rearm. A new complete controller switch emitted an enabled packet containing zero targets, gains and feed-forward torque before accepting fresh commands. A 2.005-second quaternion loss tripped at 104 ms. The explicit supported rehome took 3.175 seconds, including stationary qualification and simulated joint stops. With both components withheld for three seconds, strict and supported-rehome profiles stayed neutral and unconfigured until operator intervention. Real leg travel takes additional time.
- **On the floor**, the default remains strict. The current SPI reply lacks a trusted board or encoder restart counter. A live process or unchanged joint angle cannot establish continuity after a power interruption. Balance loss in an unsupported standing robot can make it fall. A transient moving-gait recovery that keeps torque available would require a separately qualified joint-local strategy and actuator continuity evidence. No live floor run here demonstrates that capability.

After the supported home, the selected FC controller in `pupper_gait_deploy` clears observation history, zeros velocity, resets clip and deadline state, and scopes callbacks to the new activation. The game still requires a fresh start. The supervised field recovery command is `home --supported --recover-imu`. The operator helper checks the consumed-fault marker, manager identity, Stop latch and inactive leg owners before the supported transition. [Selected native controller](../../pupper_gait_deploy/src/neural_controller.cpp) · [Operator recovery](../../foundation/hardware/FC_OPERATOR.md)

The candidate parameters are `PUPPER_FC_IMU_RECOVERY=supported_rehome` and `supported_standby`. The launch preparer preserves the frozen URDF, produces a distinct selected description and records its digest. For either candidate, select a target-built driver with the recovery contract marker and pin the file actually mapped by the manager. Every fault log captures separate quaternion and gyro sequences, monotonic receipts, ages and known batch ages after commanding neutral. [Native build and rehearsal](../../pupperv3-monorepo/ros2_ws/src/control_board_hardware_interface/README.md)

## Reader regression

In an intercepted I²C test, the old reader completed a valid balance report, then blocked on its next header read before publishing. A 120 ms stall made the report 123 ms old in the final comparison and 125 ms old in the earlier reproduction. The fixed reader made the completed report visible to the real sampler after 115 µs. With no new input, the original receipt still expired at the 100 ms watchdog. New Pi enrollments require the per-report publication capability and pin the selected library digest. The native suite passed 63 device-free tests. [Source identities, times and recovery matrix](../../foundation/results/fc-native-reliability-20261120.json)

## First physical test and rollback

1. With the native session shut down, build the shipped board package once on the Pi against its installed FC underlay. Enroll the strict profile with that new prefix. The recorded Pi already has a selected controller, gateway and detector. [Build and enrollment commands](PACKAGING.md)
2. On the stand, run Prepare and supported Home. Verify the new driver mapping, fresh feedback, walker activation and physical Stop. Give a fresh supported standing cue after Stop. Home-only can be checked separately without a controller.
3. In the lobby verify scale, posts, the correct colored team/body heading and the pink ball. With one operator and one robot, start with a short keyboard drive. End and start again on the same standing session. Time each click to first observable motion and verify Escape and physical Stop. Then try one autonomous approach and kick with the operator present. Repeat on the second robot before paired floor play.
4. For rollback, support Pupper and shut down its owning native session. From the extracted Pi directory run `python3 foundation/hardware/fc_walk_kick_release.py rollback`. The helper restores the saved enrollment or disables a first managed installation. The saved native overlay must remain available. [Shutdown and rollback](../../foundation/hardware/FC_OPERATOR.md)

The packed Pi still needs target build and enrollment, cold and warm physical timing, loaded driver and Stop verification, physical camera clearance, braking, thermal behavior, contact and paired play. The headless and native rehearsals use synthetic measurements or memory hardware. The recorded hardware runs identify their previous releases.
