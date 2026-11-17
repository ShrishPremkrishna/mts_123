# Pi source reconciliation — November 16, 2026

The Pi contained unpublished shooting and onboard perception work in `/home/pi/kick`, outside its clean but older `/home/pi/mts_123` checkout (`c6f211c`). The older `/home/pi/workspace/mts_123` checkout was also clean (`7050d51`). No newer overhead perception implementation was found in the inspected project directories. This does not inspect teammates' computers.

The combined source was fast-forwarded into GitHub `main` and `/home/pi/mts_123` at `16dd850`; later checkpoint-only commits may advance that revision. The Pi's previous source remains at `codex/pi-source-before-20261116`. Combined Mac tests passed **187 tests plus 48 subtests**; the Pi's stdlib headless smoke completed four simulated score/return scenarios and the expected blocked near-wall scenario. No ROS launch, homing or motor command was issued.

The provisioned `/home/pi/pupperv3-monorepo` is a separate, modified native installation. Its test-policy slot already contained the recovered learned-kick policy, and its binaries differ from the historical R5/R6 identities. Selected source/config/policy/library hashes were checked before and after synchronization and did not change. Do not infer which policy a future launch selects from this source checkout alone.

## Recovered contribution

[behaviors/](behaviors/README.md) preserves the recovered Python source, tests, launch/tools, learned policy and manifest. Code/model files are byte-identical to the snapshot; dated notices were added to the stale README/runbook. [RECOVERY.json](behaviors/RECOVERY.json) records original file hashes. Logs/images remain in the external archive, not Git. The installed native workspace and the original `~/kick` directory are preserved separately and are not replaced by this source import.

| Component | Evidence and limits |
| --- | --- |
| Stock-gait run-through shot | Pure controller and ROS service exist. Recorded manual-coordinate trials completed align/charge/coast/brake sequences at commanded 0.55–0.70 m/s. Achieved speed and ball contact were not measured in the formal trial table. |
| Onboard ball perception | Color beach-ball detection, optional YOLO and angular-size ranging produce capture-stamped robot-relative positions on `/soccer/ball`. Live messages appear in recorded trials. This is not overhead localization, ball velocity or goal detection. |
| Learned header kick | Policy hash matches its manifest. The reported 84.6% exported-policy simulation result is not reproduced here. One live trial aborted on lost ball; another reported done despite an extreme tilt and native emergency stop. Candidate remains unaccepted. |
| Field-input adapter | Kick nodes can consume ball/goal positions plus a robot pose. No producer of that calibrated global state was recovered. |
| Existing gameplay | `pupper_fc/` still runs against the simulator. Recovered behaviors are not wired into its controller or a live robot transport. |

Offline checks on the recovered source passed 44 tests using fake ROS/kinematics. Neither tests nor controller `done` receipts establish successful physical shots. `trial_log.csv` contains only its header; contact, ball travel and operator observations still need recovery from the teammate/video.

## Resolve before activation or integration

- **Learned targets are not velocity:** the learned node writes ball x/y and aim into `/cmd_vel`; zero means a ball at the feet, not a stopped policy. It cannot share an ordinary velocity route without explicit controller ownership.
- **Fall/activation lifecycle:** the learned bridge can return to the walker after a native tilt stop while labeling the attempt done. A timed-out activation can also complete late while its response is ignored. Unknown activation and fall state must not lead to automatic reactivation.
- **Command expiry:** the installed native controller retains the last command. The trial mux timeout does not cover the mux itself dying.
- **Perception frame:** stale/missing IMU silently produces unlevelled coordinates labeled `base_link_yaw`. Consumers do not check frame IDs; goal receivers discard original capture timestamps. Resolve these before feeding overhead data.
- **Calibration:** the trial detector used ball radius 0.097 m and empirical range scale 0.616, differing from defaults. No measured calibration error is retained. Blind-zone dead reckoning is a temporary approximation.
- **Ownership:** some trials disabled gateway checks. Publisher counts do not establish publisher identity. The standalone course/mux launch must not run beside the Gemini motion owner as-is.

## Useful evidence to recover from the operator

Which ball and bumper were used; whether manual run-through trials actually contacted and moved the ball; videos for trials `rtB07`, `rtC2`–`rtC4` and learned `lkS1`; location of the training source referenced as `c5385eb`. Do not infer physical contact from a state transition.

## Next integration decision

Use the stock-gait shot as the first candidate behind the gameplay command interface. Coordinate with the perception owner for overhead image ingestion, calibration and two-robot field state; onboard ball tracking does not replace them. Keep the learned policy as an experiment while its failure handling and physical behavior are addressed. Source checkout synchronization does not install native binaries, change selected policies, home or start services.

### Shooting baseline preparation — November 16

Fresh deployment/saved-policy inventory contains **nine distinct locomotion actors**: stock default, legacy, four Markus exports, Ryan, Shrish and Tej. There is also one learned header actor and one three-legged specialist. Fourteen filenames collapse to eleven unique exports/networks; historical native-only backups are outside this shortlist. Stock default and alternate controller slots use the **same** actor (`ecf3a4ab`); different caps do not create another policy. Inventory hashes are in the external recovery store's `policy-candidates-20261116.json`.

The normal default slot caps forward commands at **0.49 m/s**, while run-through estimates travel from the requested command. Requests above the active cap can therefore brake before contact. This is a verified mechanism, not a demonstrated explanation of every recorded miss. The installed launch has no `config_file` argument; the historical runbook's override must not be assumed effective. Historical trials sometimes explicitly selected the alternate slot and sometimes used whichever slot was active.

Next supervised baseline: select and verify stock default, command below its cap (initial target 0.45 m/s), measure one short no-ball advance and braking, then repeat with the ball and record actual contact/direction. Keep one homed session. User's temporary ball is approximately seven inches, soft/partly deflated, with no bumper fitted. Contact geometry remains to be observed. Do not enable the learned actor or test every archived gait before establishing this baseline.

External recovery store: `/Users/shrish/workspace/pupper-foundation-artifacts/2026-11-16/pi-reconciliation/`. `pupper-kick-recovery-20261116.tar.gz` SHA-256: `1e067b43a97c933f0fae2c3aa4752a15efe1adcf32281ed05ac7b219101df8f0`. Native source/config/policy archive SHA-256: `e6e0349c836616fb7c0d82f44b0a8a74134d17a2923c43bb25a251f33f38175c`. The session checkpoint records the final synchronized source and runtime distinction.
