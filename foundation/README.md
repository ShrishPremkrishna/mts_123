---
date: 2026-10-27T15:34
tags: [cs123, foundation]
---
# Pupper engineering foundation

This branch turns the Fall 2026 course software into a reviewed, testable starting
point for a final project. It preserves the student's coursework on `main`.
**The branch is not yet a hardware-validated release.** Implementation, trained
weights, simulation results, and performance on this particular robot are
separate claims. A passing unit test or a training reward curve cannot establish
the last two.

## Current robot preparation

The first supervised baseline homing completed on 29 October. All twelve motors
reported homed, the operator observed normal motion, and joint/IMU feedback was
captured. Shutdown disabled holding (confirmed by the operator), but the original
driver then aborted while destroying a joinable IMU thread. The minimal cleanup
fix builds in a separate Pi overlay and passes five lifecycle tests. A subsequent
[actual IMU/SPI initialization/destruction probe](results/driver-init-only-pi-reviewed.json)
passed without enabling motors or homing; full activated-stack shutdown remains a
separate check. See the [supervised homing record](results/supervised-homing-20261029-reviewed.json).

The robot is currently inactive, with relaxed legs confirmed by the operator.
The [first combined console trial](results/console-motion-trial-20261029-reviewed.json)
completed homing, activated the preserved default controller briefly, then logged
emergency stop and hardware deactivation. Its recorder failed on a real ROS array
type, losing detailed telemetry; the trial is incomplete. Repeated SIGINT during
shutdown ended the manager with exit -2, so graceful activated-driver teardown
also remains unproven. The frozen inputs and all surviving logs are preserved;
recorder and shutdown review are in progress before any further motion.

The [shared gamepad/Gemini stop gateway](results/shared-stop-v4-pi-reviewed.json)
has passed independent review and is staged in v4: all ten native ROS test groups,
366 Gemini tests and 166 course checks pass on the Pi. The physical gamepad,
walking performance, tricks and following remain untested. New trained policies still
need applicable course approval. The [stationary preparation record](results/hardware-session-20261029-reviewed.json)
preserves earlier backups, policy recovery, native builds, CPU policy comparisons
and synthetic Hailo inference. Live camera following and Gemini remain separate
validation steps. The original working directories and coursework main remain preserved.

## Course coverage and where it lives

| Source | Capability implemented here | Evidence and remaining boundary |
|---|---|---|
| Lab 1 | Signed bounded PD control, consistent position/velocity targets, measured-pose initialization, configurable sampled delay, sensor-loss zero-torque command | [Classical tests](tests/test_classical.py), [runtime tests](tests/test_pd_runtime.py); gains, feedback, delay and thermal behavior need the supported single-joint robot setup |
| Lab 2 | Homogeneous-transform FK, read-only topic recording, offline MCAP import and validated foot-motion export | [Recording review](results/lab2-recorded-motion-review.json) covers the saved bag and explicit trimming. Its front-right foot is static; the moving back-right trajectory exceeds a knee bound and is refused. Physical frame/sign calibration remains |
| Lab 3 and extensions | Bounded damped IK, strict unreachable-target rejection, validated gait caches, trajectory interpolation, tuner-to-keyframe conversion, cycloidal swing, additional gait patterns | [Position tests](tests/test_position_controllers.py), [single-leg tests](tests/test_single_leg.py); the full default gait is checked against bounds and FK. Geometric reachability does not prove dynamic stability |
| Lab 4 | Course PPO training/deployment stack, recovered team baseline, reproducible configuration and export checking | Recovered team checkpoints/exports agree numerically. [Paired GPU comparison](results/lab4-stablegait-clamp-gpu-comparison.json) finds improved tracking after controller-clamp finetuning, but more effort/slip and no fall-count improvement. No promotion; tighter driver limits and the shipped policy remain separate comparisons |
| Lab 5 and options | Person following, target persistence, clothing-color reacquisition, timed obstacle detours, detection watchdog, explicit integration into Lab 6 | Required and optional lab checks plus fault tests. Color and avoidance remain separate opt-ins; camera/Hailo and physical stopping behavior need the robot |
| Lab 6 tools | Finite movement limits, turn/move/arc, follow interruption, serialized jobs, honest queued/completed outcomes, interactive actions and transitions | [Gemini behavior and operation](../gemini_pupper/docs/foundation.md); course checks and fault-injection tests cover stops, failures and late service responses. Live Gemini and ROS are separate tests |
| Lab 6 sit/stay/shake | Motion editing, seated idle, left/right shake, standing exit, PPO training and exported clip contract | [Expanded-COM policy](results/lab6-sit-comx02-reviewed.json) installed locally after all eight existing scenarios passed on CPU and GPU with class driver limits. The +2 cm weakness improved; +2.5 cm remains backend-sensitive and outside the training range. [The full interactive sequence and actual default return](results/sit-interactive-default-return-reviewed.json) pass a nominal native check. Hardware checks remain |
| Lab 6 spider | Entry/idle/exit clips, separate slow walking policy, and tested controller handoff | [Final matched sensing study](results/spider-encoder-observation-study-reviewed.json): both policies complete 512/512 sampled GPU stops; measured sensing improves the stopping tail but retains support and tracking tradeoffs. [Ten full native chains](results/spider-encoder-native-reviewed.json): both nominal returns settle; all eight selected adverse returns remain moving under the fixed default. Walking code and research pairs are staged; no walking pair is installed |
| Supplied finite tricks | Twelve active finite course policies through native activation, playback and terminal hold | [Nominal screen](results/course-tricks-native-screen.json) completed all twelve with full hull contacts and driver limits. [Five standing-ending tricks and the generated trick](results/finite-tricks-default-return-reviewed.json) return to actual default walking; [seven lying-ending tricks](results/lying-tricks-later-default-return-reviewed.json) return on a later explicit request. Sneeze peak speed and randomized/hardware performance remain limitations |
| Supplied reference jump | Exact shipped table through native gait controller with paired no-jump controls | [The exact Gemini reference-message path](results/gemini-jump-api-reviewed.json), using zero velocity and the actual immediate callback, recovers only two of four trials after fresh activation and two of four after prior walking; paired no-jump controls pass all four each. [Earlier forward0.3m/s success](results/shipped-jump-native-reviewed.json) does not validate this stopped-jump tool. No reliable standing-jump or hardware claim |
| Optional Lab 1 | Timestamp-aligned motor and tactile-image datasets, recording-level splits, multitask models, PCGrad, held-out evaluation, TorchScript contact export, nonblocking contact pause | [Data/model evidence](../implicit_compliance/README.md). Contact pause is opt-in on single-leg IK; terrain heads are not connected to robot behavior. This model needs live calibration |
| Optional Lab 2 | Longer/faster references, shared multigait policy, rollout capture, phase folding, reference bootstrapping, configurable reward comparisons, randomized/bumpy evaluation, Mystery jump capture | [Experiment protocol](../pupper-mjlab/configs/optional2/README.md). Corrected flat and Bumpy studies are complete with no promotion. [Final Mystery continuation](results/optional2-mystery-reviewed.json) reduces failures but produces only 0/0/1/1 strict flight-and-recovery events across four seeds. No reliable new jump policy or promotion; all nine complete archives preserved and its idle GPU released. Earlier altered-reference runs are explicitly labeled |
| Optional Lab 3 | Pupper character/context, deterministic local hidden-trick gate, pinned UniMate generation, fitting/repair, candidate inspection, physics playback and trained tracking policy | [Reviewed standing trick](results/generated-motion-trained-review.json) is installed locally behind the secret gate: a modest rise/sway/return, not a complex dance. Seven CPU/GPU scenarios and native activation/hold passed; hardware approval remains |

The course support sources are also present: the robot monorepo, class rebase
bundle, deployment controller, gait tuner, ROS reference material, and pinned
UniMate source. Their presence does not mean the laptop has a ROS installation
or that the robot should be upgraded to a different upstream build. Inspect the
actual class-image revision before any installation.

## Simulator corrections that affect the evidence

The audit found consequential differences between apparent and tested
behavior: the Pi clamps motor targets while the original simulator did not;
inertial randomization erased the earlier center-of-mass offset; and tracking
body/leg collision hulls were disabled after creation. Each correction has an
explicit run flag, actual-model tests and a recorded contract, so old results stay
reproducible without being mistaken for corrected results. See the
[target comparison](results/lab4-deployment-contract-comparison.json),
[realized-dynamics audit](results/pupper-randomization-audit.json), and
[body-contact audit](results/tracking-body-contact-audit.json).

The source class hardware driver adds a **second, tighter joint target clamp**,
0.10 radians inside the exported controller limits on all twelve joints. Its
coordinate transform was checked. The captured Pi description now matches all
twelve named class limits; its source and installed libraries are preserved in
the [stationary-session report](results/hardware-session-20261029-reviewed.json).
Actual loaded values and physical calibration remain unverified.
The original `--deployment-target-clamp` flag matches the neural controller only.
New runs can additionally specify `--driver-limits foundation/configs/class-driver-joint-limits.json`;
the exact named limits and file hash become part of the run contract. See the
[driver source/target audit](results/local-driver-position-limit-audit.json) and
[tracking re-evaluation](results/local-driver-tracking-evaluation.json). The latter
retains one failed robustness case for Sit and eight passing Spider-clip cases;
it does not establish Spider walking or a physical robot result.

The corrected body-contact evaluation changed the original sit and spider tracking
verdicts. The first driver-limit Sit continuation was rejected, and a
reference-margin sweep also worsened tracking. The stock training distribution
covered torso x-COM offsets of ±1 cm; the difficult +2 cm case lay outside it.
Expanding only that axis to ±2 cm improved the case on both CPU and GPU while
retaining the original reference and reward weights. That reviewed policy is now
installed locally. At +2.5 cm, CPU still wobbles while GPU passes: this is an
explicit boundary, not a claim of general robustness. Spider walking remains staged because reviewed candidates retain
tracking/support tradeoffs and adverse-case failures. Historical “HOLDS UP” labels describe the simulator used at the time. They are not
sufficient evidence for deployment; new evaluations must name the actual contact,
actuation and randomization contracts. A stable-looking animation can still be
physically wrong when contact is disabled.

A later sensor audit found that the locomotion actor observes true simulated
joint angles while its action subtracts a persistent encoder calibration bias.
The hardware controller observes measured angles, including that bias; tracking
actors already model this correctly. Earlier randomized walking comparisons
remain paired, but their calibration-robustness evidence is incomplete. Nominal
zero-bias native tests cannot expose this mismatch. An explicit, versioned
measured-angle training correction is implemented and passes the full offline
regression. The [completed matched 1,000-iteration study](results/spider-encoder-observation-study-reviewed.json)
evaluates both policies using measured angles. Both pass 512 sampled base-speed
stops; measured-mode training reduces the slowest stop from 1.86 to 1.26 seconds,
but increases light idle torso support and slightly worsens tracking. The
[independent raw-trace review](results/spider-encoder-gpu-independent-review.json)
confirms these tradeoffs. No walking policy is promoted. The [hardware-coordinate derivation](results/encoder-hardware-semantics-review.json)
pins the source evidence.

For new locomotion experiments, explicitly select
`--encoder-observation-mode deployment_measured`. Historical behavior remains
available as `legacy_true`; continuing a legacy checkpoint in the corrected mode
requires an explicit, hash-bound override documented in the study. Exported mode
metadata describes training inputs. Hardware always supplies measured joint
angles; metadata cannot reveal its unknown true calibration error.

A separate reference audit caught a misleading class description: the exported
jump table contains **absolute joint angles**, while the policy observation uses
angles relative to the default pose. The first seed helper added the default
pose twice. [The correction record](results/optional2-class-seed-correction.json)
preserves all affected inputs and labels their descendants as altered-reference
experiments. The corrected helper is checked through the actual training loader
and independent exported-controller math. Capture acceptance now requires
all-body contact-free flight, upward center-of-mass motion and sustained landing
recovery; foot lift alone can happen while the torso supports the robot. See the
[independent capture audit](results/optional2-capture-independent-review.json).
The current gate uses observed first-to-last sample span: five 50 Hz samples
cover 80 ms, not 100 ms. The earlier captured continuation input remains
unchanged and explicitly fails the newer recovery gate. The
[final four-seed audit](results/optional2-mystery-final-flight-audit.json) applies
the corrected gate and retains the negative jump-reliability result.

## How the capabilities fit together

There are two operating modes. The classical PD/IK exercises use the position or
effort controller for a supported experiment. The learned behavior stack uses
the locomotion/trick controllers, camera follower, command arbitration, and
Gemini tool server. They share geometry, data contracts and validation tools;
they must not compete for the same motor command interfaces.

In the learned stack, Gemini selects a bounded tool, the server checks and queues
it, the current controller executes it, and feedback determines the outcome.
Following is explicitly stopped before another motion. A seated trick exits
through its standing clip. The Spider handoff between a clip policy and a
separate slow walking policy is implemented and tested with staged pairs; no
walking pair is installed in the current catalog, which retains the supplied
finite Spider trick. Unknown service outcomes quarantine new motion
instead of assuming that a timed-out request never happened.

A [controller completion audit](results/controller-completion-v2-reviewed.json)
found timers and stale interactive receipts could falsely report completion.
Reference requests now carry unique IDs; policy receipts identify the activation;
interactive clips also identify the specific request throughout queued playback.
Receipts follow successful command application and require the expected fresh
endpoint. The actual ROS Jazzy implementation passes 21 isolated tests, including
the reproduced stale-exit failure. Playback receipts establish controller
progress, not measured physical posture, flight or landing.

The new contact model augments the **single front-right leg trajectory** through
`ik.py --contact-model <export-directory>`. It freezes trajectory time for one
second after fresh high-confidence contact, without blocking ROS. It requires
published effort channels with verified semantics and valid timestamps. The local
class driver estimates effort from its PD controller; it does not measure torque.
See the [interface audit](../implicit_compliance/results/2026-10-27/deployment-interface-audit.json).
It is not a whole-body reflex or a general
collision detector, and it does not feed unvalidated material/slope predictions
into locomotion.

## What “maxed out” can honestly mean

The class supplies local feedback control, learned locomotion and motion
tracking, camera-based person following, language-driven behavior composition,
and experimental learned contact and generated motions. It does **not** by
itself establish navigation with a map, obstacle clearance from depth, robust
person identity, object manipulation, terrain adaptation outside the training
distribution, calibrated force sensing, or reliable recovery from arbitrary
falls. Timed visual detours are especially limited: a bounding box is not a
geometric guarantee of free space.

Command limits are permissions for a controller to receive a request. They are
not measured speed, accuracy, payload capacity, endurance or safety margins.
Likewise, a faster simulated gait is not automatically the better final-project
baseline if it slips more, uses more torque, or fails at transitions.

The [native exported-policy screen](results/shipped-policy-native-screen.json)
supports keeping the shipped course gait as the integrated default. It completed
the four nominal full-range trials; the new Mixed baseline and fast candidate
triggered tilt stops in three and two trials respectively. The fast candidate
tracked forward speed better but lost stability in reverse. These are bounded
simulation results using the policies' actual references, clocks and limits;
they do not establish a hardware speed rating. The shipped gait also retains the
reference-jump contract that the new flat-walking exports lack.

The default also has a documented robustness boundary. In selected full-hull
startup cases it keeps moving after eight seconds at zero command, even without
a preceding Spider trick. [Independent controls and parameter restorations](results/spider-default-restoration-independent-review.json)
confirm that nominal mechanics settle, while changing gains or encoder bias does
not resolve these cases. With nominal gains and zero bias, restoring the entire
mass/inertia group resolves all four cases; restoring the center of mass alone
does not. These are simulated parameter combinations, not measurements of this
Pupper, and no default policy was improved by those diagnostic interventions.
[Stop and status messages](results/stop-reporting-reviewed.json) therefore describe
commands and controller state without claiming unmeasured physical stopping or
posture. This reporting correction leaves controller behavior unchanged.

For a final project, freeze a reliable evaluated checkpoint first. Then choose
one concrete missing behavior and a repeatable success measure, such as following
the same person through an occlusion, adapting a gait to a measured surface
change, or executing a language request with verified completion. Compare against
this frozen foundation on held-out scenarios. That produces a clearer result
than stacking several new demos whose failures cannot be attributed.

## Reproduce and inspect

Use Python 3.10–3.13; the local system Python 3.15 is outside the course training
environment's range. The GPU environment is pinned by `pupper-mjlab/uv.lock`.
The provided [Colab bootstrap](scripts/colab_bootstrap.py) uses the isolated
`pupper-foundation` branch. Runtime output under `/content` is temporary and must
be copied out before releasing a runtime. Model hashes, configuration, seed and
source provenance travel with each run; never replace them with a label such as
`best` without preserving the actual checkpoint identity.

The permanent local checkout is `/Users/shrish/workspace/cs123-foundation` on
`pupper-foundation`; the coursework checkout remains on `main`. The
[local environment inventory](results/local-environment.json) records the tested
Python packages, browser build/type check and relocation verification. It is an
observed macOS environment, not a cross-platform lock. Older result paths under
`/private/tmp/cs123-pupper-foundation` remain historical provenance and map to the
permanent checkout; full archives live separately in
`/Users/shrish/workspace/pupper-foundation-artifacts/2026-10-27`.

The [interruption recovery record](results/interruption-recovery.json) identifies
exactly which temporary outputs were lost and which inputs survived. Current
training runs also use a bounded local checkpoint collector:

```sh
python foundation/scripts/preserve_colab_run.py --colab .venv/bin/colab \
  --session SESSION --remote-root /content/pupper-RUN \
  --out /durable/NEW-ARCHIVE --hours 6
python foundation/scripts/restore_colab_snapshot.py /durable/NEW-ARCHIVE /new/recovered-run
```

The collector verifies downloads against remote SHA-256 hashes, retains earlier
versions by content hash and receipt history, and downloads the latest two visible
checkpoints per folder each cycle. Restore checks hashes and sizes before writing
to a new directory. The index is a rolling **per-file** snapshot; a checkpoint and
status file may describe different moments. Recovery still requires inspecting
the actual exit record, source, inputs and references, which are preserved
separately. Collectors require the laptop and network to remain available. Keep a
complete final archive as well; neither periodic downloads nor `/content` storage
guarantees that the final interval survives a runtime loss.

The [current verification record](results/offline-validation-composite-c3ad206.json)
combines the 905-test full regression, 225 course checks and native C++ checks
with exact source hashes. The later stop-reporting correction separately passes
237 Gemini tests and 166 course tool checks; its controller logic is unchanged.
The 21 isolated ROS Jazzy tests remain separate evidence. All 17 installed policy
files still match their audited identities. Both GPU runtimes have been released
after completed outputs were preserved; no training is left running.

Useful offline checks from the outer repository (with dependencies installed):

```sh
python -m pytest foundation/tests implicit_compliance/tests gemini_pupper/tests
python gemini_pupper/lab/test.py
python foundation/scripts/check_foundation.py
```

Simulation and deployment parity checks live under `pupper-mjlab/tests/` and
`pupper_gait_deploy/test/`; run the relevant tests after changing those contracts.
The standalone C++ tests check gait references, command filtering, observation
layout and clip playback without connecting to a robot. Missing dependencies are
reported as an environment gap, not counted as passing tests.

Use the [hardware checklist](HARDWARE_CHECKLIST.md) and
[read-only inventory](scripts/robot_inventory.sh) for later state changes. Start
from the current stationary-session report rather than repeating completed
backups and builds.
Before any motion, a person beside the robot must approve that specific test and
be ready to use the physical stop. The course requires TA approval before a new
policy's first hardware run. Keep the current approved robot setup intact until
the replacement has earned promotion.
