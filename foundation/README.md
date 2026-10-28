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

## Course coverage and where it lives

| Source | Capability implemented here | Evidence and remaining boundary |
|---|---|---|
| Lab 1 | Signed bounded PD control, consistent position/velocity targets, measured-pose initialization, configurable sampled delay, sensor-loss zero-torque command | [Classical tests](tests/test_classical.py), [runtime tests](tests/test_pd_runtime.py); gains, feedback, delay and thermal behavior need the supported single-joint robot setup |
| Lab 2 | Homogeneous-transform FK and recorded-motion workflow | Student FK is preserved; FK/IK and tuner parity are checked for all four legs. Physical frame/sign calibration and camera/recording setup still matter |
| Lab 3 and extensions | Bounded damped IK, strict unreachable-target rejection, validated gait caches, trajectory interpolation, tuner-to-keyframe conversion, cycloidal swing, additional gait patterns | [Position tests](tests/test_position_controllers.py), [single-leg tests](tests/test_single_leg.py); the full default gait is checked against bounds and FK. Geometric reachability does not prove dynamic stability |
| Lab 4 | Course PPO training/deployment stack, recovered team baseline, reproducible configuration and export checking | Recovered team checkpoints/exports agree numerically. [Paired GPU comparison](results/lab4-stablegait-clamp-gpu-comparison.json) finds improved tracking after controller-clamp finetuning, but more effort/slip and no fall-count improvement. No promotion; tighter driver limits and the shipped policy remain separate comparisons |
| Lab 5 and options | Person following, target persistence, clothing-color reacquisition, timed obstacle detours, detection watchdog, explicit integration into Lab 6 | Required and optional lab checks plus fault tests. Color and avoidance remain separate opt-ins; camera/Hailo and physical stopping behavior need the robot |
| Lab 6 tools | Finite movement limits, turn/move/arc, follow interruption, serialized jobs, honest queued/completed outcomes, interactive actions and transitions | [Gemini behavior and operation](../gemini_pupper/docs/foundation.md); course checks and fault-injection tests cover stops, failures and late service responses. Live Gemini and ROS are separate tests |
| Lab 6 sit/stay/shake | Motion editing, seated idle, left/right shake, standing exit, PPO training and exported clip contract | Corrected export installed locally; [controller-limit evidence](results/lab6-sit-corrected.json) passes all eight scenarios on CPU/GPU and C++ inference parity. A [tighter driver-limit test](results/local-driver-tracking-evaluation.json) exposes a forward-COM weakness; further refinement and hardware checks remain pending |
| Lab 6 spider | Entry/idle/exit library plus a separate walking policy with matching stance handoff | Both policies are necessary. [Command sweep](../pupper-mjlab/configs/foundation/spider_commands.json) supplies bounded evaluation requests; permitted operating speeds must come from successful evaluation, not training caps |
| Optional Lab 1 | Timestamp-aligned motor and tactile-image datasets, recording-level splits, multitask models, PCGrad, held-out evaluation, TorchScript contact export, nonblocking contact pause | [Data/model evidence](../implicit_compliance/README.md). Contact pause is opt-in on single-leg IK; terrain heads are not connected to robot behavior. This model needs live calibration |
| Optional Lab 2 | Longer/faster references, shared multigait policy, rollout capture, phase folding, reference bootstrapping, reward ablation, randomized/bumpy evaluation, Mystery jump capture | [Experiment protocol](../pupper-mjlab/configs/optional2/README.md). Each candidate must beat its matched baseline; rejected experiments stay rejected. A captured jump must contain real flight and landing |
| Optional Lab 3 | Pupper character/context, deterministic local hidden-trick gate, pinned UniMate generation, fitting/repair, candidate inspection, physics playback and tracking-policy training | [Generated-motion workflow](../pupper-mjlab/docs/pupper_foundation.md). Generated kinematics alone are not deployable; the selected policy must be trained, evaluated and approved |

The course support sources are also present: the robot monorepo, class rebase
bundle, deployment controller, gait tuner, ROS reference material, and pinned
UniMate source. Their presence does not mean the laptop has a ROS installation
or that the robot should be upgraded to a different upstream build. Inspect the
actual class-image revision before any installation.

## Simulator corrections that affect the evidence

The audit found three consequential differences between apparent and tested
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
coordinate transform was checked, but the installed Pi revision remains unknown.
The original `--deployment-target-clamp` flag matches the neural controller only.
New runs can additionally specify `--driver-limits foundation/configs/class-driver-joint-limits.json`;
the exact named limits and file hash become part of the run contract. See the
[driver source/target audit](results/local-driver-position-limit-audit.json) and
[tracking re-evaluation](results/local-driver-tracking-evaluation.json). The latter
retains one failed robustness case for Sit and eight passing Spider-clip cases;
it does not establish Spider walking or a physical robot result.

The corrected body-contact evaluation changed the original sit and spider tracking
verdicts. A subsequent corrected sit continuation passes the documented suite;
spider refinements remain under evaluation. Historical “HOLDS UP” labels describe the simulator used at the time. They are not
sufficient evidence for deployment; new evaluations must name the actual contact,
actuation and randomization contracts. A stable-looking animation can still be
physically wrong when contact is disabled.

## How the capabilities fit together

There are two operating modes. The classical PD/IK exercises use the position or
effort controller for a supported experiment. The learned behavior stack uses
the locomotion/trick controllers, camera follower, command arbitration, and
Gemini tool server. They share geometry, data contracts and validation tools;
they must not compete for the same motor command interfaces.

In the learned stack, Gemini selects a bounded tool, the server checks and queues
it, the current controller executes it, and feedback determines the outcome.
Following is explicitly stopped before another motion. A seated trick exits
through its standing clip; the spider hands off between its clip policy and its
separate slow walking policy. Unknown service outcomes quarantine new motion
instead of assuming that a timed-out request never happened.

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

Useful offline checks from the outer repository (with dependencies installed):

```sh
python -m pytest foundation/tests implicit_compliance/tests gemini_pupper/tests
python gemini_pupper/lab/test.py
```

Simulation and deployment parity checks live under `pupper-mjlab/tests/` and
`pupper_gait_deploy/test/`; run the relevant tests after changing those contracts.
The standalone C++ tests check gait references, command filtering, observation
layout and clip playback without connecting to a robot. Missing dependencies are
reported as an environment gap, not counted as passing tests.

Use the [hardware checklist](HARDWARE_CHECKLIST.md) and
[read-only inventory](scripts/robot_inventory.sh) when Pupper is available.
Before any motion, a person beside the robot must approve that specific test and
be ready to use the physical stop. The course requires TA approval before a new
policy's first hardware run. Keep the current approved robot setup intact until
the replacement has earned promotion.
