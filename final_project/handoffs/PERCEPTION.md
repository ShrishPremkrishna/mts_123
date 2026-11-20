# Perception — Ryan

Updated November 19, 2026 · [Pupper FC brief](../README.md)

**Outcome:** Turn overhead mounted iPhone video into usable shared state for two autonomous Pupper soccer players on a roughly 8×12 ft field. Track both robot IDs, positions and headings, the bright ground ball's position and velocity, goals, and derived shot angles, with capture timestamps, age, uncertainty, and validity.

Coordinate the mount, field/robot AprilTags, goals, and ball with Shrish, who owns the arena. Shrish and assisting agents own gameplay integration.

These are optional workstream notes. An incoming agent's role follows its user's current task and relevant source, not this document's title. Current activity and access are in the [checkpoint](../../foundation/results/interruption-recovery.json).

## Status — November 19, 2026

Code: [overhead_perception](../overhead_perception/README.md), on `main` since November 19. That README holds the commands, interfaces, measured results and the list of what is not done.

- **Works (simulation, 56 tests; includes a top-down dashboard drawn from the published state):** camera calibration from measured floor tags, from four corner tags that define the field's position, direction and size, from 2+ unmeasured tags, or from clicked floor points; two tagged robots with heading; a ball by configurable colour ranges plus outline fit (config now set for a pink ball, range untuned); velocities; ages; each robot's view of the ball and goal (body-frame x/y, distance, bearing, shot-line angles; conventions in the package README); a bridge to `pupper_fc` `Observation`; and a line-up-then-run approach planner driven only by a robot-relative ball and goal, exercised on a fake field with no camera (`scripts/fake_approach.py`), not on real perception or a robot.
- **Observed live (desk, iPhone via Continuity Camera, November 16):** 1080p30 into the pipeline; printed tags read; ball outline fitted on one saved frame. No comparison against tape measurements.
- **Decisions so far:** tag36h11, 17 cm floor tags and 8 cm robot tags (ids 0–5, 10, 11); JSON over UDP; field frame identical to `pupper_fc/model.py`. All remain easy to change.
- **Open against the deliverables below:** physical-reference accuracy, lens and latency calibration, replay fixture and live sample, identity-swap / miss / false-detection counts, measured clock uncertainty on each physical Pi, and camera coverage of the full arena (straight-down 1080p needs about 3.8 m of height, where 8 cm robot tags become marginal).

## Immediate working baseline

Establish a reproducible recorded-video baseline and live iPhone ingestion: field calibration, both tagged robots, ball, and goals visible in a shared coordinate frame. Reuse relevant [calibration/tracking code](../../follow_me_pupper/README.md) and inspect the [harness](../../gemini_pupper/README.md); the onboard person follower is a reference, not an existing overhead pipeline. Geometric tag and color detection are candidate approaches; algorithms remain open.

## Required deliverables

- Field calibration and tag layout, measured against physical reference points; consistent robot identities/headings across the field.
- Ball position/velocity and goal state, with robot-to-ball, ball-to-goal, and alignment angles whose conventions are explicit.
- Capture-time and clock convention, measured latency/age, uncertainty, and explicit handling of missing, occluded, stale, or ambiguous observations. Report what is known rather than retaining an apparently valid old track.
- A replay fixture and live sample showing both robots, ball, goals, and derived geometry, with known failure cases.

## Proof and integration handoff

Give gameplay reproducible sample state, clips, calibration data, and measurement results against annotated or physical references. Report calibration/heading/ball-position and velocity errors, identity swaps, misses/false detections, and timing; select acceptance thresholds with gameplay from measured motion needs. Keep recorded-clip and live results distinct and identify the code/config/camera setup used.

Agree on field/body frames and units, robot IDs, goal identity, angle signs, capture-time/age, validity, and uncertainty semantics. Coordinate how state loss affects command interruption and recovery. Overhead fall-posture observations may help gameplay; they do not establish a sufficient fall detector or prove physical get-up.

iPhone capture method, computation placement, estimator/filter, update rate, state schema, and transport remain open. Use the [hardware procedure](../../foundation/HARDWARE_CHECKLIST.md) when coordinating trials involving moving robots.

## Overhead adapter — November 19

The [overhead ingress](../behaviors/overhead_ingress.py) maps Ryan’s UDP packets
to the [executor’s frame contract](../pupper_fc/README.md#shared-state-executor-and-deliberate-game-resume).
It checks the producer, session and calibration revision, converts capture times
to the robot’s monotonic clock using a measured offset bound, and emits normalized
observations to the existing [ROS bridge](../behaviors/README.md#continuous-gameplay-bridge)
when publication is selected. Camera capture latency requires its own measurement.

For a shared match, the internal adapter now supports required participant IDs
and original `captured_at` on each robot row. A missing or stale peer holds
motion even while other tracks and the enclosing frame remain fresh. Rows may
repeat the same pose/capture, but cannot change pose at that capture; the frame
capture must be at least as recent as every included row. Do not carry a cached
ball or robot forward with a new image's timestamp. Missing observations stay
missing. This is an internal normalization contract, not a required Ryan schema.

The bridge keeps action authority separate from frames. Intents have a maximum
0.5-second lease. After a camera hold, resumption needs a fresh field capture
and intent. An interrupted finite shot gets a new action ID. Operators own arm
and goal resume. The next tracking evidence is a measured run across the actual
mount, network and Pi. [Local integration evidence](../../foundation/results/perception-harness-integration-20261119.json).
