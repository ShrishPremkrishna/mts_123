# Perception — Ryan

November 12, 2026 · [Pupper FC brief](../README.md)

**Outcome:** Turn overhead mounted iPhone video into usable shared state for two autonomous Pupper soccer players on a roughly 8×12 ft field. Track both robot IDs, positions and headings, the bright ground ball's position and velocity, goals, and derived shot angles, with capture timestamps, age, uncertainty, and validity.

Coordinate the mount, field/robot AprilTags, goals, and ball with Shrish, who owns the arena. Shrish and assisting agents own gameplay integration.

These are optional workstream notes. An incoming agent's role follows its user's current task and relevant source, not this document's title. Current activity and access are in the [checkpoint](../../foundation/results/interruption-recovery.json).

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

## Internal adapter candidate — November 17

The [executor contract](../pupper_fc/README.md#shared-state-executor-and-deliberate-game-resume)
and [ROS bridge](../behaviors/README.md#continuous-gameplay-bridge) can now be
exercised with synthetic same-host inputs while the real feed is unavailable.
This does not prescribe Ryan's wire format. The adapter will map an actual sample
into metres, counterclockwise heading, fixed calibrated goals, explicit missing
tracks, and capture times with a known clock/age relationship. It must preserve
capture times rather than label a delayed image as newly captured.

The first executable bridge accepts a named local Linux monotonic clock only;
cross-device camera clock conversion is still unimplemented. Frames never renew
motion authority: action intents have a separate maximum 0.5-second lease.
After a camera hold, fresh field capture and freshly issued intent are required;
an interrupted finite shot gets a new action ID. Operator arm and goal resume
remain outside perception ingress. One real sample plus its capture-clock
convention is the next integration input; recorded replay can precede hardware.
