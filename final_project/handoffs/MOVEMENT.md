# Movement

November 19, 2026 · [Pupper FC brief](../README.md)

Give two autonomous Puppers controllable forward, backward and lateral travel, turning and braking on the roughly 8 × 12-foot field. Shrish owns movement and gameplay integration, Tej owns the leg kick, and Ryan owns overhead perception. For a shot, the walker must reach a measured pose behind the ball that puts it on the selected paw's line, settle, then hand the legs to the kick. The [project brief](../README.md) records the current agreement.

These are optional workstream notes. An incoming agent's role follows its user's current task and relevant source, not this document's title. Current activity and access are in the [checkpoint](../../foundation/results/interruption-recovery.json).

## Immediate working baseline

Start with the available gait and [native controller](../../pupper_gait_deploy/README.md), [simulation/training](../../pupper-mjlab/README.md), and [harness](../../gemini_pupper/README.md). Establish what existing motion actually achieves, including sideways travel and interruption, then tune or train against demonstrated gaps. Existing source is a starting point, not proof of soccer readiness.

## Required deliverables

- A measured movement envelope: achieved speed in ±bodyX and ±bodyY, turning, acceleration, braking time/distance, stability, and limiting failures.
- Measure elapsed time from the last walk command to foot contact, then to the first verified walking response. Record Stop or loss of tracking during the handoff separately. Tej's faster return is the first comparison candidate in the [kick procedure](../ROBOT_BUILD_AND_TEST_HANDOFF.md#operator-kick-after-the-straight-baseline). After a goal, robots must reposition and wait for ball replacement with software still running.
- If get-up is retained, coordinate with its separately assigned owner. Gameplay owns detection/preemption/coordination; physical-motion ownership remains open. Demonstrate expected arena fall postures, including left/right and other reachable configurations, before claiming recovery. Ordinary standing, a fall-stop, or retrying stand does not establish get-up. Report a safe stop after failed recovery as a failure to recover.

## Proof and integration handoff

Give gameplay and shooting the measured envelope, entry/exit and interruption expectations, recovery posture results, representative clips, trial records including misses/falls, and policy/config/release identities. Distinguish simulation, source checks, staged files, controller receipts, and observed physical outcomes.

Agree on body/field frames, units, robot IDs, command ownership/interruption, and how state age, validity, and uncertainty limit motion. Schemas and transport remain open, as do one versus multiple policies and tuning versus training.

Before physical trials, follow the [hardware procedure](../../foundation/HARDWARE_CHECKLIST.md): confirm one operator and readiness, simulate changed motion, obtain applicable course approval for new policies, and preserve joint/current limits.
