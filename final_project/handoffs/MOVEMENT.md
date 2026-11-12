# Movement — Marcus

November 12, 2026 · [Pupper FC brief](../README.md)

**Outcome:** Give two autonomous Pupper soccer players fast, controllable movement on the roughly 8×12 ft field: body-frame forward/backward and lateral travel, turning, acceleration, braking and stability. Gameplay integration belongs to Shrish and assisting agents; shooting belongs to Tej. Get-up is separate P0 work with physical-motion ownership open.

These are optional workstream notes. An incoming agent's role follows its user's current task and relevant source, not this document's title. Current activity and access are in the [checkpoint](../../foundation/results/interruption-recovery.json).

## Immediate working baseline

Start with the available gait and [native controller](../../pupper_gait_deploy/README.md), [simulation/training](../../pupper-mjlab/README.md), and [harness](../../gemini_pupper/README.md). Establish what existing motion actually achieves, including sideways travel and interruption, then tune or train against demonstrated gaps. Existing source is a starting point, not proof of soccer readiness.

## Required deliverables

- A measured movement envelope: achieved speed in ±bodyX and ±bodyY, turning, acceleration, braking time/distance, stability, and limiting failures.
- Demonstrated entry, transitions, interruption, and exit between locomotion, shooting, stopping, and recovery. After a goal, robots must reposition and wait for ball replacement with software still running, without routine rehoming.
- Coordinate with the separate P0 get-up work when assigned. Gameplay owns detection/preemption/coordination; physical-motion ownership remains open. Demonstrate expected arena fall postures, including left/right and other reachable configurations, before claiming recovery. Ordinary standing, a fall-stop, or retrying stand does not establish get-up. Report a safe stop after failed recovery as a failure to recover.

## Proof and integration handoff

Give gameplay and shooting the measured envelope, entry/exit and interruption expectations, recovery posture results, representative clips, trial records including misses/falls, and policy/config/release identities. Distinguish simulation, source checks, staged files, controller receipts, and observed physical outcomes.

Agree on body/field frames, units, robot IDs, command ownership/interruption, and how state age, validity, and uncertainty limit motion. Schemas and transport remain open, as do one versus multiple policies and tuning versus training.

Before physical trials, follow the [hardware procedure](../../foundation/HARDWARE_CHECKLIST.md): confirm one operator and readiness, simulate changed motion, obtain applicable course approval for new policies, and preserve joint/current limits.
