# Shooting

November 16, 2026 · [Pupper FC brief](../README.md)

**Outcome:** Repeatable directional shots on a bright ground ball from stationary and moving starts. Two autonomous Puppers play on a roughly 8×12 ft field; overhead perception supplies robot/ball/goal geometry. The [project brief](../README.md) owns current assignments: Shrish owns movement, headbutts and gameplay integration with assisting agents; Tej owns foot-kick work, and Ryan owns perception.

These are optional workstream notes. An incoming agent's role follows its user's current task and relevant source, not this document's title. Current activity and access are in the [checkpoint](../../foundation/results/interruption-recovery.json).

## Immediate working baseline

Follow the TA-guided progression: use the existing gait to align behind the ball and run through it with the front bumper. Next explore a specialized primitive with stationary and moving entries, then ball-in-simulation training. Keep the baseline playable while evaluating later stages; advancing training complexity should address a measured limitation. These stages have separate evidence, and later training is not a prerequisite for the first working game.

Start from [simulation/training](../../pupper-mjlab/README.md), [native deployment interfaces](../../pupper_gait_deploy/README.md), and [harness interruption behavior](../../gemini_pupper/README.md). These establish reusable motion components, not a verified soccer simulator or shot.

## Required deliverables

- A runnable shot with documented starting conditions, inputs, contact behavior and exit to locomotion; compare specialized candidates against the existing-gait baseline.
- Stationary and moving-entry results. When adding ball-contact simulation, report assumptions and transfer limitations separately from physical shooting results.
- Measured contact success, ball direction/distance, misses, falls, and robot stability after contact. Show align → contact → regain controlled motion → resume, including interrupted and failed attempts.
- Preconditions and interruption behavior gameplay can use. Coordinate fall stopping with gameplay; physical get-up scope/ownership remain open for discussion.

Foot kicking and a dedicated goalie posture are stretch work.

## Proof and integration handoff

Provide gameplay with a reproducible invocation/example, primitive inputs and outputs, entry/exit expectations, observed duration, failure cases, clips/trial records, and policy/config/release identities. Keep simulated results, controller receipts, and physical observations distinct. After a goal, gameplay must be able to reposition the robot and await ball replacement without routine rehoming or restarting software.

Agree on robot IDs, body/field frames, angle units, capture-time/age, validity/uncertainty, and command interruption. Scripted, learned, or combined motion, policy count, schema, and transport remain open.

Use the [hardware procedure](../../foundation/HARDWARE_CHECKLIST.md) before physical trials: one operator, simulated changed motions, applicable course approval for new policies, and preserved joint/current limits.
