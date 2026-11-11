# Pupper final project

As of November 10, 2026, Shrish is revising the final-project direction and will provide the new brief after the context cleanup. This folder contains documentation. The earlier discussion concerned robot soccer; its specific demo, architecture and division of work are recorded as [November 4 planning](PLANNING_20261104.md).

## Available implementation

- [Gemini Pupper](../gemini_pupper/README.md) provides conversation, tool discovery, queued actions, cancellation, job status and operator controls.
- [Lab 5 code](../follow_me_pupper/README.md) provides camera calibration, a Hailo detector, person tracking and following.
- [Training](../pupper-mjlab/README.md) and [native deployment](../pupper_gait_deploy/README.md) provide policy development and execution.
- [The evidence index](../foundation/README.md) identifies observed capabilities, failed experiments and selected releases. [The dated checkpoint](../foundation/results/interruption-recovery.json) records source, staging and last observed hardware state.

These components provide a starting point for exploration. Current APIs and source describe how they connect. The new project brief will establish the intended demo, hardware, interfaces, ownership and acceptance criteria.

## Earlier team work

The [perception handoff](PERCEPTION_HANDOFF.md) and [gait handoff](GAIT_HANDOFF.md) describe the November 4 assignments. Their tasks and proposed interfaces are historical until reconfirmed. At that point Shrish owned the harness and videos, and a fourth teammate owned the [Lab 6 written submission](../foundation/submission/lab6/README.md).

On November 10, GitHub also showed unmerged teammate branches `tej/opt-lab1-implicit-compliance` at `f9aa7b9` and `tej/opt-lab-3` at `1d896d8`. The latter changes secret-gate code and adds stretch-dance assets. Branch presence establishes available source, while submission status and physical acceptance require their own evidence.
