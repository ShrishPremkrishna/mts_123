# Pupper FC

Pupper FC aims to have two autonomous Puppers play soccer in an approximately 8 × 12-foot arena. An overhead iPhone camera observes field/robot markers, a bright ground ball and two goals. Gameplay uses a 2D field model. The initial shot uses locomotion and a front bumper; foot kicking and a special goalie posture are stretch goals.

## Current agreement — November 12, 2026

- Marcus owns movement. Tej owns shooting. Ryan owns overhead perception and shooting geometry. Shrish and the lead agent own gameplay and integration; Shrish builds the arena.
- Autonomous fall detection, physical self-righting and resumption are required for the base project. The existing ordinary stand/restart path does not provide this capability.
- After goals, a person may replace the ball. Software stays running and robots manage their restart positioning. Routine rehoming or manual robot repositioning between plays is outside the intended base demo.
- Power-on should prepare the selected stack automatically with minimal setup. Motor initialization and match-start behavior still need agreement and verification.
- Keep a working prototype and improve it through measured experiments. Policy count, training methods, module boundaries and transport remain implementation choices.

The [plan](PLAN.md) records outcomes, proposed game rules, integration responsibilities, acceptance evidence and unresolved decisions. Teammate handoffs are [movement](handoffs/MOVEMENT.md), [shooting](handoffs/SHOOTING.md) and [perception](handoffs/PERCEPTION.md).

## Implementation and access

This directory currently contains documentation, not a working soccer runtime. Reusable components live in [Gemini Pupper](../gemini_pupper/README.md), [native deployment](../pupper_gait_deploy/README.md), [training](../pupper-mjlab/README.md) and the [Lab 5 follower](../follow_me_pupper/README.md). The [repository map](../README.md) lists setup and resources. [The checkpoint](../foundation/results/interruption-recovery.json) separates source, staging and hardware observations.

Current work is local planning/context preparation on `codex/pupper-fc-plan`. A teammate is using the robot for Lab 6. No push, Pi connection, deployment or service change is part of this block; Shrish will give the go for Pi work. Lab 6 materials remain in the [writer packet](../foundation/submission/lab6/README.md).

The November 4 [planning](PLANNING_20261104.md), [gait handoff](GAIT_HANDOFF.md) and [perception handoff](PERCEPTION_HANDOFF.md) are historical and do not assign current project work. Tej's optional-lab branches remain separate; their last observed identities are in the November 10 version of this page in Git history.
