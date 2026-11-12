# Pupper FC

Pupper FC aims to have two autonomous Puppers play soccer in an approximately 8 × 12-foot arena. An overhead iPhone camera observes field/robot markers, a bright ground ball and two goals. Gameplay uses a 2D field model. The initial shot uses locomotion and a front bumper; foot kicking and a special goalie posture are stretch goals.

## Current agreement — November 12, 2026

- Marcus owns movement. Tej owns shooting. Ryan owns overhead perception and shooting geometry. Shrish owns gameplay and integration with assistance from agents; Shrish builds the arena.
- Autonomous fall detection, physical self-righting and resumption are separate P0 work required for the base project. Physical get-up ownership is open. The ordinary stand/restart path is not get-up. A first gameplay baseline can develop while recovery remains unfinished; recovery acceptance stays open.
- After goals, a person may replace the ball and give one resume signal. Software stays running and robots manage restart positioning. Routine rehoming or manual robot repositioning between plays is outside the intended base demo.
- Power-on should prepare the selected stack automatically with minimal setup. A supported startup pose and one deliberate start/arm action are accepted; their implementation needs verification. Other game-rule decisions can wait.
- Keep a working prototype and improve it through measured experiments. Policy count, training methods, module boundaries and transport remain implementation choices.

The [plan](PLAN.md) records outcomes, proposals, integration responsibilities and acceptance evidence. Working notes cover [movement](handoffs/MOVEMENT.md), [shooting](handoffs/SHOOTING.md) and [perception](handoffs/PERCEPTION.md).

## Implementation and access

This directory currently contains documentation, not a working soccer runtime or get-up implementation. Reusable components live in [Gemini Pupper](../gemini_pupper/README.md), [native deployment](../pupper_gait_deploy/README.md), [training](../pupper-mjlab/README.md) and the [Lab 5 follower](../follow_me_pupper/README.md). The [repository map](../README.md) lists setup and resources.

From the repository root, run `python3 foundation/scripts/session_status.py` for dated task, branch and access state in [the session checkpoint](../foundation/results/interruption-recovery.json). Verify current ownership and authorization before hardware work.

The November 4 [planning](PLANNING_20261104.md), [gait handoff](GAIT_HANDOFF.md) and [perception handoff](PERCEPTION_HANDOFF.md) are historical and do not assign current project work. Tej's optional-lab branches remain separate; their last observed identities are in the [November 10 snapshot](https://github.com/ShrishPremkrishna/mts_123/blob/acd705d9207a024cdaf98c4dd368feba41a6ed4e/final_project/README.md).
