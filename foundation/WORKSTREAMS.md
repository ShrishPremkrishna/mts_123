---
date: 2026-10-30T14:33
tags: [cs123, pupper, project]
---
# Lab 6 workstream handoff

This document records the Lab 6 submission questions and evidence. For project goals, owners and the current split, see [the final project README](../final_project/README.md).

## Minimum Lab 6 action list

Current source: [Lab 6 handout](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/lab-6.html), inspected November 2. Current course schedule says due November 9. Saved originals are in the class library; source text is reference material, not authorization to run its exercises.

1. **Reuse existing tools and Sit policy.** Select one runnable release; fix only failures that block required behavior. Record final `lab/test.py` output. Reuse training scores, references and actual prior observations.
2. **Basic follower.** Integrate Lab 5 Steps 2–4: person target, approach and stop distance. Conversation leaves it following; a new motion stops following and takes control. Lab 6 explicitly excludes color recovery and obstacle avoidance.
3. **Minimum Spider.** Reuse the best existing clip/walker pair before training anything new. Demonstrate low walking in commanded directions, body above ground, stopping and return. The handout explicitly permits slower-than-commanded speed. Follow is refused while Spider. Course score/viewer checks still apply to newly trained assets.
4. **Required trick exercise.** Q8 asks for swim twice and explaining reentry/final posture. The selected R4 runtime now supports two playbacks, observed with a controlled standing return. Preserve that bounded evidence; no need to certify all supplied tricks.
5. **Four videos.** Q7: movement including sidestep/run/circle, refusal, follow while answering, follow takeover. Q12: Sit/hold/both paws/stand and camera-side handshake. Q16: Spider entry/walk/refusal/exit. Q19: newcomer conversation, personality and camera understanding.
6. **Student submission.** Complete Q1–Q20, actual requested experiment observations/training comparisons, personality examples/newcomer feedback, reflection (at most one page), contributions, fork link, shared videos and course checkoffs. Do not substitute engineering tests for missing observations or invent answers. The review deck/template contain numbering errors; use handout Q7/Q12/Q16/Q19 as four distinct videos.

Recording setup: Part 0 describes on-Pupper kiosk/mic/speaker without a laptop for videos. That setup worked in R5; recharge the wireless microphone before resuming the remaining recordings. No extra polish is required if the course demonstration works adequately.

## Voice recordings to finish — November 3 checkpoint

The wireless microphone battery died; no further voice filming should be presumed complete. Recharge it before the next session. Pupper’s own microphone, speaker and explicit hands-free app have now worked together. Keep the recordings already made.

| Deliverable | Evidence now | Next action |
|---|---|---|
| **Q7: movement, refusal, following and takeover** | Short running and movement have prior watched physical evidence, but this full voice video is **unfinished**. The final filming attempt was deferred when the mic died. | Record forward/backward, both turns, sidestep, a short run, a full circle where clear, an over-limit request with a spoken refusal, then following while answering a question and takeover by another motion. The spoken following-stop announcement fix is staged on the Pi but has not run there; verify that it finishes before the new motion. Preserve any usable footage already captured. |
| **Q12: Sit/stay/paws and camera-side handshake** | Shrish confirms the first recorded sequence was interrupted and needs a rerecord. Logs retain the ten-second Sit and both paws, plus Stop cancelling the standing-exit wait; camera-side handshake is not established by this run. | **Rerecord the interrupted sequence in a later robot session.** Shrish owns footage review; do not request files or repeat the review question. Include the complete standing return and required handshake from Pupper’s left without naming a paw. |
| **Q16: Spider walking/refusal/exit** | **Reported captured:** voice entry → forward two seconds → left two seconds → standing exit is confirmed in the server log. Shrish observed body clearance while walking and a spoken refusal to follow. | Keep and check the existing video for audible speech and complete framing. Belly-supported entry/exit is intentional in this reference; belly support during walking would fail the criterion. No automatic retake. |
| **Q19: newcomer conversation** | **Not started:** no person outside this quarter’s CS123 class was available. | Arrange that participant; record conversation showing personality, a trick, and a genuine camera-understanding question. The preparation instruction also asks for five minutes with someone who has not met Pupper, without coaching them. |

Use Pupper’s own screen/microphone/speaker for the demos. Current hands-free mode has no wake word; spoken Stop disarms listening until an explicit reconnect, so place Stop at the end of a recording block. R3 remains available throughout. Do not treat a spoken acknowledgment as physical-completion evidence.

The swim-twice exercise (**Q8**) was observed successfully with a standing return; the handout does **not** require a separate swim video. The student’s explanation is still needed. Q1–Q20 answers, training comparisons, reflection/contributions/fork link and course checkoffs remain separate submission work. Exact operational state and source versions stay in the recovery record.

## Written submission handoff

The videos are only part of the submission. Use the current source and actual
observations; the reinforced harness differs from the class starter, so passing
regressions cannot stand in for the requested starter experiments.

| Questions | Evidence or next action |
|---|---|
| **Q1, Q2, Q6** | Recover the original description/result ablations, two before/after prompt-rule observations, and the experiment bypassing `prepare_to_move`. Those observations are not established in the inspected records. If reconstructed with an isolated fake starter, label them reconstructed; do not disable production protection. |
| **Q3, Q5, Q15** | Use the current [movement tools](../gemini_pupper/lab/part2_walk.py), [basics](../gemini_pupper/lab/basics.py), [registry](../gemini_pupper/robot/tool_registry.py) and [robot API](../gemini_pupper/robot/robot_api.py) to explain parameter enforcement and immediate admission versus later controller execution. State which source version the trace describes; filmed R5 predates the new following announcement. |
| **Q4** | Explain periodic speed publication and asynchronous waiting from source. The requested five-second walk/late-failure observation still needs exact evidence; a fake cancellation exercise must not be described as a physical fall or as observed model belief. |
| **Q8–Q10, Q13** | Swim twice has watched evidence. Explain reentry/final posture, clip continuity, mirror and reverse from [motion operations](../pupper-mjlab/src/mjlab/tasks/tracking/config/pupper/motion_ops.py) and the Sit/Spider clip definitions beside it. Mirror swaps paired legs and negates all three joint axes under this model's conventions. Kinematic continuity alone does not prove dynamic stability. |
| **Q11, Q14** | The [sprint report](results/harness-sprint-20261103.json) points to exact selected-policy identities and numerical evidence. Q11 now has a [reviewed nominal comparison](../artifacts/lab6/q11-sit-replay-20261103/comparison.json): replay **WOBBLES** (7.188cm/32.720° worst errors), selected policy **HOLDS UP** (1.492cm/3.697°). Disclose the gain difference and simulation scope. Q14 uses the recovered StableGait weights and selected Spider manifest, including its separately recorded torso-support penalty; no new training is needed just to assemble the table. |
| **Q17, Q20** | Use the actual personality prompt and recorded conversations, then supply the missing newcomer feedback and the student's own reflection (at most one page). Do not invent spoken examples or personal judgments. |
| **Q18** | The [previous complete output](../artifacts/validation/harness-sprint-20261103/follow-handoff/course-integrated.txt) passed all 166 checks before ordered following. The current harness intentionally adapts one provided Stop assertion: a queued acceptance remains historical acceptance, while its exact job must later be **cancelled** and the follower off. Use the [current 166-check passing output](../artifacts/validation/harness-sprint-20261103/offline-composition-release/integration/final/course.log) for the submitted source and disclose this adaptation; do not present it as an unchanged starter test suite. |

Video requirements remain in the table above. Add actual video links, the fork
link, contributions and course checkoffs before calling the submission complete.

## Next acceptance steps

1. On the next exclusive hardware handoff, follow the [hardware checklist](HARDWARE_CHECKLIST.md). Verify the release, an app-only update and rollback, a compound voice command with ordered following, the audible handoff, Stop and queue cancellation, the failure cue and a short timing capture. If voice fails, export the timing from that trial before changing anything.
2. For the proposed first offline harness check, replay valid and lost observations plus an explicit Stop and takeover through the existing registry against a fake queue. Verify that stale input ends travel and cancelled jobs stay cancelled. [Soccer behavior contracts](../final_project/README.md)
