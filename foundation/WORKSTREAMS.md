---
date: 2026-10-30T14:33
tags: [cs123, pupper, project]
---
# Pupper: current scope and proposed sprint

Updated November 2, 2026 from Shrish's explicit reset. This replaces the October 30 three-workstream plan. Earlier scope remains in Git history, not an active backlog. The action checklist is approved and implementation is active. Soccer implementation is deferred until the general agentic harness is dependable and extensible. Implementation starts with environment cleanup and the general harness.

## Current goal and ownership

**Our lane:** finish the required minimum of Lab 6 and make the existing voice/tool/control system cohesive, dependable and easy to extend. Soccer offense/defense remains future project context; no ball tracking, kick/block training or soccer integration belongs in this sprint.

- Teammate: gait, sprinting, jumping and physical movement capabilities.
- Teammate: Optional Lab 3 implementation and generated motion work.
- Shrish + this agent: Lab 6 minimum, workspace/deployment cleanup, general agent harness, skill integration and validation. Soccer software ownership is a later decision.
- Lecture 7 analysis is deferred at Shrish's request. Actual slides are not posted at the inspected link; do not invent their contents.

Do not make Lab 6 completion depend on pretty tricks, perfect heading, all twelve routines, generated dances, general navigation, fall recovery or broad policy optimization. Optional Lab 3 is no longer our completion requirement.

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
| **Q7: movement, refusal, following and takeover** | Short running and movement have prior watched physical evidence, but this full voice video is **unfinished**. The final filming attempt was deferred when the mic died. | Record forward/backward, both turns, sidestep, a short run, a full circle where clear, an over-limit request with a spoken refusal, then following while answering a question and takeover by another motion. The spoken following-stop announcement fix is reviewed offline but not deployed; verify that it finishes before the new motion. Preserve any usable footage already captured. |
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

## Action checklist for the next sprint

1. **Fix the workspace.** Make the engineering checkout the primary app project; expose course/artifact folders and verify practical write/network access. Reuse working Python/Node/ROS environments and authenticated coding tools. Done when normal edits and focused checks work without recurring path/permission setup.
2. **Unify deployment.** Maintain one existing prepare/deploy/start/status/stop workflow; reconcile source and actual app/native/policy versions; preserve rollback. App/browser restarts are separate from motor initialization. Done when a release can be reproduced and identified without manual dated-path assembly.
3. **Simplify continuity.** Keep short goals, enduring rules and current state in the existing entry points. Move historical detail out of default reading; use compact result/decision records and agreed telemetry retention. Done when another agent can resume from a few short files without replaying the chat.
4. **Finish core harness behavior.** Consistent typed tools, authoritative status and accepted/running/completed/failed/cancelled outcomes; one coherent voice path; correct compound ordering and control ownership. Address reproducible latency, reconnect, over-refusal and duplicate/missing feedback problems. Keep prompts concise and derive capabilities/state from code.
5. **Prove extensibility.** Add a small test skill through a handler and declaration, without scattered frontend/speech edits. Exercise composition, failure, timeout, Stop and takeover of a continuous behavior using fake execution, then existing physical skills. Reuse the registry/queue; no new agent framework.
6. **Close minimum Lab 6.** Basic follower, usable Spider, swim-twice exercise, required Sit/movement/vision demonstrations, four videos, Q1–Q20 and checkoffs as detailed above. Reuse trained assets; polish and OptionalLab3 are excluded.
7. **Accept and freeze the baseline.** Run one concise end-to-end acceptance set covering conversation, compound commands, skill transitions, Stop, failure/reconnect and adding a tool. Capture useful timing and repeat core scenarios on the selected release. Fix observed blockers; save the accepted release, operating limits and short run instructions. Then discuss soccer implementation.

Order: workspace/deployment/continuity first; core harness and minimumLab6 proceed in parallel where possible; final acceptance joins them. Subagents/Claude handle independent implementation and review tasks, with one owner of robot commands. No wholesale rewrite or broad training sweep.

## Deferred soccer design context — outside this sprint

The current architecture is a foundation model choosing tools plus local controllers, not an end-to-end VLA that directly predicts robot actions. Keep that separation:

- Model: conversation, user intent, tactical objective, skill selection and honest explanation.
- Local task controller: timestamped ball tracking, target updates, approach/alignment, predicted goal crossing, bounded response and target-loss handling. Runs independently of cloud speech.
- Motion skills: walking/turning, kick/block and entry/exit supplied by policies or suitable conventional control. One owner commands motion at a time.

`approach_ball` and `defend_goal` are continuous feedback behaviors, not long queues of stale move instructions. Global Stop interrupts them; another motion explicitly takes ownership. A kick clip is a candidate until ball contact/displacement and robot recovery are measured.

Tool contract: typed bounded arguments; prerequisites/operating limits; start and cancellation behavior; accepted/running/completed/failed/cancelled status; measured outcome separate from requested intent. Advertise only installed capabilities. Keep prompts short and derive capabilities/state from runtime data; code enforces invariants.

Ball state needs timestamp, coordinate frame, track identity/confidence, bearing/range where calibrated and uncertainty. Defense adds predicted crossing point/time and reachability. Initial ball, surface, goal dimensions and speed envelope are still to be agreed. Self-play is a stretch goal after an offense and defense baseline works.

## Environment findings and intended fixes

- The user already corrected the project folder. This running chat still starts in the coursework checkout and requires approved commands for foundation writes; those commands work. Use the engineering checkout in a future correctly rooted chat without asking the user to repeat the project setup. Preserve coursework main.
- Prefer Mac for agent/browser/development and Pi for robot runtime and hardware-specific work. Moving everything to the Pi is not currently justified; it would tie development to robot availability. Earlier read-only SSH timed out; a later November2 hostname/uptime check succeeded. No services or motors were started by that check.
- The maintained packager now pins reviewed source/build inputs and produces an app delta for the existing stage/persistent-session workflow. Native, IMU and policy identities remain separate. App-only attachment is tested offline; its first live upgrade/rollback acceptance is still required. HEAD is not presumed physically tested.
- `.venv-api/bin/python` now provides the stable API-test entry point while reusing the existing tested environment; no speculative reinstall or venv move is needed.
- One stable browser origin, visible release identity and microphone/speaker status should eliminate repeated key/configuration/setup ambiguity.

## Minimal memory and evidence

Reuse three entry points: repository `AGENTS.md` for short enduring rules, this file for scope/acceptance, and `session_status.py`/`results/interruption-recovery.json` for current release, active work and next action. The capability map and dated results are reference material to open only for a concrete question.

Normal run record: intended test, selected release, result, useful timing and next decision. Record compact structured request/job events. Keep detailed sensor/video traces for diagnostics, baselines and important demonstrations; rotate routine telemetry with an agreed retention policy. Existing archives remain preserved until retention is agreed; no automatic deletion now. Do not create another report for every successful command.

Use subagents or Claude for independent code/review tasks with clear ownership. Integrate one result and retire the subtask. A fresh session should start from this small record rather than replaying the conversation.

## Current execution

The final overnight offline release is reviewed, built, packaged and preserved. Ordered following, terminal-failure feedback, fresh replies after idle Stop, extension composition, transport failure handling and guarded browser-exit Stop have software evidence. A bounded Debug timing export supports diagnosis without retaining conversation or media. All owned jobs are finished; no deployment or new robot evidence was added. Next: supervised acceptance of this release, Q7, the interrupted Q12 rerecord, Q19 and student submission. Q16 remains reported captured. Soccer and gait optimization stay deferred. Use the recovery record for the selected package; a superseded candidate is explicitly held after a reproduced rollback-compatibility bug.
