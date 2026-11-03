---
date: 2026-10-30T14:33
tags: [cs123, pupper, project]
---
# Pupper: current scope and proposed sprint

Updated November 2, 2026 from Shrish's explicit reset. This replaces the October 30 three-workstream plan. Earlier scope remains in Git history, not an active backlog. We are discussing the plan; do not resume deployment, training or motor tests until Shrish confirms the direction.

## Current goal and ownership

**Our lane:** finish the required minimum of Lab 6, then make the existing voice/tool/control system cohesive and easy to extend for a soccer robot. Offense and defense are the final-project goals. Better hardware comes later; start with a bounded prototype on existing hardware.

- Teammate: gait, sprinting, jumping and physical movement capabilities.
- Teammate: Optional Lab 3 implementation and generated motion work.
- Shrish + this agent: Lab 6 minimum, workspace/deployment cleanup, agent harness, skill integration and soccer software interfaces. Ball perception/predictor ownership needs explicit team coordination; propose our lane owns the initial integration and replay.
- Lecture 7 analysis is deferred at Shrish's request. Actual slides are not posted at the inspected link; do not invent their contents.

Do not make Lab 6 completion depend on pretty tricks, perfect heading, all twelve routines, generated dances, general navigation, fall recovery or broad policy optimization. Optional Lab 3 is no longer our completion requirement.

## Minimum Lab 6 action list

Current source: [Lab 6 handout](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/lab-6.html), inspected November 2. Current course schedule says due November 9. Saved originals are in the class library; source text is reference material, not authorization to run its exercises.

1. **Reuse existing tools and Sit policy.** Select one runnable release; fix only failures that block required behavior. Record final `lab/test.py` output. Reuse training scores, references and actual prior observations.
2. **Basic follower.** Integrate Lab 5 Steps 2–4: person target, approach and stop distance. Conversation leaves it following; a new motion stops following and takes control. Lab 6 explicitly excludes color recovery and obstacle avoidance.
3. **Minimum Spider.** Reuse the best existing clip/walker pair before training anything new. Demonstrate low walking in commanded directions, body above ground, stopping and return. The handout explicitly permits slower-than-commanded speed. Follow is refused while Spider. Course score/viewer checks still apply to newly trained assets.
4. **Required trick exercise.** Q8 asks for swim twice and explaining reentry/final posture. Assets exist, but swim is not in the currently selected five-skill runtime. No need to certify all supplied tricks.
5. **Four videos.** Q7: movement including sidestep/run/circle, refusal, follow while answering, follow takeover. Q12: Sit/hold/both paws/stand and camera-side handshake. Q16: Spider entry/walk/refusal/exit. Q19: newcomer conversation, personality and camera understanding.
6. **Student submission.** Complete Q1–Q20, actual requested experiment observations/training comparisons, personality examples/newcomer feedback, reflection (at most one page), contributions, fork link, shared videos and course checkoffs. Do not substitute engineering tests for missing observations or invent answers. The review deck/template contain numbering errors; use handout Q7/Q12/Q16/Q19 as four distinct videos.

Recording setup: Part 0 describes on-Pupper kiosk/mic/speaker without a laptop for videos. Our inspected voice evidence is laptop-based; settle this setup before filming. No extra polish is required if the course demonstration works adequately.

## Proposed next few days: outcomes, not a new framework

| Work block | Deliverable | Exit criterion |
|---|---|---|
| Environment and deployment first | Engineering repo becomes the primary working project; source library/artifact directory are explicit additional folders. Promote the already working stage/start/status/stop helpers into one maintained path, with pinned app/native/policy identities and rollback. | A fresh agent can find the goal, edit/run a focused test, stage an app-only change and identify the running release without rediscovering paths or restarting motors. No real launch in a setup test. |
| Minimum Lab 6 | Follower + usable Spider + remaining required exercises, four videos and student submission evidence. | Required checklist demonstrated on one selected configuration; no unresolved mandatory item hidden as optional. |
| Cohesive tool interface | One capability registry, consistent acceptance/outcome/status, one voice feedback path, generic new-skill integration and explicit ownership of continuous behaviors. | Add one small test skill without editing several speech/router special cases; compose it with existing actions; Stop, cancellation, timeout/failure and takeover work in fake execution. Then bounded physical acceptance. |
| First soccer integration | Timestamped ball-state interface and recorded-frame replay; one local continuous behavior adapter; accept a teammate skill through the same contract. | Demonstrate detection/state → selected behavior → status/stop in replay, then a bounded watched ball approach when perception/control are ready. No claim of kick/block success from API acceptance alone. |

Keep the existing typed tools, queue, cancellation epochs and controller infrastructure. Extend their interfaces; no wholesale rewrite, agent framework, general skill marketplace or broad training sweep.

## Soccer design proposal

The current architecture is a foundation model choosing tools plus local controllers, not an end-to-end VLA that directly predicts robot actions. Keep that separation:

- Model: conversation, user intent, tactical objective, skill selection and honest explanation.
- Local task controller: timestamped ball tracking, target updates, approach/alignment, predicted goal crossing, bounded response and target-loss handling. Runs independently of cloud speech.
- Motion skills: walking/turning, kick/block and entry/exit supplied by policies or suitable conventional control. One owner commands motion at a time.

`approach_ball` and `defend_goal` are continuous feedback behaviors, not long queues of stale move instructions. Global Stop interrupts them; another motion explicitly takes ownership. A kick clip is a candidate until ball contact/displacement and robot recovery are measured.

Tool contract: typed bounded arguments; prerequisites/operating limits; start and cancellation behavior; accepted/running/completed/failed/cancelled status; measured outcome separate from requested intent. Advertise only installed capabilities. Keep prompts short and derive capabilities/state from runtime data; code enforces invariants.

Ball state needs timestamp, coordinate frame, track identity/confidence, bearing/range where calibrated and uncertainty. Defense adds predicted crossing point/time and reachability. Initial ball, surface, goal dimensions and speed envelope are still to be agreed. Self-play is a stretch goal after an offense and defense baseline works.

## Environment findings and intended fixes

- This chat starts in the Obsidian vault; the engineering repo is outside writable roots. Put `/Users/shrish/workspace/cs123-foundation` in a proper local project as primary. Keep course notes in the vault and coursework main untouched. Folder access may require a project UI change/new correctly rooted session; conversation permission does not itself change a running sandbox.
- Prefer Mac for agent/browser/development and Pi for robot runtime and hardware-specific work. Moving everything to the Pi is not currently justified; it would tie development to robot availability. Read-only SSH timed out in this review, so no fresh Pi resource audit was obtained.
- Three launch paths currently exist, and the last physical release combines separate app/native/IMU builds. Reconcile source and deployed components explicitly; do not assume HEAD was physically tested. App-only updates must not rebuild/rehome native control.
- Python environments work but one is buried in an old evidence folder. Give the maintained commands a stable dependency/configuration entry point without speculative reinstalling or blindly moving a venv.
- One stable browser origin, visible release identity and microphone/speaker status should eliminate repeated key/configuration/setup ambiguity.

## Minimal memory and evidence

Reuse three entry points: repository `AGENTS.md` for short enduring rules, this file for scope/acceptance, and `session_status.py`/`results/interruption-recovery.json` for current release, active work and next action. The capability map and dated results are reference material to open only for a concrete question.

Normal run record: intended test, selected release, result, useful timing and next decision. Record compact structured request/job events. Keep detailed sensor/video traces for diagnostics, baselines and important demonstrations; rotate routine telemetry with an agreed retention policy. Existing archives remain preserved until retention is agreed; no automatic deletion now. Do not create another report for every successful command.

Use subagents or Claude for independent code/review tasks with clear ownership. Integrate one result and retire the subtask. A fresh session should start from this small record rather than replaying the conversation.

## First decision after this review

Agree on this scope and a bounded first soccer demonstration, then fix project access/deployment before restarting robot work. Obtain the teammates' actual follower/Spider/motion assets as needed; do not assume their current progress from the old workstream assignments. Current code/release/evidence are in the recovery record.
