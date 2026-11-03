---
date: 2026-10-30T14:33
tags: [cs123, pupper, project]
---
# Pupper: current scope and proposed sprint

Updated November 2, 2026 from Shrish's explicit reset. This replaces the October 30 three-workstream plan. Earlier scope remains in Git history, not an active backlog. Current request is to finalize the action checklist. Soccer implementation is deferred until the general agentic harness is dependable and extensible. Next implementation starts with environment cleanup, not soccer-specific work.

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
4. **Required trick exercise.** Q8 asks for swim twice and explaining reentry/final posture. Assets exist, but swim is not in the currently selected five-skill runtime. No need to certify all supplied tricks.
5. **Four videos.** Q7: movement including sidestep/run/circle, refusal, follow while answering, follow takeover. Q12: Sit/hold/both paws/stand and camera-side handshake. Q16: Spider entry/walk/refusal/exit. Q19: newcomer conversation, personality and camera understanding.
6. **Student submission.** Complete Q1–Q20, actual requested experiment observations/training comparisons, personality examples/newcomer feedback, reflection (at most one page), contributions, fork link, shared videos and course checkoffs. Do not substitute engineering tests for missing observations or invent answers. The review deck/template contain numbering errors; use handout Q7/Q12/Q16/Q19 as four distinct videos.

Recording setup: Part 0 describes on-Pupper kiosk/mic/speaker without a laptop for videos. Our inspected voice evidence is laptop-based; settle this setup before filming. No extra polish is required if the course demonstration works adequately.

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

- This chat starts in the Obsidian vault; the engineering repo is outside writable roots. Put `/Users/shrish/workspace/cs123-foundation` in a proper local project as primary. Keep course notes in the vault and coursework main untouched. Folder access may require a project UI change/new correctly rooted session; conversation permission does not itself change a running sandbox.
- Prefer Mac for agent/browser/development and Pi for robot runtime and hardware-specific work. Moving everything to the Pi is not currently justified; it would tie development to robot availability. Earlier read-only SSH timed out; a later November2 hostname/uptime check succeeded. No services or motors were started by that check.
- Three launch paths currently exist, and the last physical release combines separate app/native/IMU builds. Reconcile source and deployed components explicitly; do not assume HEAD was physically tested. App-only updates must not rebuild/rehome native control.
- Python environments work but one is buried in an old evidence folder. Give the maintained commands a stable dependency/configuration entry point without speculative reinstalling or blindly moving a venv.
- One stable browser origin, visible release identity and microphone/speaker status should eliminate repeated key/configuration/setup ambiguity.

## Minimal memory and evidence

Reuse three entry points: repository `AGENTS.md` for short enduring rules, this file for scope/acceptance, and `session_status.py`/`results/interruption-recovery.json` for current release, active work and next action. The capability map and dated results are reference material to open only for a concrete question.

Normal run record: intended test, selected release, result, useful timing and next decision. Record compact structured request/job events. Keep detailed sensor/video traces for diagnostics, baselines and important demonstrations; rotate routine telemetry with an agreed retention policy. Existing archives remain preserved until retention is agreed; no automatic deletion now. Do not create another report for every successful command.

Use subagents or Claude for independent code/review tasks with clear ownership. Integrate one result and retire the subtask. A fresh session should start from this small record rather than replaying the conversation.

## First decision after this review

Start with project access/deployment when implementation resumes. Soccer choices wait until harness acceptance. Obtain the teammates' actual follower/Spider/motion assets as needed; do not assume their current progress from the old workstream assignments. Current code/release/evidence are in the recovery record.
