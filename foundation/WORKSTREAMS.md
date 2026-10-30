---
date: 2026-10-30T14:33
tags: [cs123, pupper, project]
---
# Pupper Foundation Workstreams

Build a dependable Pupper foundation for the final project by finishing and improving the class capabilities through three parallel engineering workstreams. Each owner delivers working software **and robot evidence**; final integration should connect capabilities already demonstrated individually.

The ambition is roughly 90–100% coverage of the usable class capabilities on our standard Pupper. This is a scope target, not a measured completion percentage. Missing tactile hardware and unresolved jump research remain explicit exclusions. Required Spider behavior belongs inside this plan.

| Owner | Workstream | Owns |
|---|---|---|
| Shrish + agent | 1. Lab 6 and interactive skills | Gemini, tools/UI, command sequencing, Sit/shake, supplied tricks and Optional Lab 3 |
| Teammate 1 + agent | 2. Locomotion and sim-to-real | Live gait tuning, Lab 4/Optional Lab 2 comparisons, simulator corrections and Spider policies |
| Teammate 2 + agent | 3. Perception and autonomy | Lab 5 following, target selection/recovery, color and obstacle-detour options |

## 1 Lab 6 and interactive skills

**Goal:** A coherent robot experience: ask Pupper to move, follow, perform a skill or describe its surroundings, and get correct execution and status.

**Starting point:** Most tool/queue code exists. API standing worked on the supported robot. Sit and a modest generated motion have simulation evidence. Actual Gemini tests exposed claims of actions that were never dispatched; voice and physical integration still need work.

| Deliverable | Proof of completion |
|---|---|
| Complete Lab 6 movement and conversation: walk, turn, curve, wait, stop, refusal and camera understanding | A live voice demonstration with actual tool calls and robot behavior matching the request. Accepted, completed, cancelled and failed actions are reported accurately. Exercise interruption; a spoken stop claim alone does not count. |
| Interactive Sit/stay/shake and supplied tricks | Robot enters Sit, holds at least 10 seconds, offers each paw, stands and returns to walking. Name the supported trick repertoire, demonstrate each included trick and exit, and list deferred tricks explicitly. |
| Optional Lab 3 personality, context, hidden-trick gate and generated motion | Demonstrate the intended conversation behavior, allowed/denied hidden-trick access, and the generated motion with a clean return. Keep secrets outside the repository. |
| One integrated behavior interface | Following can be interrupted by a new command. The Spider pair from stream 2 works through Lab 6, including entry, walking, stop and return. No stale queued action starts after cancellation. |

**Without the robot:** Finish tools/UI/prompt behavior; test actual Gemini against fake execution; fix sequencing and cancellation; evaluate or retrain Sit/generated-motion candidates where needed; prepare complete demonstration sequences. Fake text tests do not establish live speech interruption.

**With the robot:** Exercise direct tools, then live speech; test full skill sequences and transitions; verify camera understanding and physical stopping.

**Ownership and handoff:** Owns Sit/generated-motion training and Lab 6 action arbitration. Receives the follower service from stream 3 and Spider assets from stream 2; does not independently rewrite those components. Hands over one working Lab 6 entry point, configuration and demonstrated repertoire.

**Start in:** `gemini_pupper/`, its [foundation guide](../gemini_pupper/docs/foundation.md), [Gemini findings](results/gemini-realtime-live-reviewed.json), [Sit evidence](results/lab6-sit-comx02-reviewed.json) and [generated-motion evidence](results/generated-motion-trained-review.json).

## 2 Locomotion and sim-to-real

**Goal:** Make Pupper move reliably, establish useful measured gait profiles, and deliver the required Spider behavior. Own the training decisions needed to close observed simulation-to-robot gaps.

**Starting point:** Policies, training pipelines and a browser gait tuner already exist. Some candidates improve tracking while worsening slip/effort. Spider retains simulation failures. The tuner changes explicit IK trajectories, not neural-policy weights; improved references can feed subsequent RL training.

| Deliverable | Proof of completion |
|---|---|
| Usable live gait tuning | Existing tuner respects controller ownership, stop behavior, fresh feedback and bounded target changes. Demonstrate enabling, changing a gait and releasing control on the robot. |
| Stock baseline and useful gait comparisons | Measure actual speed, drift, slip, body stability and stopping on stated surfaces. Compare prepared profiles and recovered neural candidates; identify the default and any useful alternatives. Keep the baseline if a candidate has no demonstrated advantage. |
| Targeted Lab 4 and Optional Lab 2 improvements | Use measurements to choose reference, reward, sensing or dynamics changes. Preserve reproducible training/export settings and comparisons, including rejected candidates. A higher training reward alone is insufficient. |
| Required Spider clip and walking pair | Demonstrate entry, low-stance walking, stopping and return to default without relying on belly-supported locomotion. Supply reviewed/approved policies, references, controller settings, operating limits and transition requirements to stream 1. |

**Without the robot:** Repair the existing tuner bridge; prepare batches of valid gait profiles; analyze recordings; evaluate policies and full transitions in simulation; train targeted comparisons; prepare course approval evidence. The bridge currently publishes hold targets when “Send” is off, lacks a feedback-age watchdog and replaces active gait caches without a fresh blend—address these before live use.

**With the robot:** Run focused gait/profile comparison blocks; collect joint targets versus measurements, IMU, battery and available actuator data; film actual travel and contact behavior. Test approved candidates and reproduce the problems that inform the next training round. Keep existing hardware limits/gains unless a separately justified change is reviewed.

**Ownership and handoff:** Owns locomotion/Spider models and reference design. Stream 1 owns their user-facing activation and transitions. Coordinate shared training-library edits with stream 1. Hand over versioned assets and measured operating limits, not just checkpoint filenames.

**Start in:** `pupper-gait-tuner/`, `ik_heuristic_walking_lab/extension/`, `pupper-mjlab/`, [Optional Lab 2 experiments](../pupper-mjlab/configs/optional2/README.md) and [Spider findings](results/spider-encoder-native-reviewed.json).

## 3 Perception and autonomy

**Goal:** Dependable person following and the provided optional perception behaviors, exposed as one service that Lab 6 can control.

**Starting point:** Actual camera/Hailo person detection worked. Following, target persistence, color recovery, timed detours and stale-detection handling are implemented; their combined physical behavior remains to be established.

| Deliverable | Proof of completion |
|---|---|
| Reliable basic following | Robot follows a selected person, turns appropriately and stops at the configured distance in recorded scenarios. State the tested lighting, distance and speed range. |
| Target loss and recovery | Test occlusion, leaving/re-entering view and two people crossing. Retain the intended target or stop/reacquire according to the stated recovery policy; wrong-person following is a failure. Target loss or stale detections must not sustain uncontrolled following. |
| Optional color recovery and obstacle detours | Demonstrate each option separately, then together where useful. Record failure conditions. The provided timed detour is not general collision-free navigation. |
| Lab 6 follower service | Activate/deactivate repeatedly; stop and relinquish motion control when Lab 6 requests another behavior. Exercise stalled frames/detection and service restart behavior. |

**Without the robot:** Build replay cases from recorded frames/detections; tune target selection and recovery; test stale data and service lifecycle; fix failures and prepare scenario batches. Reuse the existing viewer/service rather than building another interface.

**With the robot:** Calibrate against actual camera conditions; measure following distance and response; test occlusion, person crossings, color recovery and detours; verify physical stopping and Lab 6 takeover. Use one live camera/Hailo owner at a time.

**Ownership and handoff:** Owns follower internals and activate/deactivate behavior. Stream 1 owns command arbitration and calling that service. Deliver one tested service/configuration, replay cases and known environmental limits.

**Start in:** `follow_me_pupper/`, its [integration guide](../follow_me_pupper/README.md) and [camera evidence](results/live-camera-perception-reviewed.json).

## Shared working agreement

- Work concurrently in separate branches/worktrees with clear file ownership. Share the robot in focused blocks; prepare batches of configurations/scenarios before each block. Isolate variables when diagnosing a failure, rather than serializing all development.
- The integration agent owns shared runtime/deployment fixes. The current joystick discovery/shutdown issue gates motor testing, not offline work in any stream. Only one stack owns motor commands at a time.
- Each claimed capability needs the exact code/model/configuration, repeatable launch/test instructions, logs and video, measured results and stated limitations. Use at least three clean repetitions of each agreed core scenario as an initial acceptance screen, plus relevant stop/interruption cases; broader reliability testing follows.
- Newly trained policies need the applicable course approval. Homing starts on the stand, with a present operator. This document assigns work; it does not authorize unattended motor tests.
- Integrate working increments throughout. Required Spider behavior and physical evidence cannot be deferred while declaring all three streams complete.

## What remains after these workstreams

Final consolidation should cover installation from a clean checkout, combined behavior sequences, longer sessions, wider conditions and regression testing. Audit the remaining Lab 1–3 physical exercises/extensions individually; reviewed PD/FK/IK code does not establish every hardware exercise. Optional Lab 1 requires the missing tactile/single-leg setup. Reliable jumping and further rough-terrain gains remain research goals if the existing experiments still fail. These exclusions prevent an honest claim of literal 100% completion.

## Agent startup context

Engineering checkout: `/Users/shrish/workspace/cs123-foundation`, branch `pupper-foundation`. Preserve coursework in `/Users/shrish/workspace/cs123`. Snapshot: 30 October 2026; read current state before acting.

1. Read repository `AGENTS.md`; run `python3 foundation/scripts/session_status.py` from the engineering checkout.
2. Use [current state](results/interruption-recovery.json) for activity/blockers and [capability evidence](README.md) for completed work. Read only relevant reports; do not restart old experiments from historical notes.
3. Robot access is `ssh mts`. Standard class Pupper; no tactile rig. Credentials stay outside chat, logs and Git. Reuse the tested environments and existing artifacts.
4. Confirm your workstream/file ownership with the integration owner before editing shared code or deploying. Source handouts live under `/Users/shrish/Library/Mobile Documents/iCloud~md~obsidian/Documents/vaulted/Fall2026Classes/Cs123/References/`; consult the relevant original handout for course requirements.
