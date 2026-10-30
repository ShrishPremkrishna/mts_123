---
date: 2026-10-30T03:22
tags: [cs123, workflow]
---
# Pupper foundation working agreement

This is the engineering branch, not graded coursework. Preserve the separate
coursework checkout. The goal is a reproducible, measured class foundation;
research candidates and simulation passes are not physical capabilities.

## Start with current state

Run `python3 foundation/scripts/session_status.py` from this checkout. Read
`foundation/results/interruption-recovery.json` only if more detail is needed.
That compact file owns current activity, blockers, environment and next actions.
The archived recovery record and dated result reports are historical evidence,
not instructions to restart old work. Read only evidence relevant to the task.
`foundation/README.md` owns capability explanations; the hardware checklist owns
the next physical session procedure. Do not maintain another parallel status log.

## Keep the work narrow and useful

- One person/agent owns each changed file. Delegate concrete independent work;
  root integrates once authors freeze their sources. Agents do not commit or
  operate the robot unless root explicitly delegates that ownership.
- Root alone normally owns SSH, credentials, hardware and live API access.
  No credentials in repository, logs, tool output or artifacts.
- Use the tested Python path in current state; numerical tests may need the
  separate numerical environment. Do not reinstall dependencies speculatively.
- Before more training, name the hypothesis, comparison metric and stopping limit.
  Prioritize a repeatable floor baseline over broad new policy sweeps.
- Native integration checks must use the exact executable, names, configuration
  and message contract selected for the trial. Test inputs without motors first.
- Fix the smallest observed defect. Reuse current tools; do not introduce another
  wrapper, job framework or versioned staging directory without a concrete need.
- Run the tests affected by a change. Run a broader regression once sources are
  frozen, and repeat only after relevant changes or a new unresolved concern.
  Test counts are evidence about software, never a substitute for native behavior.
- Keep tool output bounded: summarize counts, distinct values and short errors;
  preserve raw output in files. Use bounded subprocess timeouts and a fresh SSH
  connection for read-only recovery; do not spin on an unavailable robot.
- Do not use human supervision time for offline packaging, dependency discovery,
  large builds or tests that could have been completed without the robot.

## Robot, evidence and stopping

Follow the user's current authorization and the applicable course hardware rules.
Existing authorization does not need repeated permission questions; ask for actual
physical readiness/observations when the planned motion requires them. No motion
in an offline session, no automatic homing on reconnect, and no widened limits or
gains. Hardware initialization itself may home. Newly trained policies need the
applicable course approval before first hardware execution.

Preserve frozen trial inputs and complete outputs; source in `artifacts/` is
historical evidence, not the editable implementation. Current reusable hardware
helpers live in `foundation/hardware/`. Do not repackage edited inputs as if an
older passing run tested them. Missing remote output is a recovery task, not a
passed run. Before future long compute, demonstrate collection of one verified
checkpoint and keep collector health visible; preserve a complete final archive
before releasing that runtime. Periodic snapshots remain partial.

At a handoff, update current state with exact blocker, next action, active owners,
process/exit evidence and pending archives. Leave completed agents and schedules
idle unless the user asked for continuing work. A fresh session should resume in
minutes without rereading the entire conversation or launching anything.
