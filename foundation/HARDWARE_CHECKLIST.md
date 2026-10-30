---
date: 2026-10-27T15:31
tags: [cs123, foundation]
---
# Next robot session

The objective is a repeatable stock floor baseline, then Lab 6 integration.
Current state and exact environment paths live in
[interruption-recovery.json](results/interruption-recovery.json). Do not repeat
completed inventory, builds, policy recovery or training unless a dependency
changed. The [original discovery checklist](results/history/HARDWARE_CHECKLIST-before-20261030.md)
remains historical reference for backups and inventory details.

## Finish before asking for supervision

1. Recover the complete failed floor run identified in current state. Inspect
   distinct Joy node names/namespaces, endpoint count, physical input receipts,
   joystick child log and shutdown-coordinator exit/logs. Preserve and hash the
   complete output before cleanup. The generic publisher error is not a diagnosis.
2. Use the canonical input-only preflight in `foundation/hardware/floor_stock/`
   once reviewed. It must use the selected joystick executable, device name,
   node name and parameters, with no motor manager or hardware plugin loaded.
   Confirm a single expected Joy source, fresh valid input, and complete child
   shutdown. Passive diagnostics never prompt for buttons. The separate
   Options/R3 exercise needs current host-wide no-motor confirmation; an isolated
   ROS domain does not isolate the physical gamepad. A disconnected controller
   may require the user's PS button later; report that dependency without starting
   motor software.
3. Explain and resolve the prior shutdown fallback. Passing local tests or a
   graph preflight alone does not establish this. Reuse the current verified
   environment and unchanged stock policy, limits and gains. Restore the reviewed
   installed-library/deployment identity checks before releasing the canonical
   motion gate; a source inventory alone is insufficient. Freeze and review
   the corrected selected input set once, after all owners finish editing.
4. Have the exact launch, bounded stop/cleanup, capture destination and prior
   baseline available. Run relevant offline tests before the user arrives.
   No broad rebuilds or new experimental policy selection during this session.

## Supervised sequence

1. Confirm Pupper is on the stand, legs clear, controller connected, battery
   healthy and the user beside the physical power disconnect. Announce the
   specific bounded trial and use the user's authorization; do not repeatedly
   ask to approve unchanged software preparation. Homing itself moves motors.
2. Home only on the stand. Keep walking inactive for placement. Ask for actual
   floor placement/readiness before enabling the physical controller path;
   never infer placement from elapsed time or an old confirmation.
3. Demonstrate stock standing and physical R3 stopping. Record actual active
   controller, zero-command path, sensors, stop transition, cleanup and the
   user's observation. Inactive status does not prove physical stillness or
   zero torque. Repeat only to check reproducibility or resolve a specific issue.
4. After standing/stopping succeeds, run one short conservative forward movement,
   then stopping. Use existing allowed command ranges and limits. Change one
   thing at a time; extend duration or add turning only after observed success.
5. Use the same baseline for direct Lab 6 API standing, movement and cancellation.
   Test controller handoffs and stop ownership before introducing live Gemini.
6. With fake-only Gemini checks complete, try a small supervised request set and
   compare requested, accepted, completed and observed behavior. Add following,
   target loss/reacquisition and optional color/detour behavior in separate trials.

Stop on strain, abnormal motion, heat, sensor loss or uncertain ownership. If a
new issue needs substantial software work, release the user from supervision,
leave the robot stopped, and prepare the next concrete trial offline.

## What remains excluded

New trained Sit/generated policies need applicable course approval before their
first hardware run. Spider walking and standing jumps retain known simulation
failures and are not promoted. Optional Lab 1 tactile work requires the missing
single-leg/sensor setup. Do not raise limits or gains to obtain a pass.

## Finish a session

Preserve complete run outputs, exact input identities and the operator's actual
observations. Record missing evidence explicitly. Update the one current-state
record with the next blocker, process/exit state, and pending archives. Leave no
implicit job running and no schedule active unless the user requested it.
