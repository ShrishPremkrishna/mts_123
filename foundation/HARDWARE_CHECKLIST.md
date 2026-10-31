---
date: 2026-10-27T15:31
tags: [cs123, foundation]
---
# Next Workstream 1 robot session

Extend the [observed basic floor controls](results/lab6-floor-basic-20261030.json)
to Live Gemini and approved interactive skills. Exact current deployment,
ownership and unfinished work belong in
[interruption-recovery.json](results/interruption-recovery.json). Earlier failed
floor-input trials are historical; do not restart their timed choreography.

## Ready offline; next Pi integration

The [offline rehearsal](results/workstream1-offline-rehearsal-20261030.json) is
complete: software checks/build, actual Chrome against the fake API, repeated
Gemini comparisons, and installed-descriptor sequencing through fake ROS. Reuse
those prepared cases. They do not establish physical balance, real browser
microphone/camera behavior or in-flight speech interruption.

1. Inventory teammate changes before integrating the final Python, UI and selected
   session profile on the Pi. Preserve the previously reviewed native overlay;
   no C++ changed in this offline iteration. Rebuild native packages only if
   compatibility requires it, then verify the selected controller/protocol markers
   and repin exact source, executable, configuration and policy hashes. Preserve
   stock caps, gains, physical Joy settings and stop paths. Generic
   `start.sh`/`stop.sh` are not interchangeable with the reviewed staged setup.
2. Select the exact controllers for the session. The persistent launch in
   `foundation/hardware/lab6_session.launch.py` requires an explicit selection:
   `PUPPER_SESSION_SKILLS=''` selects stock locomotion only; unset refuses startup.
   `PUPPER_SESSION_SKILLS='sit_stay_shake'` adds only Sit. Set the same selection
   for launch and API, using the same `PUPPER_GEMINI_DIR` checkout. The runtime
   manifest binds selection, policies, descriptors and configuration; mismatched
   APIs are refused and missing controllers are not spawned on demand. Reference
   jumps and research walking pairs are excluded. Have recording and owned
   shutdown ready before motion.
3. Obtain applicable course approval before new trained Sit/generated policies'
   first hardware trials. Use the existing
   [Sit sequence](results/sit-interactive-default-return-reviewed.json),
   [finite return](results/finite-tricks-default-return-reviewed.json),
   [lying return](results/lying-tricks-later-default-return-reviewed.json) and
   [generated-motion](results/generated-motion-trained-review.json) evidence.
   Simulation and native receipts do not establish physical performance.

The final Gemini 3.1 Flash Live default uses HIGH thinking. The diagnostic's
`--thinking-level MINIMAL` reproduces the comparison; other models are unchanged
by default. HIGH improved tool selection but retained two unsupported posture
claims and one unsupported sensor claim. One text request took 8.776 s from
submission to the fake Stop response; physical/audio Stop timing remains
unmeasured. Use actual action receipts and observation; keep direct Stop and
physical R3 independent of speech. `current_action` is current work, `last_outcome` is history,
and physical pose/speed/stopped fields remain unknown.

## Supervised work blocks

1. Begin with a read-only inventory and confirm who owns the robot now. Check
   placement, battery, controller and physical power-stop readiness. Reuse the
   established environment; repeat input diagnostics only when something changed
   or actual evidence requires it. The prior 16.49 V / 0% display is consistent with the software
   model's five-cell, 3.3 V-per-cell floor. The actual pack was not independently
   inspected; that estimate is not proof of battery health.
2. If a fresh hardware launch is needed, home on the stand with legs clear. Keep
   walking inactive for placement. When the operator is ready on the floor, one
   physical Options press releases the existing gateway; there is no chat-timed
   countdown. Reuse that homed session between actions. API Stop cancels work and
   requests zero velocity while holding the walking controller; physical R3 latches
   its abort. Explicit manager-first shutdown ends the owned hardware session.
3. Briefly confirm the current direct-control baseline, then spend the block on
   missing behavior: bounded arc/wait sequences and Live requests whose actual
   tool calls, status and physical outcomes agree. Reuse the demonstrated short
   forward/back/turn commands instead of repeating the entire discovery process.
   Include actual microphone input, camera understanding and in-flight speech
   interruption/cancellation with pending work. These remain unvalidated; voice
   narration is not stopping evidence.
4. With the selected approved Sit profile, request `sit_stay_shake`, hold at least
   ten seconds, `shake_left`, `shake_right`, and the interactive `stand_up` exit;
   then make a bounded walk and stop. Also check that an ordinary move from Sit
   uses the exit first. Compare observed paw/pose behavior with clip receipts.
   Use a separate bounded interruption case; do not force a recovery through strain.
5. Select supplied finite tricks by their recorded behavior and limits. A
   standing-ending trick returns to locomotion; a lying-ending trick holds that
   endpoint until a later explicit request. The existing finite `spider` is not
   the required research Spider walking pair. Known high-speed/robustness limits
   remain relevant when choosing what to exercise.
6. For approved `secret_dance`, demonstrate denial while locked, unlocking without
   motion, explicit playback and return, then relocking on Stop. The installed
   motion is a gentle sway/rise, not a complex dance. Test Optional Lab 3's context
   habits by voice, including a different request resetting the handshake streak
   and reconnect losing conversational history. Receive following and Spider
   integration from their owners rather than replacing their workstreams.

Batch prepared cases while the user is available. Stop on unexpected strain,
loss of feedback or uncertain ownership; if substantial coding is needed, finish
that offline instead of consuming supervision time. No higher limits or gains
are needed to establish the next baseline.

## Finish and hand off

Stop the owned session, verify process exits, and preserve the actual observations
and exact inputs. Check archive contents as well as hashes: the last session's
archive matched byte-for-byte but had empty finalization files and a truncated
telemetry tail. Keep originals and label any recovered metadata separately.
Update the one current-state record; leave no implicit running job or schedule.
Missing tactile hardware, unpromoted Spider walking and unresolved standing-jump
reliability remain explicit exclusions, not hidden completion claims.
