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

## Prepare without the robot

1. Finish the relevant software checks and web build before supervision. The
   existing course tests and `gemini_pupper/lab/test_skill_sequences.py` exercise
   actual installed descriptors through fake ROS: Sit, ten-second stay, both paws,
   exit/walk, finite endpoints and the hidden generated trick. They establish
   software sequencing, not physical balance or Live speech behavior.
2. Select the exact controllers for the session. The persistent launch in
   `foundation/hardware/lab6_session.launch.py` is one available workflow; an
   equivalent reviewed workflow is fine. It requires an explicit selection:
   `PUPPER_SESSION_SKILLS=''` selects stock locomotion only; leaving the variable unset refuses startup.
   `PUPPER_SESSION_SKILLS='sit_stay_shake'` adds only Sit. Set the same explicit
   value for launch and API server, and use the same `PUPPER_GEMINI_DIR` checkout.
   The generated runtime manifest binds selection, policies, descriptors and
   configuration. A mismatched/unscoped API refuses the selected profile; missing
   selected controllers are not spawned on demand. This is configuration, not
   hardware approval. Reference jumps and research walking pairs are excluded.
3. Preserve the tested stock caps, gains, physical Joy settings and stop paths.
   The prepared profile and recent server fixes still require native integration
   checks on the Pi; a local test pass is not deployment evidence. Generic
   `start.sh`/`stop.sh` are not interchangeable with the reviewed staged setup.
   Have source/model identities, recording destination and owned shutdown ready.
4. Obtain the applicable course approval for new trained Sit/generated policies
   before their first hardware trial. Use the existing
   [Sit sequence](results/sit-interactive-default-return-reviewed.json),
   [finite return](results/finite-tricks-default-return-reviewed.json),
   [lying return](results/lying-tricks-later-default-return-reviewed.json) and
   [generated-motion](results/generated-motion-trained-review.json) evidence.
   Nominal simulation success does not justify blanket approval of every trick.
5. Prepare a small voice request set against fake execution: correct dispatch,
   unsupported requests, interruption, and the three Optional Lab 3 conversation
   habits. Check wrong/right secret access and reconnect behavior. Keep production
   secrets outside Git, prompts and logs; the test-only password is not a robot
   credential. Fake text checks do not establish live speech interruption.

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
   Include cancellation with pending work; voice narration is not stopping evidence.
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
