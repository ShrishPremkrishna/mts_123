# Shooting

November 19, 2026 · [Demo testing order](../PLAN.md#current-sprint--november-19)

The selected demo shot uses Tej’s left or right leg kick. Shrish owns the field-guided approach and the handoff from walking. The current automatic executor uses a forward run-through in simulation. The physical demo needs an explicit leg-kick action before autonomous shooting.

## Fast transition candidate

Tej’s u5 release uses direct kick clip entry and a short walker restart from standing. The optional walker profile sets 0.2 s initialization and 0.3 s fade, with a separate slow stand-up path. Tej reports left and right no-ball trials following a commanded 0.305 m walk. Whole-job time fell from 12.5 s on u3 to 5.7 s on u5. The 2.3 s from walk end to nominal paw contact and 2.2 s from that phase to walker readiness use the clip’s expected contact frame. Those trials establish neither observed impact time nor physical response to the next walk command. [Tej’s timing table and release settings](https://github.com/ShrishPremkrishna/mts_123/blob/30b84a096deab79a43a966224ccd571c21f5662a/gemini_pupper/docs/kick-ball.md#timing-of-the-kick-all-kick-tools)

Keep the fast transition as a selective integration candidate. The donor can count a return to the idle clip as completion, and the optional walker settings need binding to the active launch as well as the tool process. Main’s operator kick checks the requested side/clip and verifies walker restoration. Its kick blend is still 0.5 s initialization plus 1.0 s fade. The field-game bridge currently requires the stock walker, while the kick tool requires Gemini’s walker and trick controller. Use one compatible runtime that gives walking, kick and Stop exclusive ownership in turn. [Operator kick](../../gemini_pupper/lab/operator_kick.py) · [Field-game bridge](../behaviors/fc_executor_bridge.py)

## Approach and ball tracking

Ryan supplies field observations. The normalized camera feed currently consumes measured robot/ball tracks and configured goals. FC’s tactics choose each robot’s role and FC’s pose controller already turns field position error into body-frame forward, lateral and yaw velocities. For leg shooting, give that controller a paw-specific target, refresh it with each fresh observation, reduce speed near arrival and allow turning in place. Predict moving-ball contact using both the time needed to reach the stance and the wind-up. Once the kick is committed, freeze the side and issue it once. Use the onboard camera for people/obstacles. A close-range ball check is a later option if top-down measurements miss the paw’s contact window.

Tej’s timed tool uses travel = gain × commanded speed × elapsed time after the start delay. The donor fitted gain 1.31 and delay 0.14 s at 0.20 m/s for its walker. Main’s stock gait reached about 20 cm in one second and 38 cm in two at the same commanded speed. Use camera-measured remaining distance and heading to finish an approach. Timing calibrations belong to the tested gait, speed and surface. [Donor’s pose mode](https://github.com/ShrishPremkrishna/mts_123/blob/30b84a096deab79a43a966224ccd571c21f5662a/gemini_pupper/docs/kick-ball.md#L105-L118) · [Marked floor trials](../../foundation/results/staged-field-session-20261119.json)

## What earns a place in the demo

1. After agreement, reproduce both walking entries on a pinned release. Record the actual clip, stability, return and first walking step. Use the [operator procedure](../ROBOT_BUILD_AND_TEST_HANDOFF.md#operator-kick-after-the-straight-baseline) for release selection and applicable course clearance.
2. With the intended ball, mark its forward and lateral position relative to each foot. Measure the contact window and walk-end-to-impact time on video, shot direction, distance, misses and walking recovery.
3. Approach using fresh overhead observations. With the chosen yaw toward goal, subtract the rotated, measured ball offset for that paw from the field ball position to obtain the robot-center target. Slow and align, dispatch one kick, verify the clip and restored walker, then choose the next play. Hold before committing if the observations or strike authority expire. Retain physical Stop during the action.
4. Start with a stationary ball and the more repeatable foot. Add rolling balls using measured wind-up and camera age, then rehearse role changes and two-robot play. Record the entire approach, actual ball travel and resumed movement in the same take.
