# Shooting

November 19, 2026 · [Demo testing order](../PLAN.md#current-sprint--november-19)

The selected demo shot uses Tej’s left or right leg kick. Shrish owns the field-guided approach and the handoff from walking. The combined release drives one walking policy into either kick, observes the applied clip and resumes walking on a new native activation. Tej owns physical kick and recovery testing. [Selected release and simulation evidence](../pupper_fc/README.md#walking-and-kicking-in-one-release)

## Fast transition candidate

Tej’s u5 release uses direct kick clip entry, a short walker restart from standing (0.2 s initialization and 0.3 s fade) and a separate slow stand-up path. The Pi log has four successful jobs. Each includes a commanded 0.305 m approach and reports restoration of locomotion.

| u5 configuration | Left, whole job | Right, whole job |
| --- | ---: | ---: |
| Default 0.15 s settle, exit clip at 2.6 s | 5.76 s | 5.63 s |
| Zero settle, exit clip at 2.1 s | 4.98 s | 5.03 s |

The native controller log isolates the interval from activating the kick to activating the walker. It fell from 3.25 s on both legs to 2.70 s left and 2.75 s right. Tej reports 12.5 s for the whole job on the older u3 release. His default-u5 estimate of 2.3 s from walk end to impact uses the clip’s expected contact frame. Actual impact, direction and the first resumed walking step were unmeasured in these four jobs. The later 16:58 Pi records supersede the branch note that the faster settings were untried. A newer kickfast4 log contained two approach-only dry runs. [Pi record and log identities](../../foundation/results/staged-field-session-20261119.json) · [Tej’s branch report](https://github.com/ShrishPremkrishna/mts_123/blob/30b84a096deab79a43a966224ccd571c21f5662a/gemini_pupper/docs/kick-ball.md#timing-of-the-kick-all-kick-tools)

Keep the fast transition as a selective integration candidate. The donor can count a return to the idle clip as completion, and the optional walker settings need binding to the active launch as well as the tool process. Main’s operator kick checks the requested side/clip and verifies walker restoration. Its kick blend is still 0.5 s initialization plus 1.0 s fade. The combined profile runs the stock gait weights in the guarded `gemini_walk` controller. It transfers leg control to `trick_leg_kick` for the chosen clip, then requires a new walker binding before another movement. [Operator kick](../../gemini_pupper/lab/operator_kick.py) · [Combined release procedure](../ROBOT_BUILD_AND_TEST_HANDOFF.md#combined-walk-and-leg-kick-release)

## Approach and ball tracking

Ryan supplies field observations. The normalized camera feed currently consumes measured robot/ball tracks and configured goals. FC’s tactics choose each robot’s role and FC’s pose controller already turns field position error into body-frame forward, lateral and yaw velocities. For leg shooting, give that controller a paw-specific target, refresh it with each fresh observation, reduce speed near arrival and allow turning in place. Predict moving-ball contact using both the time needed to reach the stance and the wind-up. Once the kick is committed, freeze the side and issue it once. Use the onboard camera for people/obstacles. A close-range ball check is a later option if top-down measurements miss the paw’s contact window.

Tej’s timed tool uses travel = gain × commanded speed × elapsed time after the start delay. The donor fitted gain 1.31 and delay 0.14 s at 0.20 m/s for its walker. Main’s stock gait reached about 20 cm in one second and 38 cm in two at the same commanded speed. Use camera-measured remaining distance and heading to finish an approach. Timing calibrations belong to the tested gait, speed and surface. [Donor’s pose mode](https://github.com/ShrishPremkrishna/mts_123/blob/30b84a096deab79a43a966224ccd571c21f5662a/gemini_pupper/docs/kick-ball.md#L105-L118) · [Marked floor trials](../../foundation/results/staged-field-session-20261119.json)

## What earns a place in the demo

1. Tej qualifies both walking entries on a pinned release. Record the actual clip, stability, return and first walking step. Use the [operator procedure](../ROBOT_BUILD_AND_TEST_HANDOFF.md#operator-kick-after-the-straight-baseline) for release selection and applicable course clearance.
2. With the intended ball, mark its forward and lateral position relative to each foot. Measure the contact window and walk-end-to-impact time on video, shot direction, distance, misses and walking recovery.
3. Approach using fresh overhead observations. With the chosen yaw toward goal, subtract the rotated, measured ball offset for that paw from the field ball position to obtain the robot-center target. Slow and align, dispatch one kick, verify the clip and restored walker, then choose the next play. Hold before committing if the observations or strike authority expire. Retain physical Stop during the action.
4. Start with a stationary ball and the more repeatable foot. Add rolling balls using measured wind-up and camera age, then rehearse role changes and two-robot play. Record the entire approach, actual ball travel and resumed movement in the same take.
