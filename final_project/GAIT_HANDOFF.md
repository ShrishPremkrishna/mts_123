# Gait teammate handoff

Own Optional Lab 2 and the movement Pupper needs for soccer. The attacker needs a fast approach, a **grounded header** and a controlled stop. The goalie needs to move sideways before a rolling ball crosses the goal line and stay upright for the next shot. A taped lane and a marked lateral line are enough to start.

Work from the team repository on `main`. Use `pupper-mjlab/` for gait training and `final_project/motion/` for soccer-specific profiles, skill adapters and the measured limits you hand to the other owners. [Team repository](https://github.com/ShrishPremkrishna/mts_123)

## Close Optional Lab 2

Use the [course handout](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-2.html) as the submission checklist. The deliverables are a fork containing the capture pipeline, gait tables and configs, plus one technical report. Start with the existing [experiment protocol and run records](../pupper-mjlab/configs/optional2/README.md).

| Handout part | Evidence to assemble |
| --- | --- |
| 0–1 · References and fast gait | One page on gait generation, capture format/resampling and branch-selection constants. Explain the authored gait parameters, show the reference video, and compare against the stock fast-slot trot on high-speed translation/yaw error, torque² and slip. |
| 2 · Bootstrap | Link the committed capture code. Give each round’s tracking error, torque², slip, achieved speed and video. Explain how you selected steady cycles and why you stopped. |
| 3 · MixedGaits | Show transitions across the fast threshold in simulation and the Bumpy robustness result. For an accepted multi-gait candidate, film deployment and measure real top speed versus commanded speed. Identify the largest sim-to-real discrepancy. |
| 4 · Mystery | Report what the jump experiments actually produced. Simulation footage and failed landings are valid findings for this open-ended part. |

**Current gap.** The fast and Bumpy comparisons exposed tradeoffs. A short stock run was observed on hardware, but actual speed was unmeasured. Keep the trained candidate’s deployment video and measured speed open until you have them. Check that every video and run link is accessible from the submitted report. [Flat comparison](../foundation/results/optional2-corrected-flat-review.json) · [Bumpy comparison](../foundation/results/optional2-checked-bumpy-review.json)

## Establish the soccer baseline

1. **Start offline with one gait.** Drive the shipped policy with forward, reverse, left, right and turn commands, then transitions between them. Its native interface already accepts body-frame velocity in m/s and yaw rate in rad/s. Lab 6 currently routes sideways run requests to walking. Preserve that distinction when reporting results. [Native controller](../pupper_gait_deploy/src/neural_controller.cpp) · [Lab 6 movement tools](../gemini_pupper/lab/part2_walk.py)
2. **Measure one supervised floor block.** Film marked distances from a fixed camera. Record command start, brake request and settled stop against the video. Repeat each trial on the same surface and retain misses and falls. Use the selected profile’s limits and coordinate a single robot operator. [Robot session procedure](../foundation/HARDWARE_CHECKLIST.md)

   | Trial | What to measure |
   | --- | --- |
   | Forward sprint and reverse run into a stop | Requested and achieved speed in each direction, time to reach speed, braking distance and time, final heading and stability. |
   | Left and right bursts, then a reversal | Distance reached one second after command in each direction, time and overshoot to stop or reverse, slip and recovery. Give perception and harness owners a small table of reachable distances by direction and elapsed time. |
   | Aligned header, when the ball is available | From behind a stationary ball, drive the nose through it toward a taped gate and brake. Record contact speed, ball direction/distance, whether it cleared the gate, and whether Pupper recovered. Base gait trials can start without a ball or field. |

3. **Train against a measured gap.** Start with a velocity-conditioned policy trained without a ball. If it fails a soccer trial, name the metric to improve, then compare one change against the stock policy under identical conditions. A specialist sprint policy must also pass entry from ordinary movement and exit back to braking. Keep the stock release available for rollback.
4. **Evaluate jumping.** Inspect loading, whole-body flight, landing and sustained recovery in replay. Prior Mystery trials produced only 0, 0, 1 and 1 strict flight-and-recovery events across four seeds. Before a Mystery hardware attempt, require repeatable simulated recovery and the TA approval specified by the course. A failed screen closes the evaluation with evidence and leaves physical jumping open. [Jump evidence](../foundation/results/optional2-mystery-reviewed.json)

**At the team review,** share the course fork/report and any outstanding item, one measured movement table with policy/config identity and trial counts, representative clips, and the limiting failure. Give the harness owner the demonstrated speeds, braking distances and lateral reach it can actually use.
