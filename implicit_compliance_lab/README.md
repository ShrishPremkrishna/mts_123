# Optional Lab 1: Implicit Compliance

This folder holds the completed Optional Lab 1 ([spec](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-1.html)): the two classifier notebooks with outputs, the answers to questions 3 and 4, an ablation study, and Pupper code that pauses the Lab 3 triangle trajectory for 1 s when the joint states show contact. On the handout splits, the explicit (DenseTact image) classifier gave 98.2 % incline and 89.8 % material accuracy at its final epoch. That epoch was its best, and its last-5-epoch mean is 92.4 % and 72.5 %. The implicit (joint-state) classifier gave 95.9 % contact, 85.7 % incline and 79.0 % material accuracy. Joint states reproduce the contact labels with 93 % to 96 % accuracy under every split, also in a later recording session, and joint effort alone gives 92 % to 94 %. In a later session, terrain identity from joint states falls to chance level, but the images keep most of their incline accuracy.

## Answers

### Q3. Accuracy of each classifier

Final-epoch validation accuracy, as the notebooks print it (handout 90/10 random split):

| Classifier | Validation set | Contact | Incline | Material |
|---|---|---|---|---|
| Explicit: ResNet-50 on DenseTact images, 10 epochs | 225 contact images | — | **98.2 %** | **89.8 %** |
| Implicit: 1D CNN on joint states, W = 200 samples (≈ 0.2 s), 50 epochs | 10,797 windows. Incline and material: the 5,465 contact windows among them | **95.9 %** (precision 96.9 %, recall 95.0 %) | **85.7 %** | **79.0 %** |

The handout targets are approximately 90 % for both explicit tasks, and approximately 95 %, 90 % and 80 % for the implicit tasks.

Three qualifications apply:

1. The explicit validation accuracy is unstable. Over the last 5 epochs, the mean is 92.4 ± 4.9 % (incline) and 72.5 ± 13.5 % (material). The final epoch is the best epoch for both tasks. The implicit classifier is stable: 95.9 ± 0.1 %, 88.7 ± 1.6 % and 81.4 ± 1.5 % over its last 5 epochs. (± is the population standard deviation over the 5 epochs. All runs use one seed.)
2. For the implicit classifier, the random split overestimates accuracy. Neighbouring windows share up to 90 % of their samples (W = 200, stride 20). When the last 20 % of each recording is held out (blocked split, 30 epochs), the last-5 means fall from 86.2 % to 80.5 % (incline) and from 77.6 % to 69.4 % (material). Contact accuracy does not change (95.2 % and 94.7 %). For the explicit classifier, the epoch-to-epoch noise is larger than this effect: blocked split 84.5 % and 79.8 % at the final epoch, and 90.9 % and 80.4 % as last-5 means.
3. The two classifiers use different contact labels. The implicit labels come from `contact.csv`, which the collection program wrote at 10 Hz. The explicit images are those that the notebook's reference-image filter (`detect_contact`) keeps. Of the 6,621 images, `contact.csv` marks 3,342 as contact and the filter keeps 2,242. Of the kept images, 2,241 are also contact in `contact.csv`, and the filter drops 1,101 images that `contact.csv` marks as contact. The two labels agree for 83.4 % of the images. Thus the explicit classifier sees a stricter set of contact frames, and the two accuracies are not a like-for-like comparison.

### Q4. Comparison: tactile sensing and motor proprioception

**Contact.** The joint states alone reproduce the contact labels with 93 % to 96 % accuracy under every split, including a later recording session. But this accuracy alone does not show force sensing. A CNN on any one channel group gets 91 % to 93 % (Table C), and the position-only and effort-only CNNs give the same final accuracy (93.2 %). A threshold on the mean joint speed alone gets 75 % (Table D). A probable cause is the collection protocol: the gait was fixed, and the program held the leg still for 1 s after each detected contact. In this dataset, a still leg at a known point of the trajectory therefore indicates contact. The data does not isolate this cause.

**Effort.** On Pupper, joint "effort" is `kp * (q_command - q) + kd * (qd_command - qd)`, the deflection of the PD-controlled joint. A PD-controlled motor acts as a spring: under a load, the joint deflects until the spring torque balances the load. Effort is therefore a load signal. But it also contains the velocity term and the commanded trajectory, so it is not a pure contact force. Three results show that effort carries information that the kinematics do not:

* With linear models, the 3 window-mean efforts give 90.1 % contact accuracy. The 3 mean positions give 81.6 %, and the 3 mean speeds give 68.7 % (Table D, W = 200). A CNN closes this gap for position (93.2 % against 93.2 %).
* With equal training (20 epochs), adding effort to position and velocity increases incline accuracy from 62.8 % to 75.5 % and material accuracy from 51.7 % to 64.5 % (last-5 means, Table C).
* Effort alone gives the highest terrain accuracy of the single channel groups: 62.0 % and 51.1 %, against 54.0 % and 42.7 % for position and 37.5 % and 28.3 % for velocity. The position run varies by 6 to 8 points between epochs, so one seed does not resolve the difference between effort and position.

**Comparison with the tactile sensor (same session).** On the blocked split, the explicit classifier is better for terrain. The last-5 means are 90.9 % against 80.5 % (incline) and 80.4 % against 69.4 % (material). At the final epoch the gaps are 2.6 and 8.4 points. Because the explicit run is unstable (SD 6.5 and 7.4 points) and uses stricter contact labels, the size of the gap is uncertain. It is between approximately 3 and 10 points. For the random-split notebook model, 68 % of the implicit incline errors are between neighbouring angles. For material, confusions between rough and very rough and between rough and slip are 39 % of the errors.

**Later session.** The strongest test validates on the 20 angled recordings of the second session (15° to 75°, the four materials). The handout authors recorded that session the next morning, approximately 12 h after the angled recordings of the first session. Training uses all other recordings. These include the foam and 0° recordings, which come from the same morning as the validation data. Results:

* Contact transfers: 93.4 % with all channels and 92.5 % with effort only.
* Terrain identity from the joint states does not transfer. Incline accuracy is 22.4 %, approximately the chance level of 20 % (5 balanced classes). Material accuracy is 9.5 %, below the chance level of 25 %, so the model maps the new recordings systematically to wrong materials. The model can also predict "none" and "foam", which do not occur in this validation set.
* The DenseTact images keep most of the incline accuracy: 81.9 % at the final epoch, 70.0 ± 8.8 % as the last-5 mean (932 images). They lose material identity: 36.9 %, and 29.7 % as the last-5 mean, against a majority class of 31 %.

A possible explanation, which these experiments do not test: each incline-and-material combination has one recording per session. The joint-state terrain heads can therefore use features that are specific to a session, for example the exact touchdown point on the ramp. The image sees the local surface angle at the sensor, which stays the same between sessions.

**Conclusion.** For contact, the results agree with the hypothesis. The motors, through their PD compliance, act as implicit contact sensors. Their joint states reproduce the tactile contact labels with 93 % to 96 % accuracy, also in a later session, and effort alone gives 92 % to 94 %. But this dataset cannot separate force sensing from the protocol cues (fixed gait and a pause on contact). For terrain, the joint states carry information about slope and material within one session but not across sessions, while the tactile images keep the incline. This agrees with the success of proprioception-only reinforcement learning for locomotion, which needs contact more than terrain identity, but this lab does not test locomotion. The motors do not replace a tactile sensor for terrain identification. The deployment challenge on the robot, with a different trajectory and no pause before detection, is the direct test of contact sensing.

## Ablation results

All numbers are validation accuracy in %. Each cell gives the final-epoch value and, in parentheses, the mean ± population SD over the last 5 epochs. All runs use seed 0. Splits:

* **Random**: a seeded 90/10 split of windows or images. The notebooks and `analysis/ablation.py` order the folders differently, so their validation sets differ.
* **Blocked**: the last 20 % of each recording is validation. The implicit runs leave a one-window gap before it. The explicit run splits images by capture time with no gap.
* **Session**: validation is the 20 angled recordings of the second session (`*_new`, 15° to 75°). Training is all other recordings, including the second-session foam and 0° recordings.

Incline and material accuracy use contact windows or contact images only. Chance level is 1/6 (17 %) for each 6-class task. On the blocked validation windows, the largest class is 18.5 % (incline) and 21.2 % (material). On the session split it is 20 % (incline, 5 classes) and 27 % for the windows or 31 % for the images (material, 4 classes).

**Table A. Explicit against implicit**

| Classifier | Split | Epochs | Contact | Incline | Material |
|---|---|---|---|---|---|
| Explicit, ResNet-50 (notebook) | random | 10 | — | 98.2 (92.4 ± 4.9) | 89.8 (72.5 ± 13.5) |
| Explicit, ResNet-50 | blocked | 10 | — | 84.5 (90.9 ± 6.5) | 79.8 (80.4 ± 7.4) |
| Explicit, ResNet-50 | session | 10 | — | 81.9 (70.0 ± 8.8) | 36.9 (29.7 ± 5.6) |
| Implicit, W = 200 (notebook) | random | 50 | 95.9 (95.9 ± 0.1) | 85.7 (88.7 ± 1.6) | 79.0 (81.4 ± 1.5) |
| Implicit, W = 200 | random | 30 | 95.1 (95.2 ± 0.0) | 84.1 (86.2 ± 1.8) | 76.4 (77.6 ± 0.8) |
| Implicit, W = 200 | blocked | 30 | 94.7 (94.7 ± 0.1) | 81.9 (80.5 ± 3.1) | 71.4 (69.4 ± 2.9) |
| Implicit, W = 100 | session | 20 | 93.4 (93.2 ± 0.7) | 22.4 (24.3 ± 1.3) | 9.5 (9.8 ± 1.0) |

**Table B. Window size W** (all 9 channels, mean of epochs 16 to 20; contact / incline / material)

| W (samples, ≈ ms) | Random split | Blocked split |
|---|---|---|
| 50 | 94.3 / 79.4 / 68.1 | not run |
| 100 | 94.5 / 79.9 / 69.5 | 94.8 / 75.1 / 63.3 |
| 200 | 94.9 / 82.7 / 73.6 | 94.8 / 79.3 / 68.8 |
| 400 | not run | 95.2 / 81.3 / 69.5 (20-epoch run) |

The 50, 100 and 200 runs have 30 epochs. The 400 run has 20 epochs, so its contact-loss weight decays faster, and its row is not fully comparable. Contact accuracy changes by less than 1 point with W. From W = 100 to W = 200, incline and material accuracy increase by 3 to 6 points. From 200 to 400 (blocked), they increase by 2.0 and 0.7 points. The notebook uses W = 200: W = 400 doubles the label delay (W/2 samples, approximately 200 ms, with the majority label) and the size of `fc1` for a small gain. The 50 blocked and 400 random runs were stopped to reduce GPU memory use (14 GB wired memory, 9.5 GB swap with 6 jobs on the GPU at the same time).

**Table C. Input channels** (W = 100, 20 epochs for every row)

| Channels | Split | Contact | Incline | Material |
|---|---|---|---|---|
| all 9 | blocked | 94.6 (94.6 ± 0.1) | 76.2 (75.5 ± 1.0) | 68.1 (64.5 ± 2.3) |
| position + velocity | blocked | 94.2 (94.2 ± 0.2) | 65.3 (62.8 ± 3.1) | 54.1 (51.7 ± 5.5) |
| effort | blocked | 93.2 (93.1 ± 0.1) | 64.5 (62.0 ± 1.6) | 52.6 (51.1 ± 1.3) |
| position | blocked | 93.2 (92.0 ± 0.8) | 62.0 (54.0 ± 6.4) | 51.9 (42.7 ± 7.9) |
| velocity | blocked | 91.2 (91.2 ± 0.1) | 36.3 (37.5 ± 0.7) | 28.4 (28.3 ± 0.5) |
| all 9 | session | 93.4 (93.2 ± 0.7) | 22.4 (24.3 ± 1.3) | 9.5 (9.8 ± 1.0) |
| effort | session | 92.5 (92.3 ± 0.1) | 23.3 (24.3 ± 1.8) | 12.9 (9.9 ± 1.5) |

**Table D. Simple contact baselines** (blocked split, contact accuracy. Each window becomes one value per channel: its mean, or its mean absolute value for speed)

| Model | W = 100 | W = 200 |
|---|---|---|
| always predict the larger class | 51.4 | 51.2 |
| threshold on mean speed | 74.9 | 75.2 |
| logistic regression, mean position | 81.4 | 81.6 |
| logistic regression, mean speed | 69.9 | 68.7 |
| logistic regression, mean effort | 89.2 | 90.1 |
| logistic regression, all 9 means | 89.4 | 90.0 |

**Table E. Deployment candidates** (W = 200, blocked split, newest-sample label, standardized inputs, 30 epochs. Replay check on the blocked validation part with p ≥ 0.7, 3 evaluations to set contact, 5 to clear it)

| Channels | Contact | Incline | Material | Touchdowns detected | Median delay | False triggers in 3.5 min |
|---|---|---|---|---|---|---|
| velocity + effort | 94.4 (94.4 ± 0.2) | 74.2 (74.7 ± 1.3) | 64.2 (63.2 ± 1.8) | 89 / 90 | 61 ms | 3 |
| effort (deployed) | 94.4 (94.3 ± 0.2) | 71.0 (70.0 ± 1.1) | 60.0 (59.8 ± 1.4) | 89 / 90 | 61 ms | 2 |

## Changes to the handout code

The handout notebooks do not run unchanged. These are the corrections:

| Notebook | Problem in the handout | Correction |
|---|---|---|
| Both | `_parse_folder_name` reads the full path. A number in `data_dir` (for example `.../Fall 2015/...`) can set the incline label. | Parse the path below `data_dir` (`os.path.relpath`). |
| Both | The folder `45 degrees/45-very-rough` uses hyphens, so its 37 contact images and 32,078 joint samples get the label "rough". | Also accept `very-rough`. |
| Explicit | `ThreadPoolExecutor` is used but not imported. | Import it. |
| Explicit | `lookups/` does not exist in the dataset, so `to_csv` fails. | `os.makedirs(..., exist_ok=True)`. |
| Explicit | Threads add samples in arbitrary order, so even a seeded split changes between runs. | Sort the samples. Seed `random_split` (a student TODO). |
| Explicit | `models.resnet50(pretrained=True)` is deprecated. | `weights=ResNet50_Weights.IMAGENET1K_V1` (same weights). |
| Implicit | `os.path.join(data_dir, "/lookups/...")` drops `data_dir` and writes to `/lookups`. | `os.path.join(data_dir, "lookups", ...)`, one file per window size. |
| Implicit | The text calls the last three channels "acceleration". | They are `*_effort` (see below). |

`detect_contact` (explicit notebook) uses the nearest reference image of each group. The reference images are read with `cv2.imread` (BGR), and the query images come from PIL (RGB). Both are center-cropped. This combination keeps exactly 2,242 of the 6,621 images, the number in the handout's 2,017 + 225 sanity check. Other combinations that Claude tried keep different numbers. The mean distance to each group keeps 61 to 264 images (four combinations of channel order and crop). The nearest reference keeps 2,216 with RGB reference images and 2,176 to 2,179 without the crop.

## What "effort" means on Pupper

The joint-state "effort" is not a measured motor current. The Pupper driver computes it from the motor PD law (`pupperv3-monorepo/ros2_ws/src/control_board_hardware_interface/src/control_board_hardware_interface.cpp:557-560`):

```
effort = kp * (q_command - q) + kd * (qd_command - qd)
```

Thus effort is the deflection of the position-controlled joint, scaled by the PD gains, plus a velocity term. A PD-controlled motor acts as a spring and a damper. Under a static load, `kp * (q_command - q)` is the torque that the motor applies. When the foot pushes on a surface, the joint deflects until this torque balances gravity and the contact torque. This deflection is the "implicit compliance" of the lab title.

## Deployment challenge

### Design

| Problem | Decision |
|---|---|
| PyTorch on the Raspberry Pi 5 overloaded the robot in the original project. | `robot/contact_detector.py` runs the exported CNN with NumPy only. One evaluation (W = 200) takes 0.13 ms on the M4, 0.7 % of one core at 50 Hz. Measure the Pi with `test_contact_detector.py`. NumPy uses one BLAS thread. |
| Real-time input format | A ring buffer keeps the newest W `joint_states` samples of `leg_front_r_1..3` in the dataset column order (position, velocity, effort). There is no resampling. The dataset rate is 976 Hz on average (948 to 992 Hz per recording), and `ik.yaml` sets the controller rate to 1000 Hz. The node prints the measured rate every 5 s. The subscription queue is 200 deep (`ik.py` uses 10), so that a slow IK callback does not drop samples. |
| Detection delay | The notebook labels a window by majority vote, so its output changes only when half of the window is in contact (approximately W/2 samples late). The deployed model is trained with the label of the newest sample (`--label last`). |
| False positives and negatives | Hysteresis: contact needs 3 consecutive evaluations (60 ms at 50 Hz) with p ≥ 0.7. Release needs 5 evaluations with p < 0.3. On the replay, p ≥ 0.7 halves the false triggers of p ≥ 0.5 (2 against 4) and adds 18 ms of median delay. A non-finite joint state counts as no contact. |
| Pause loop | The leg is still during the pause, and in the dataset a still leg looks like contact. One pause is allowed per touchdown: the next pause is armed only after the detector reports release for 0.2 s after the pause ends (`--rearm-s`). |
| Transfer to a Pupper without DenseTact | The deployed model uses only the 3 effort channels, standardized with the training mean and standard deviation. Positions depend on the leg geometry (the dataset leg had a DenseTact lower leg) and on the collection trajectory, which can differ from the team's `ik.py` triangle. Effort is the load signal. With effort only, the blocked-split contact accuracy is 93.2 %, against 94.6 % with all 9 channels (W = 100, 20 epochs). |
| Missing effort field | If `joint_states` has no effort, the node logs an error and disables the pause. |
| More training data | `contact_ik.py --log run.csv` writes every joint-state sample in the dataset's `joint_states.csv` format, plus the detector output. Use these files to fine-tune the model or to set the thresholds. |

### Answers to the challenge topics

**How do we preprocess the motor states in real time to match the training format?** The node subscribes to `joint_states` and takes position, velocity and effort of `leg_front_r_1..3` in the column order of `joint_states.csv`. A ring buffer keeps the newest 200 samples, with no resampling, because the robot and the dataset both run at approximately 1 kHz. The node then selects the model's channels (the 3 efforts) and standardizes them with the training mean and standard deviation in the `.npz`. It evaluates the model at 50 Hz. The node prints the measured message rate, and it disables the pause if the effort field is missing.

**Do we use the raw model predictions or apply filtering?** We apply filtering. A single-window output changes back and forth at the start and end of a contact. The hysteresis sets contact after 3 evaluations with p ≥ 0.7 and clears it after 5 evaluations with p < 0.3. On the replay of unseen data, this gives 89 of 90 touchdowns with a median delay of 61 ms and 2 false triggers in 3.5 min. With p ≥ 0.5 the delay is 43 ms, but there are 4 false triggers. The training label also matters: the notebook's majority label delays the output by approximately half a window (median 155 ms), and the newest-sample label removes most of this delay (61 ms).

**How do we handle false positives and false negatives?** A false positive causes an unnecessary 1 s pause. A false negative lets the leg continue through the touchdown. The node uses four measures:

* `--p-on` and `--n-on` set the balance between the two errors. Set them from the free-swing and ramp logs (procedure steps 4 and 5).
* One pause is allowed per touchdown. The next pause is armed only after 0.2 s of release, because a held leg looks like contact.
* A non-finite joint state counts as no contact, and a missing effort field disables the pause.
* `--no-pause` runs and logs the detector but never pauses, so that you can do a check of the detector first.

**Do we collect more data or modify the model?** We modified the model for this task, and more data is probably necessary.

* Model changes: effort-only inputs, standardization and the newest-sample label (Table E). Positions depend on the dataset leg (a DenseTact lower leg) and on its trajectory. Velocity depends on the 1 s pause of the collection program.
* More data: the team's leg is different from the dataset leg. Also, terrain identity did not transfer to a later session, although contact did (92.5 % with effort only). To collect data, run `contact_ik.py --no-pause --log run.csv`, which writes `joint_states.csv` rows. For the labels, write a `contact.csv` (`timestamp,contact_value`) from a known reference, for example from forward kinematics with a ramp at a known height. Put both files in a `tactile_data`-style folder, next to the original recordings, and retrain with `analysis/ablation.py --channels eff --label last --norm --save-model`. The script trains from random weights, so keep the original recordings in the folder.

### Desk checks (no robot)

`python3 robot/test_contact_detector.py --parity --data <tactile_data> --part val`, with the deployed model:

| Check | Result |
|---|---|
| Hysteresis and pause state machine (synthetic sequences, including a release during the hold) | pass |
| NumPy against PyTorch outputs (64 windows at the training scale, contact p from 0.00 to 1.00) | maximum difference 9.5e-6 |
| Inference time, W = 200, Apple M4 | 0.13 ms per window |
| Replay of unseen data (last 20 % of all 56 recordings, 1 kHz stream, 50 Hz detector, hysteresis started at the beginning of each recording) | 89 of 90 touchdowns detected. Delay after the touchdown label: median 61 ms, 90th percentile 116 ms. 0 repeated edges during a contact. 2 false triggers in 3.5 min without contact |
| Replay of all data (mostly training data), notebook model (majority label, all channels), same detector settings | 463 of 468 touchdowns, median delay 155 ms, 1 false trigger in 18.3 min |
| `contact_ik.py` with a mocked `rclpy`, fed 12 s of a recording (the mock is not included in this folder) | 3 pauses for 3 touchdowns. IK and PD timers run, and the log file has the dataset format |

The replay delay is measured from the `contact.csv` label, which has its own delay of up to 100 ms (10 Hz). The one missed touchdown is in `foam/60 degrees`.

### Procedure on Pupper

WARNING: Put Pupper on its stand before you start a controller. The front-right leg moves through the full triangle.

The robot procedure writes to the robot, so the team does these steps. Claude did not copy files to the robot.

1. Pull this folder onto the robot, in the same checkout as `ik_heuristic_walking_lab/`.
2. In terminal 1, start the Lab 3 controller stack. Replace `<checkout>` with the folder of the repository on the robot:

   ```bash
   cd <checkout>/implicit_compliance_lab/robot
   ros2 launch ../../ik_heuristic_walking_lab/ik.launch.py
   ```

3. In terminal 2, do the desk check on the Pi. Make sure that the speed line shows less than 10 ms per window:

   ```bash
   python3 test_contact_detector.py
   ```

4. With nothing under the foot, run the detector without the pause for 30 s:

   ```bash
   python3 contact_ik.py --no-pause --log free_swing.csv
   ```

   Make sure that no `CONTACT` line appears and that the status line shows approximately 1000 Hz.
5. Put a ramp or a book under the foot at touchdown height. Then run the same command with `--log ramp.csv`. Make sure that one `CONTACT` line appears for each touchdown.
6. Run the full challenge. The leg must stop for 1 s at each touchdown:

   ```bash
   python3 contact_ik.py
   ```

7. If step 4 shows false contacts, increase `--p-on` (for example 0.85) or `--n-on`. If step 5 misses touchdowns, decrease them.

If `IK_LAB_DIR` is set, `contact_ik.py` imports `ik.py` and `kinematics.py` from that folder. Otherwise it uses `../../ik_heuristic_walking_lab`. Unlike `ik.py`, `contact_ik.py` does not send `[0, 0, 0]` on exit, so the leg holds its last position.

## Reproduce

The local runs used Python 3.12 with PyTorch 2.15 (MPS backend on an Apple M4). The notebooks keep the Colab paths, but they were not run in Colab.

1. Unzip the dataset outside the git repository (557 MB):

   ```bash
   unzip ~/Downloads/tactile_data.zip -d ~/Downloads/classes/cs123/
   ```

2. Install the packages:

   ```bash
   pip install torch torchvision scipy scikit-learn opencv-python-headless pandas pillow tqdm peft
   ```

3. In Colab, upload `tactile_data/` to `MyDrive/`, then run each notebook from the top. On a laptop, skip the `drive.mount` cell and set `data_dir` to the local folder.
4. Run the ablations from `analysis/`. Each run writes one JSON file with the per-epoch history:

   ```bash
   python ablation.py --data ~/Downloads/classes/cs123/tactile_data --window 100 --split blocked --channels eff --epochs 20 --out results/w100_blocked_eff.json
   ```

   `explicit_split.py` needs `lookups/explicit_train_dataset.csv`, which the explicit notebook writes.

## Files

| Path | Content |
|---|---|
| `notebooks/explicit_classifier.ipynb` | Completed explicit (DenseTact image) classifier, with outputs |
| `notebooks/implicit_classifier.ipynb` | Completed implicit (joint-state) classifier, with outputs and model export |
| `analysis/ablation.py` | Implicit classifier under window, split, channel and label ablations |
| `analysis/explicit_split.py` | Explicit classifier under the blocked and session splits |
| `analysis/baselines.py` | Threshold and logistic-regression contact baselines |
| `analysis/results/*.json` | Results of each run, with per-epoch history (`w100_blocked_all.json` is the 30-epoch run, `w100_blocked_all_20ep.json` the 20-epoch run of Table C) |
| `robot/contact_detector.py` | NumPy CNN, ring buffer, hysteresis, pause state machine |
| `robot/contact_ik.py` | Lab 3 IK node with the contact pause (ROS 2) |
| `robot/test_contact_detector.py` | Desk checks: state machine, speed, NumPy-PyTorch parity, replay |
| `robot/implicit_classifier.npz` | Deployed model |
