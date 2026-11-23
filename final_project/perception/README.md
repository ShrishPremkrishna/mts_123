# Onboard metric ball perception

November 22, 2026. Offline implementation; **no physical accuracy is claimed**. Robot, camera and services were not touched.

## What exists

| Part | Path | Role |
|---|---|---|
| Ball Eye geometry and detector | `follow_me_pupper/robot/ball_eye.py` | Maintained port of the recovered u6 source (sha256 `fd255940…f351f73`): double sphere lens, stereographic floor view, panorama cone (`sin ρ = sin w · cos lat`), mount, outline fit, range `R / sin ρ`. Three proposal streams on the same frame: Ball Eye YOLO (`eye`), the maintained pink tracker's panorama box as a seed (`pink`), and pink components in the floor view (`pinkfloor`). Image-noise uncertainty per measurement. No clock, tracker or learning. |
| Selected profile | `follow_me_pupper/robot/ball_eye_profile.json` | `estimated`: radius 0.11 m from `arena.json` (setup estimate), u6 mount estimate, lens from `camera_params.yaml`. `profile_id` and `calibration_id` are content hashes. A test pins the radius to `arena.json`. |
| Camera node hookup | `viser_camera.py --fc-metric-profile JSON [--fc-ball-eye-model ONNX --fc-ball-eye-sha256 HEX]` | Opt-in. After the obstacle envelope and pink row are published, the same capture goes to a one-slot `MetricWorker` thread. A job that is no longer current before or after measurement is dropped. Publishes `/fc/onboard_ball_metric`. |
| Wire contract | `onboard_ball.py` (`fc_onboard_ball_metric_v1`, `MetricBallReceiver`) | Original ROS and Pi-monotonic capture times, producer/sequence, profile identity, model hash, per-method candidates, `uncertainty_scope: image_only`. Stale/future/duplicate/reordered/malformed rows change nothing. |
| Consumer | `final_project/pupper_fc/local_ball.py` | `LocalDocking` returns evidence with status `overhead` (no enabled profile; behaviour unchanged), `blocked` (enabled profile but no current agreed local ball; overhead may steer but not authorize a kick) or `local` (body-frame docking). It handles admission and pre-reset cutoffs, outlier gating (outliers never seed, but do suspend the track), and absence rows that end the track. Motion expires the track: commanded, unknown (kick clip/hold), overhead drift since capture, or overhead seeing the ball move. Agreement and the wind-up forecast use a noise-shrunk velocity. `live_profile()` is unqualified until radius, mount and an independent local error are measured. Live docking also needs camera rows marked measured. `synthetic_profile()` accepts only synthetic rows. |
| Runner telemetry | `combined_fc_runner.py` | `local_ball` event, `usage: telemetry_only`, beside the existing `onboard_ball` display event. |
| Comparison / capture tool | `final_project/perception/compare_onboard_ball.py` | Synthetic, negative, sensitivity and optional labelled-real comparison on identical images. |
| Synthetic frames | `final_project/perception/synthetic_ball.py` | Ray-traced sphere in saved real Pupper fisheye frames, with exact labels. |
| Seam rehearsal | `final_project/perception/rehearse_local_docking_seam.py` | Applies the proposed controller/executor insertions to a temporary copy and runs the real Executor. |

The existing person/obstacle hold, `/fc/obstacle_detections` envelope, COCO-model-bound soccer-ball exception and bearing-only `/fc/onboard_ball` contract are unchanged.

## Evidence (synthetic, Mac CPU)

Model `ball_eye.onnx` sha256 `ae3a4592…0a827c` loads through OpenCV DNN with a fixed 320×320 input; its training provenance is unknown. The run used 15 positions × 6 backgrounds: one plain floor plus five saved real Pupper v3 fisheye frames with the same lens parameters, from `pupperv3-monorepo/ai/playground/…/images`. Errors are against ray-traced labels, so they measure the pipeline under the assumed lens and mount, not camera accuracy.

| Range | eye (fit/box) | pink panorama seed | pinkfloor |
|---|---|---|---|
| Contact < 0.3 m | 7/12, max 0.5 mm | **0/12** | 12/12, max 0.5 mm |
| Near 0.3–0.5 m | 11/30, max 0.9 mm | 19/30, max 2.5 mm | 27/30, max 2.8 mm |
| Mid 0.5–0.9 m | 2/24 (one box 0.66 m wrong) | 22/24, median 3.5 mm, max 7.6 cm | 22/24, median 3.4 mm, max 10.7 cm |
| Far ≥ 0.9 m | 16/24 boxes, median 1.5 cm | 22/24 boxes, median 5.8 cm | 3/24 |

- The maintained panorama pink tracker cannot propose a ball at paw contact. At 0.258 m the ball crosses the panorama's bottom pole and spreads across its full 700 px width, so every pixel-shape gate fails (aspect 0.47, fill 0.39). The stereographic floor view keeps it round: floor-view pink and Ball Eye measure it.
- The eye model's low hit rate on rendered balls says nothing about real frames. It was trained on real images.
- The large mid/far outliers come from one background where the ball is composited over skin-coloured legs. The outline grows into them, and the image-only sigma (1–3 cm) understates the error. pink and pinkfloor share the outline fit, so their agreement is not independent.
- Box-only measurements can be badly wrong. Docking accepts outline fits only.
- No ball measurement on 9 real ball-free frames (all streams).
- Sensitivity, measured with a wrong parameter on renders made with the selected one (median / max error over 4 positions): radius 0.091 → 6.2 / 18 cm; tilt ±3° → 0.3–0.5 / 8 cm; yaw +3° → 2.4 / 7.5 cm; camera x +1 cm → 1.0 / 7.1 cm.
- Mac median for all three streams together: about 30 ms per frame. Pi cost is unmeasured.

The seam rehearsal used the real Executor and a patched copy. With an overhead ball 4.5 cm short and no seam, or with the default live profile, the robot is `kick_ready` at 0.25 s with the true offset at 0.303 m, outside the window (0.270 ± 0.02 m). With the synthetic profile it steps on local evidence and is `kick_ready` at 1.25 s with the true offset at 0.267 m. With overhead 0.20 m off (beyond the agreement budget), it holds in `align` and never kicks.

Rerun: `…/.venv-fc/bin/python final_project/perception/compare_onboard_ball.py --model <onnx> --model-sha256 <hex> --out <json>` and `.venv-api/bin/python final_project/perception/rehearse_local_docking_seam.py`. Tests: `foundation/tests/test_ball_eye.py`, `foundation/tests/test_onboard_ball.py`, `final_project/pupper_fc/tests/test_local_ball.py`, `gemini_pupper/tests/test_runner_session.py`.

## Seams other owners must integrate

1. **Executor (`integration.py`)**:
   - Add `local_docking_profile=None` and per-robot `LocalDocking`, reset in `_clear()`.
   - Call `note_overhead(pose, track capture, ball, frame stamp)` after a frame is admitted.
   - Call `note_command(now, vx, vy, wz)` for every planned command.
   - Call `unknown_motion(now, …)` on `finish_kick` and in `_hold_external`. A clip or hold can move the body without a velocity command.
   - Pass `local_docking=evidence(now, pose, pose capture)` to paw ticks.
   - Add `observe_local_ball(robot_id, row, now)` as a trusted local adapter, like `set_external_hold`, not an upstream event. Unknown event kinds stop the executor. It takes the receiver row so searching/conflict rows end the track.
2. **Controller (`controller.py`)**: `tick(..., local_docking=None)` → `_paw_attack`. `"local"` evidence docks with `body_contact_window`, decides readiness with `local_kick_ready` (steady, in window, forecast through the wind-up), and steers on the body-frame error. Commands expire with `local.expires_at` (pose age ≤ 0.15 s, like overhead readiness). `"blocked"` evidence downgrades an overhead `kick_ready` to `align`. Nothing is remembered across controller cancels: kick authority comes only from current evidence. The exact anchored text is in `rehearse_local_docking_seam.py`.
3. **Runner (`combined_fc_runner.py` + `fc_executor_bridge.BridgeCore`)**:
   - Pass the docking profile through `BridgeCore`.
   - For each newly admitted `MetricBallReceiver` row, call `executor.observe_local_ball(robot_id, row, clock())`.
   - Use `live_profile(field)` until qualified.
4. **Release closure (runtime)**: add `follow_me_pupper/robot/ball_eye.py` to `FIELD_FILES`, `ball_eye_profile.json` to the data files, and `final_project/pupper_fc/local_ball.py` once the executor imports it. Stage the model by hash; it already exists on the Pi at `~/.config/gemini-pupper/ball_eye.onnx`. Add the camera-node flags to the launch only after Pi CPU impact is measured. `test_runtime_import_closure_is_staged` currently fails on `ball_eye.py` and on the concurrent `prediction.py` until this is done.
5. **UI**: the `local_ball` event is safe to ignore. If shown, label it as unqualified image-only telemetry.

## Prepared real comparison (P1)

There is no saved onboard ball footage: the November 20 recordings contain telemetry only.

**Capture:** motors off, robot standing on its stand at its normal base height. Use the real ball and the selected profile. Tape-measure ball centre positions in base_link at 0.258/0.27 m (both paws), 0.3, 0.4, 0.6, 1.0 and 1.5 m, at lateral 0 and ±0.15 m. Include clutter and pink/skin distractors, and record the lighting.

**Save:** the decoded JPEGs, plus `labels.csv` with columns `file,x_m,y_m`.

**Measure separately:**
- ball circumference, for the radius
- camera position and tilt, for the mount
- base height

Then run the tool with `--real-dir DIR --labels labels.csv`.

**Qualification for live docking** needs, on held-out frames, measured radius and mount plus an independent local error at most half the lateral window (≤ 7.5 mm). It also needs a measured motion/exposure budget. Until then `live_profile` stays off.

## Independent review (Opus 5.5, November 23)

A pinned `claude-opus-5-5` read-only review reproduced the following gaps; all are fixed and covered by tests or the rehearsal:
- an overhead-only kick after local disagreement (the seam's engagement flag was cleared by controller cancels);
- a stale local fix kept despite a newer outlier or "searching" row;
- a local kick path allowing a pose up to 0.35 s old and skipping the rolling-ball forecast;
- pre-reset captures re-entering after a reset;
- kick clips and holds counted as zero motion;
- slow overhead drift going unnoticed;
- self-declared qualification flags.

It also confirmed Lens/View/Pano/Mount/`circle_to_body` give bit-identical results to the recovered source over 2000 random cases. Still open:
- the image-only sigma overstates noise on full arcs and understates it on short arcs;
- the 0.15 m agreement budget cannot detect a radius-sized (about 4 cm) range error, so only `measured_error_m` qualifies the radius;
- the u6 `cv2.setNumThreads(2)` CPU limit is not applied (it is process-wide and would also slow the obstacle remap), so Pi contention must be measured;
- on the Pi, Ball Eye rows may arrive older than the consumer's 0.25 s limit.

## Remaining physical unknowns

- True ball radius, camera mount (x, z, tilt, yaw) and base height while walking.
- Exposure-to-stamp delay of `camera_ros`.
- Real outline-fit error under the field's lighting and clutter.
- Ball Eye recall on real frames, and its training set.
- Pi CPU time, and its effect on obstacle cadence.
- Robot sway while "still".
- The agreement budget (0.15 m, from Tej) and the motion budgets (1 cm / 0.03 rad commanded, 3 cm / 0.06 rad overhead step). These are synthetic choices.
- Contact geometry and timing.
