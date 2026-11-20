# Onboard pink-ball telemetry

The optional FC camera path consumes the newest ROS frame and publishes an age-limited ball observation on `/fc/onboard_ball`. The `--fc-ball-mode` choices are pink appearance alone, `confirmed` for a same-frame YOLO corroboration, and `off`. The optional `--fc-timings` output records capture-to-publication timing. Existing camera launches opt into this path with `--source ros --fc-headless`. The runtime needs the Pi’s ROS camera, Hailo runtime and an installed YOLOv8n-seg HEF. Pass its path via `--model` if it lives outside the detector directory.

Run `python -m pytest foundation/tests/test_onboard_ball.py foundation/tests/test_detection_capture_stamp.py` with the camera test dependencies. The archive-based cadence replay also needs its original capture receipts.

The next camera-only acceptance run should compare all three modes on the same pink ball and distractors. Record capture age, fresh detection rate, dropped frames and board temperature over a sustained run. Agreement on archived timestamps and software fixtures establishes the telemetry contract. Blue's optical accuracy, sustained Hailo throughput and any thermal cause of earlier missed detections still need measurements.
