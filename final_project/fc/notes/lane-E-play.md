# Lane E: Play screen, match flow, Ready, feeds, recording (2026-11-23)

Built on `fc-v1` @ 15e3f31. Usage and behaviour: [../README.md](../README.md) (Play screen section).

## Works now (verified)
- `/` Play screen, `/ops` the old test page. Screenshots (1280x720, sim and synthetic camera):
  [play-screenshots/](play-screenshots/) `two-robots-live.png`, `one-robot-live.png`, `camera-synthetic.png`.
- Match flow in sim: kickoff → spots → countdown → play → goal → restart on a still ball → full time;
  pause, reposition, jobs, variant switch (V) mid-match, referee touches. `fc/tests/test_play.py` (flow,
  ticker, banners, health, recorder, camstream↔CamFeed) and new cases in `test_app.py`; 85 pass in
  `.venv-fc`, 83 + 2 skipped (no OpenCV) in `.venv`.
- Camera overlay: synthetic source, calibrated → field lines and belief rings land on the rendered tags
  (homographies sent at full precision; `clean()` rounding to 4 decimals broke them at first).
- Run video: a 45 s synthetic match wrote 681 frames at 15 fps (real time) to `overhead.mp4`.
- `.venv-fc` at the main repo root: Python 3.12.8 (`/usr/local/bin/python3.12`), requirements.txt pins.

## Not verified / needs the robot side
- `fcctl.sh ready <profile>` and `down` are another lane's work: the app sends them (allowlisted, quoted,
  confirm for ready) and parses JSON progress lines, tested only with a fake process and dry-run.
- `robot/camstream.py` has never run on a Pi; the stock profile may not start camera_ros at all
  (then `--source picamera2`, only when nothing else holds the sensor).
- Synthetic eye ran at 2-7 fps on this laptop with the page open (synthetic rendering is the cost; the
  view encode is ~3 ms and the mp4 write ~2 ms per frame). The stale-tag banners it showed were true.
- Nothing here has driven a physical robot.

## Decisions
- STOP ends the match (the Play screen shows a "STOPPED" card with the score); Pause keeps it.
- Goals only count in the play phase. A bridge halt (IMU/mux) zeroes that robot but does not end a flow
  match, so the other robot can play on (demo plan variant A → B).
- Ready button only shows with the camera source or `--dry-run-ops` (in sim it would still ssh the real robot).
