# Lane D: gyro fault containment (2026-11-23)

Code lives in `fc/robot/`: `imu_health.py`, `imu_test.py` and `imu/`. Tests: `pytest fc/robot/tests/test_imu_health.py` (24 pass).

## ImuHealth (thresholds are params)
Raw X = −0.707·(gx − gz). A pulse is raw X < −0.02 rad/s while raw Y and Z stay under 0.01. Real motion moves all three axes, so it does not count. The window is 10 s.
- **bad**: tilt > 10° held 2 s at zero command (`bad_cause: tilt`, lie down), or IMU unchanged > 500 ms (`stale`, stop).
- **warn**: pulse density > 0.2, |median raw X| > 0.03, tilt > 6° held 2 s, or unchanged > 200 ms.
- A level persists 3 s after its condition clears.

Replay (`imu/replay_health.py`):
- **ks4**: warn at 18–47 s and 154–187 s (pulses from boot). Bad from **533 s** onward, repeating to 990 s (12–30° tilt, joints still).
- **ks6**: warn through the mild episode (512–1023 s). **No tilt-bad.** A real 22° kick/pick-up lasting 1.3 s sets the 2 s hold. One 0.5 s stale-bad at 1036 s is real: the broadcaster's stamps stop for 1.25 s, a loop stall.

## imu_test tomorrow
Run `python3 imu_test.py --mode M --seconds N --condition LABEL`. It prints one JSON line; the log goes to `~/fc-v1/imu/runs/`.
1. On the stand, no stack: `--mode bno --seconds 600 --condition unpowered`. This reads the BNO directly over I²C, including 0x07 (uncalibrated gyro plus the chip's bias estimate). It refuses if a stack or another I²C user is running, and it never touches SPI (legs limp).
2. The same with the battery on: `--condition battery_on`.
3. Stack up, legs homed on the stand, profile ROS env: `--mode ros --seconds 600 --condition holding`.
4. `--condition warm` and `--condition cold` (hair dryer, then a cold pack).

**How to read it.** `verdict` is `hardware_or_electrical` (uncalibrated raw X pulses), `calibration` (raw X clean, bias estimate moves) or `clean_this_run`. Check `gyro_uncal_raw.x.neg_per_min` (healthy 0–4), `gyro_bias_estimate.x`, `rv_gravity_drift`, `chip_temp_raw` and `pi_temp_c`.

**Accidental first run** (60 s, motors off, no stack, Pi at 64 °C):
- Raw X stayed between −0.004 and +0.010 rad/s, with 0 pulses. The bias estimate stayed within ±0.006; gravity drift was 0.16°.
- Verdict: **clean**. The fault was absent then, so this run does not discriminate.

That run wrote only to the BNO: a soft reset, a product-ID request and set-feature commands. All of that is RAM state, cleared by the driver's own reset at its next start. No tare, calibration, DCD-save or FRS command was sent. Its leftover empty `/tmp/control_board_hardware.lock` is harmless; the reader now removes it.

## BNO calibration state (2026-11-24)
New `imu_test.py` bno-mode options. `summary.calibration` holds each chip response, and `summary.sent` holds every packet written, in hex.
- `--report-calib`: read the ME calibration enables at start and end. Read-only.
- `--gyro-dyncal off|on`: RAM-only for the run. The run ends with a soft reset and a re-read, proving the change reverted.
- `--clear-dcd`: **flash**. Sends SH-2 Clear DCD and Reset. Explicit only; not run yet.

The reader sends only documented SH-2 commands. Tare, save DCD, DCD auto-save and FRS writes are refused in code (`tests/test_imu_calib.py`).

Runs on mts (`~/fc-v1/imu/runs/20261124T0645*` to `T0649*`, tool copy in `~/fc-v1/imu/tool-calib-20261124/`):
- **Boot default:** accel 1, gyro 0, mag 1.
- **Gyro dyncal OFF vs ON, 3 min each at standstill:** identical. Bias median 0.002 rad/s, drift 0, a ±3 LSB dither in both, raw X clean (std 0.0014).
- **What it means:** see the RCA note. In short, gyro calibration cannot explain the ROS gyro faults.
- **Verdict rule changed:** "bias moving" now compares the medians of 6 time segments (> 0.005 rad/s) instead of the raw span. The span is still reported. Summaries also add `rv_vs_accel_deg` (fused gravity against measured accel) and `streams.*.identical` (the longest bit-identical run).

**Stock driver** (Pi `~/pupperv3-monorepo` `0e50afb`, installed .so sha `01813d58…`): at start it soft-resets (SH-2 channel 1), requests the product ID, then enables only 0x05 and 0x02 at 10 ms. It sends no calibration, DCD, tare or FRS commands. The chip runs its boot default and reloads its saved DCD on every reset.

**`imu/stock-bno-dyncal.patch`** (built in isolation, not deployed):
- At start the driver reads and logs `FC_BNO_CAL boot accel= gyro= mag=`.
- If `FC_BNO_DYNCAL=A,G,M` is set, it configures those enables in RAM before enabling reports. It re-reads them and logs `set … (RAM, confirmed)` or a WARNING.
- Unset, the behaviour is stock apart from that one query, which adds up to about 0.3 s at start.
- Build: `~/fc-v1/build/stock-bno-dyncal/`, .so sha `dafee3d7…`. The only warning is a pre-existing one.
- Bench check: a standalone harness (`harness/`, no ROS, no SPI, holding the driver lock) confirmed boot 1,0,1, set 0,0,1 confirmed, the next start back at 1,0,1, a malformed value ignored, and quaternions flowing in every case.
- Proposed use: `FC_BNO_DYNCAL=0,0,1` freezes accel calibration so walking cannot teach and save a new one. This is a hedge, not a demonstrated fix.

**`imu/stock-imu-filter.patch`** (2026-11-24, not deployed): `FC_IMU_FILTER=1` publishes our own accel-corrected orientation and a de-spiked gyro X. See [imu-filter.md](imu-filter.md).

## Heading hold off (no rebuild)
`heading_hold.kp = 0` in the policy JSON disables the hold (`neural_controller.cpp:291-319`, verified); the kp=0 policy has sha `9f5a3e98…`.
Profiles `stock_hh0` / `tej_u6_hh0` (fc/robot/README.md) build and verify a fresh overlay in `<run>/hh0/` at every start and refuse to start if that fails. `status` reports `heading_hold`.
- **u6**: prepare gets `--launch/--config/--urdf` from the overlay's `foundation/hardware/`; kit-script sha pins are still enforced.
- **stock**: `ros2 launch <run>/hh0/launch_hh0.py teleop:=False`.

Files-only dry run on mts 2026-11-23 (`~/fc-v1/tmp/hh0-dryrun-20261123T115639Z/`, no stack started): u6 `persistent_session.py prepare` succeeded with native `37938659` and imu `1dd92b65`, and its generated config points `neural_controller_default` and `gemini_walk` at the kp=0 policy. Evaluating the stock launch description points `neural_controller_default(_alt)` at the policy; `neural_controller` (legacy) is unchanged.
Still needed: one straight-walk floor check per stack.

## Stale-IMU tolerance
- **stock**: no driver change. ImuHealth `stale_ms` plus a bridge stop catches a frozen IMU.
- **u6**: `imu/u6-stale-tolerance.patch` holds the last good sample for up to about 300 ms without reports instead of disabling motors on one 100 ms miss. NaN and bad-norm faults still fail immediately. It logs `FC_IMU_GAP … resumed after N ms`, which separates reader starvation from a BNO dropout. Built in isolation at `~/fc-v1/build/u6-stale-tol` (sha `e95b4816…`); all 56 tests pass. **Not deployed**; it needs a bench run first.

## What decides replacement
- Uncalibrated raw-X pulses with motors unpowered: in-chip fault, replace the BNO or the board.
- Pulses only while the motors are holding: electrical, check wiring first.
- Clean raw X but a moving bias: calibration, a software fix.
- A clean Pupper 2 under the same test isolates this board.
