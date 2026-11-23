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

## Heading hold off (no rebuild)
`heading_hold.kp = 0` in the policy JSON disables the hold (`neural_controller.cpp:291-319`, verified). Build the overlays on the Pi with `python3 imu/heading_hold.py u6|stock <dir>`; the kp=0 policy has sha `9f5a3e98…`.
- **u6**: in the prepare script, pass `--launch/--config/--urdf` from `~/fc-v1/imu/overlays/u6-hh0/foundation/hardware/`. A dry run of profile generation pointed `gemini_walk` and `neural_controller_default` at the kp=0 policy.
- **stock**: `ros2 launch ~/fc-v1/imu/overlays/stock-hh0/launch_hh0.py`.

This needs one straight-walk floor check.

## Stale-IMU tolerance
- **stock**: no driver change. ImuHealth `stale_ms` plus a bridge stop catches a frozen IMU.
- **u6**: `imu/u6-stale-tolerance.patch` holds the last good sample for up to about 300 ms without reports instead of disabling motors on one 100 ms miss. NaN and bad-norm faults still fail immediately. It logs `FC_IMU_GAP … resumed after N ms`, which separates reader starvation from a BNO dropout. Built in isolation at `~/fc-v1/build/u6-stale-tol` (sha `e95b4816…`); all 56 tests pass. **Not deployed**; it needs a bench run first.

## What decides replacement
- Uncalibrated raw-X pulses with motors unpowered: in-chip fault, replace the BNO or the board.
- Pulses only while the motors are holding: electrical, check wiring first.
- Clean raw X but a moving bias: calibration, a software fix.
- A clean Pupper 2 under the same test isolates this board.
