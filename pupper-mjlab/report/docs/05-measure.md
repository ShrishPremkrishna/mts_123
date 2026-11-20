# 5 · Measure (optional)

## Record a session

```bash
./record.sh bumpy_floor      # -> ~/runs/<time>_bumpy_floor (MCAP). Ctrl+C to stop
```

Topics: joints, IMU, `/cmd_vel`, `/joy`, the e-stop status, the student policy's output and latency, and `/rosout`.

## Standing check vs sim

On the floor, Square active for at least 4 s, stick at zero:

```bash
./stand_check.py ~/policies/23pt1499.json      # 5 s window; --seconds N to change
```

It prints:
- joint offset from the policy's default pose, per leg (rad; left-back highlighted)
- left share of |effort|: a load proxy. `effort` is the driver's PD estimate, not measured torque
- IMU roll and pitch

Sim reference for bumpy: left-back tuck about **0.38 rad**, about **59%** of the load on the left. Flat control has neither.
Joint order (`config.yaml`) is FR, FL, BR, BL × (1 = hip fore/aft, 2 = hip side, 3 = knee). Left back = indices 9–11.
