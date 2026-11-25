# Floor session runbook (fc v1.1, one robot: blue = `mts`)

Every step runs from the UI (Play screen `/`, tests `/ops`). Each run saves `fc/runs/<UTC>-<test>/`. Stop at the
first failed step and fix it. Keep sessions under 10 minutes and let the Pi cool between them.

**Three stops, know them before you start.**
- **STOP (space)**: zero speed to every robot. It stands still, legs stay on. Only while the UI tab has focus.
- **E-STOP (Esc, or the striped button; per robot on its card)**: signed e-stop burst. The bridge does the R3
  action: legs off, the robot **drops** (damped), and stays latched until **Ready** re-arms it (the light says
  `E-STOPPED · re-arm = Ready`; nothing clears it by itself). "NOT CONFIRMED by the robot" = use R3.
- **R3 on the gamepad**: the same drop, and the only stop if the laptop, link or mux is gone.

The bridge also drops a robot that tilts past 45° (fallen cutoff); the card then says "E-STOPPED by the robot".
`down` lowers the robot to its belly under position control (stock gentle lie-down; robot-verified Nov 24; u6
still drops) and stops the stack. E-STOP or R3 during the lowering aborts it
(legs off: a damped drop from part-way down). If it reports **STILL STANDING**, use R3 or support
the robot; never `down --force`.

## Setup
- [ ] Battery charged, stand ready, gamepad paired, R3 within reach, soft mat for the e-stop step.
- [ ] Robot tag fitted, corner tags 0-3 down, iPhone Center Stage off.
- [ ] Code and key on the Pi: **stage** on the robot card (`python -m fc.robot.keygen` once, if no key).
- [ ] `ssh mts hostname -I`, then `fc/run.sh camera --robots blue --udp-host blue=<LAN IP>`.
- [ ] Robot card: auth ok, RTT p95 < 20 ms, loss 0. Field: calib < 1.5 px, eye ≥ 15 fps, yaw ≈ 0° facing east;
      `ctl status` shows the field size from the camera.
- [ ] New ball or light: `python -m fc.ctl ballcal` (ball in view). New ball: `python -m fc.ctl rollcal --apply`, then
      copy the value into `config/params.json` `ball_decel` (the live setting is lost on restart).

## Steps
| # | Action | Pass | If it fails |
|---|---|---|---|
| 1 | Preset **slow**. Robot on the stand: **Ready** (homes). Move it to the floor: **Ready** again (stands, bridge up) | Light says `ready`; **status** shows `speed_cap` 0.49 and `heading_hold` on | Copy the op output. Another profile running: **down** first |
| 2 | `/ops` **Drive** (WASD/QE) slowly for 1 min | Follows keys, no drift, IMU ok, Pi < 75 °C | IMU BAD: the app already zeroed it; R3 |
| 3 | Every stop, robot low over the mat: space; Ctrl-C the app (restart it); gamepad stick; R3; **E-STOP** (Esc) | Space and app killed: still < 0.5 s. Stick wins. R3 and E-STOP: a damped drop, card `E-STOPPED` (confirmed), Drive refused | Note which stop failed and end the session |
| 4 | Re-arm with **Ready** (stands it up again). Then **Deadman** | E-STOP clears only after Ready; `stop_time_s` ≤ 0.5 | E-STOP still latched: read the Ready output |
| 5 | **Step** at Base (**Base** button): fwd 0.1 / 0.2 / 0.3, turn ±0.5, east and west | `latency_s` < 0.35, `gain_lin` within ±10 %, `\|heading_bias_deg\|` < 5 | Copy gains/lag into `sim_*` and rerun Sim eval |
| 6 | Preset **fast-play-robot**, **down**, **Ready** on the stand, **Ready** on the floor (passes `--speed-cap 0.80`). Step fwd 0.4 and 0.75 | **status** shows `speed_cap` 0.8; no "max_vx > robot speed cap" warning; gain at 0.75 recorded | Warning stays: the stack runs the old cap; down + Ready again |
| 7 | **Push ball** ×5 with preset **slow**, then ×5 with **fast-light** (light ball; same 0.8 cap as fast-play-robot) | Goals ≤ 30 s, no tip, slows near the ball | Note failing run names |
| 8 | **down** | Robot lies down, stack stops | STILL STANDING: R3 or support it |

**A/B athletic:** step 7 with **fast-light** ×5 vs **athletic** ×5 (same cap); compare goals, time and `wz_reversals`.

**A/B heading hold:** presets `stock-hh-off` / `u6-hh-off` pass `--heading-hold off`; down + Ready, repeat 5 and 7.

**Record each step:** run folder names, pass/fail, and what the summary misses (sounds, stumbles, how the drop
looked). Each run's `params.json` records the preset, its `_stack`, the running profile and stack options.
