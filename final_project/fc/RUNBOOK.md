# Floor session runbook (one robot: blue = `mts`; current checklist, Nov 27)

Run steps from the UI (home `/`, Play `/play`, tests `/ops`) or `python -m fc.ctl`; each run saves
`fc/runs/<UTC>-<test>/`. Stop at the first failed step and fix it. Sessions ≤ 10 min, let the Pi cool between them.
Two presets: **`slow`** (0.15 m/s) and **`play`** (max_vx 0.75). Both carry `_stack` speed_cap 1.5, so Ready passes
`--speed-cap 1.50` once and slow ↔ play needs no down + Ready. Other speeds: `ctl set max_vx=…`; back with `ctl preset play`.

**Stops** (know them first). **Space** = zero speed, legs on (UI tab focused). **Esc / E-STOP** = signed burst, legs
off, damped drop, latched until **Ready** ("NOT CONFIRMED" → R3). **Gamepad R3** = the same drop, works with no
laptop or link. The bridge also drops a robot tilted past 45°. `down` lowers it gently; STILL STANDING → R3 or
support it, never `down --force`. Pi-side details: [robot/README.md](robot/README.md).

## Checklist

| # | Action | Pass | If it fails |
|---|---|---|---|
| 1 | Swap the battery, power-cycle the Pi (clears a stuck I2C bus). Tags on, corner tags down, mat and stand ready | `ssh mts hostname -I` answers | No answer: check the power, wait 1 min |
| 2 | `bash fc/robot/stage.sh mts && bash fc/robot/sha_check.sh mts` (robot files since 1b2b879 are not on the Pi) | `MATCH` | Read the diff; restage |
| 3 | Restart the app: `fc/run.sh camera --udp-host blue=<LAN IP>` (camera default: robots with a host in `robots.json`, preset **play**; the banner names both; `--robots`/`--preset base` override). New site, or the camera was remounted: field empty, all four corner tags in view, delete `fc/config/eye_cal.json` first (the app fits and saves a new one) | Card: auth ok, RTT p95 < 20 ms (LAN), loss 0; calib < 1.5 px, eye ≥ 15 fps; eye message `calibrated: tags … verify` (not `unverified` / `CAL SUSPECT`) | Wrong IP: update `robots.json`. `not calibrated: <reason>`: fix what it names (e.g. a hidden corner tag). `fit error … rejected`: tags flat and fully in view, then retry |
| 4 | Preset **slow**. Robot on the stand: **Ready** (homes, `--speed-cap 1.50`). Floor: **Ready** again. Then preset **play** (no restart) | `status`: `speed_cap` 1.5 (if 0.49, slow's `_stack` change has not landed: down, preset play, Ready). Ready fails `imu` (exit 5) = at-rest tilt > 6° | Robot is left standing: **down**, then step 5 on the stand |
| 5 | `ssh mts bash ~/fc-v1/robot/fcctl.sh imu-level check` (and `show`) | Standing 2–4° (< 2° homed on the stand); `status.imu_cal` stored = applied. Then delete `~/fc-v1/robot/imu_params.json` if present (default 10/6° guards from the next bridge start) | Robot level on the stand, legs off: `fcctl.sh imu-level`, then re-home |
| 6 | **Speed ladder**, open floor, each step with `wall_brake`: `ctl set max_vx=1.0 wall_brake=0.5`, Step fwd 1.0 + 2 s bursts and arcs; then 1.2, 1.4 (1.5 optional). Sim: above 1.0 buys nothing (solo median 8.2 s at 1.0, 9.7 s at 1.4; wall-ball tips 1/20 → 2/20), so the floor decides | No fall or stumble; record gain and `latency_s` per step | Stop the ladder; keep the last clean `max_vx` |
| 7 | **Solo dribble** (Push ball) at the best clean speed: 3 open, 3 wall, 2 corner balls | Goals; no fence contact or tip; trapped corner = referee touch | Note the run; fall back one ladder step |
| 8 | **Goalie** vs Shrish: rolled shots and dribbles at goal | Keeper shuffles mid-mouth, shifts toward the ball; count saves | Note shots and positions |
| 9 | **Show flow**: **W** (warm-up), **W** (to spot), **K** (3-2-1), score, celebration, back to spots, 3-2-1 | Every phase runs; robot stays inside the fence margin | **P** to start; note the stuck phase |
| 10 | **Autoplay**: `ctl test autoplay minutes=3`, watched from afar | Goals each cycle, no tip | Note the cycle |
| 11 | **vs-human**: `play` alone vs Shrish, 3 min | Goals for / against; is the defence better than Nov 27? | Note what beat it |
| 12 | POV feed on Play; RTT p95 on the card during play | Feed live; RTT p95 (campus Wi-Fi was ~200–250 ms → use Shrish's router) | `ctl ready --pov off` |
| 13 | **down** | Lies down, stack stops | STILL STANDING: R3 |

**Two robots:** play at `max_vx` ≤ 1.0 until the Coach's anti-ram brake has been checked on the floor.

**Record each step:** run folder names, pass/fail, and what the summary misses (sounds, stumbles, fence touches).
Each run's `params.json` holds the preset, `_stack`, the running profile and stack options. After the session,
update [STATUS.md](STATUS.md) with what changed.
