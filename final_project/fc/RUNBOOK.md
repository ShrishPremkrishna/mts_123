# Floor session runbook (blue = `mts`, red = `pupper-red`; current checklist, Nov 28)

Run steps from the UI (home `/`, Play `/play`, tests `/ops`) or `python -m fc.ctl`; each run saves
`fc/runs/<UTC>-<test>/`. Stop at the first failed step and fix it. Sessions ≤ 10 min, let the Pi cool between them.
Two presets: **`slow`** (0.15 m/s) and **`play`** (max_vx 0.75; the app starts with it). Both carry `_stack` speed_cap 1.5,
so slow ↔ play needs no down + Ready. **Play speed: `ctl set max_vx=1.0 wall_brake=0.5`** (1.2 was clean, 1.4 fell
Nov 27: [STATUS](STATUS.md)); back with `ctl preset play`.

**Stops** (know them first). **Space** = zero speed, legs on (UI tab focused; pauses a match, score kept). **Esc /
E-STOP** = signed burst, legs off, damped drop, latched until **Ready** ("NOT CONFIRMED" → R3). **Gamepad R3** = the
same drop, works with no laptop or link (each robot needs its own paired DualSense). The bridge also drops a robot tilted
past 45°. `down` lowers it gently; STILL STANDING → R3 or support it, never `down --force`. Pi side: [robot/README.md](robot/README.md).

## Wi-Fi in a new place (no touchscreen)

Touch overlays stay off on both robots (the GT911 touch chip shares I2C-1 with the IMU), so there is no on-screen Wi-Fi.
Both robots auto-join their saved networks (Shrish's iPhone hotspot, TP-Link_03CC_5G, Stanford…). In a new place:
turn on the phone hotspot, wait for the robot to join, `ssh mts` / `ssh pupper-red` (over the hotspot or Tailscale), then
Shrish runs `nmcli dev wifi connect <SSID> --ask` for the new network. Update `udp_host` in `fc/config/robots.json` if the
LAN IP changed.

## Checklist (blue first; red follows the robot 2 block)

| # | Action | Pass | If it fails |
|---|---|---|---|
| 1 | Swap the battery, power-cycle the Pi (clears a stuck I2C bus). Tags on, corner tags down, mat and stand ready | `ssh mts hostname -I` answers | No answer: check the power, wait 1 min, Wi-Fi section above |
| 2 | `bash fc/robot/sha_check.sh mts` (restage with `bash fc/robot/stage.sh mts` if robot files changed) | `MATCH` | Read the diff; restage |
| 3 | Start the app: `fc/run.sh camera` (robots with a host in `robots.json`, preset play; the banner names both). New site or the camera moved: field empty, four corner tags in view, **Recalibrate** (`ctl op eye.recalibrate`, idle only) | Card: auth ok, loss 0, RTT p95 < 100 ms; calib < 3 px, eye ≥ 15 fps; eye message `calibrated: tags … verify` | `not calibrated: <reason>` / `CAL SUSPECT`: fix what it names (hidden or moved corner tag), then Recalibrate. Camera feed dropped: restart the app (no reconnect yet) |
| 4 | Robot on the stand: **Ready** (homes). Floor: **Ready** again | `status`: `speed_cap` 1.5. Ready fails `imu` (exit 5) = at-rest tilt > 6° | Robot is left standing: **down**, then `fcctl.sh imu-level check` on the stand |
| 5 | **Solo dribble** (Push ball) at `max_vx` 1.0: 3 open, 3 wall, 2 corner balls (needs a straight fence) | Goals; no fence contact or tip; trapped corner = referee touch | Note the run; larger `wall_margin_m` |
| 6 | **Goalie** vs Shrish (new pre-shot hedge 10e77b6) | Saves vs Nov 27's 14/18; robot visible > 95 % | Note shots and positions |
| 7 | **Show flow**: **W**, **W**, **K**, 90 s match | Every phase runs, inside the fence margin; calibration holds (no restart) | **P** to start; note the stuck phase |
| 8 | **down** | Lies down, stack stops | STILL STANDING: R3 |

## Robot 2 (`red`) first motion — Shrish present

Prereqs from [PROVISION.md](robot/provision/PROVISION.md): `ssh pupper-red` works, `provision.sh --check pupper-red red red 11` READY.

| # | Action | Pass |
|---|---|---|
| R1 | Pair red's own DualSense: `bash ~/pupperv3-monorepo/robot/utils/pair_ps5_controller.sh` on red (hold PS+Create) | `--check` 21/21; R3 drops red, not blue |
| R2 | Robot level on the stand, legs off: `ssh pupper-red bash ~/fc-v1/robot/fcctl.sh imu-level`, then `imu-level check` | Level < 2° homed on the stand |
| R3 | Tag-11 board in a colour other than blue's, then `ctl robotcal red` on the live camera | Red tracked with the tag covered |
| R4 | Preset slow, red only (`--robots red`): Ready on the stand, then floor; stops (Space, Esc, R3) | Each stop works |
| R5 | Step fwd at 1.0 facing east and west | Gain, `heading_bias_deg` per direction (eye offsets are global for now: open item) |
| R6 | Both robots, `max_vx` ≤ 1.0: show flow 1v1 | Anti-ram holds, bumps at walking pace only |

**Record each step:** run folder names, pass/fail, and what the summary misses (sounds, stumbles, fence touches).
After the session, update [STATUS.md](STATUS.md).
