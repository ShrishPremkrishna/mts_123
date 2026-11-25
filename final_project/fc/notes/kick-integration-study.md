# Leg kick for fc v1: study (Nov 24, 2026, read-only)

**Recommendation: no-go before Dec 3.** DEMO.md already lists "leg kick" under do-not-build.

## 1. Assets

- **Leg-kick policy `9c841f26…`** is a tracking policy with the clips `stand_idle`, `kick_right` and `kick_left` (3.0 s each, kp 5.5). The paw swings through a ball at (0.258, −0.046 / +0.040) m.
  - It runs on the **stock** plugin: the Pi's `libneural_controller.so` has `TrickClip` and a `~/clip` topic, but no `start_clip`. Tej's penalty pipeline loaded it in the square slot.
  - Robot results:
    - Nov 16: 2 right kicks hit.
    - Nov 19: left hit, right miss.
    - u5: 4 clips confirmed, contact not measured.
    - Tej's handoff: 3 hits in 4 kicks; contact window ±2 cm forward, ±1 cm sideways.
- **`kick_in_place`, `kick_at`, `kick_ball` and the short walker restart** (0.2 + 0.3 s) work **only on u6** (Tej's HTTP server and build). Our `tej_u6` profile omits the API.
  - u5 result: walk 0.3 m, kick and walker ready in 5.0–5.7 s.
- **`overhead_kick.py`**: overhead staging, then the **onboard** camera for the final line-up, then `kick_in_place`. u6 only, offline-tested only.
- **Penalty pipeline and `leg_kick_buttons.py`** (stock): STRICT switch, clip, watchdog, switch back. Never moved the robot.
- **Header kick `84928297…`**: simulation only (85 %), needs a head bumper.

**Pi `mts`** (read only):
- `~/policies` holds 2 walking policies (`5a94a149…`, `42d4863b…` = stock `test_policy.json`).
- `9c841f26` exists only in the u-releases (20261118–u6) under `~/pupper-foundation-session-20261029/`.
- Stock walker blend: 2 + 2 s. No stack was running.

## 2. Minimal design

- **Pi.** `stack_overlay.py` adds an inactive `neural_controller_kick` (9c841f26, blend 0.2 + 0.3 s), the same way it adds the lie-down controller. It goes into `estop_controller` and the bridge `--legs`, so R3, link E-STOP and the fallen cutoff cover it.
- **Packet.** A signed `action: kick_left|kick_right` with an id. The bridge refuses it unless:
  - the walker is active,
  - the IMU is ok,
  - there is no latch or hold,
  - the robot has been still for at least 0.3 s.

  Then the sequence is:
  1. STRICT switch walker → kick.
  2. Publish the clip after the blend.
  3. STRICT switch back when the clip returns to `stand_idle` (cap 3.5 s).
  4. Ack `kick{id, phase, ok, why}`.
- **During the clip.**
  - Motion packets are ignored; the dead-man does not abort; Stop switches back at once.
  - E-STOP and the fallen cutoff do R3.
  - ImuLie switches kick → lie-down.
  - The watchdog stays on. A refused switch back means R3.
- **Brain.** A `KICK` phase:
  1. Stop about 0.6 m behind the ball on the paw line and average 0.5–1 s of observations.
  2. Pulse until the ball is within ±1 cm sideways.
  3. Creep blind to x = 0.258 m and send the kick once.

  Kick only for a still ball with no opponent within 1 m; otherwise push.

## 3. Risks

- **Alignment (decisive).** The kick needs ±1 cm sideways. Our goto lands within 5.6–6.9 cm (one 14 cm miss); the 8 cm tag is seen in 18 % of far-half frames; heading offsets of 7–8° vary with gait (1° ≈ 0.5 cm at the paw).
  - Tej's bridge needed the onboard camera for this; ours is off, with no detector.
- **Time.** About 8–20 s per kick, mostly line-up (then blend 0.5 s, contact 1.45 s, return 1.15 s, stock walker restart 4.2 s). Fast push goals take 16–22 s in total.
- **Switch safety.** A new path that bypasses the mux; the policy jitters if the robot is lifted. Needs simulation, a stand test and **course approval** (not confirmed).
- **Heat.** Minor (49 °C idle; one policy at a time).

## Effort and decision

About 3–4 dev days plus 2 floor sessions to reach a measured hit rate. Only S2, S4 and the Sat 28 freeze remain, and all are needed for tag visibility and Stage 1 (3/7 goals). **No-go.** After Dec 3, use this stock-slot design with a ≥ 15 cm tag and an onboard or close-range ball fix.
