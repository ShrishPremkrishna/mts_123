"""Rehearse the shared overhead-to-leg-kick software path with a single simulated Pupper.

Run with python -m final_project.pupper_fc.kick_rehearsal [--output results.json].
Inputs are synthetic measured tracks. This module opens no camera, network or robot connection.
"""
import argparse
from dataclasses import asdict, replace
from hashlib import sha256
import json
import math
from pathlib import Path
from time import perf_counter_ns

from final_project.behaviors.overhead_ingress import IngressCore, MAX_FRAME_AGE_S
from final_project.overhead_perception.fieldcam.filters import CVFilter2D, YawFilter
from final_project.overhead_perception.fieldcam.net import REPLY_PROTOCOL, STATE_PROTOCOL
from .integration import Executor
from .kick_handoff import KickHandoff
from .model import Field, Limits, zero
from .simulation import Simulator, SimulatedKick


ROBOT = "pupper-1"
EPOCH = "offline-kick"
DT = .02
CAMERA_DELAY = .060
DROPOUT = (1.0, 1.8)
SPEEDS = (.15, .30, .45)
SCENARIOS = ("center", "offset", "wrong_side")


def instrument_camera(field):
    return IngressCore(source_id="synthetic-overhead", source_session="rehearsal",
                       producer_id="perception", epoch=EPOCH, boot_id="virtual-pi",
                       robot_map={"striker": ROBOT}, calibration_revision=1, field=asdict(field))


def sync_clocks(ingress, now):
    # Synthetic source monotonic clock is 10 s ahead; exercise a 2 ms probe round trip.
    probe = ingress.make_probe(now - .002)
    ingress.accept_probe({**ingress.identity(), "nonce": probe["nonce"], "protocol": REPLY_PROTOCOL,
                          "received_mono": now + 10 - .001, "sent_mono": now + 10 - .001}, now)


def camera_datagram(ingress, observation, field, sequence, now, filters=None):
    """Encode fieldcam's wire schema before production decoding/admission; no image detector."""
    capture = 1000. + observation.captured_at
    metadata = {"visible": True, "measured_this_frame": True, "measured_at": capture, "age_s": 0.}
    robot = {**metadata, **asdict(observation.robot), "vx": 0., "vy": 0., "wz": 0.}
    ball = {**metadata, **asdict(observation.ball)} if observation.ball else {"visible": False}
    if filters is not None:
        # Same filters and parameter values as fieldcam's field.yaml. Synthetic
        # measurements enter just after image-space localization for this noisy case.
        robot_xy, ball_xy, yaw_filter = filters
        for row, tracker in ((robot, robot_xy), (ball, ball_xy)):
            if row.get("visible"):
                accepted = tracker.update((row["x"], row["y"]), capture)
                x, y, vx, vy = tracker.predict_to(capture)
                row.update(x=float(x), y=float(y), vx=float(vx), vy=float(vy),
                           measured_this_frame=accepted, measured_at=tracker.t_meas)
        yaw_filter.update(robot["yaw"], capture)
        robot.update(yaw=yaw_filter.yaw, wz=yaw_filter.rate)
    goals = {side: {"x": sign * field.length / 2,
                    "y_min": -field.goal_width / 2, "y_max": field.goal_width / 2}
             for side, sign in (("west", -1), ("east", 1))}
    # The synthetic camera emits the measured position, without extrapolation.
    # The transport timestamps preserve elapsed virtual time since capture.
    packet = {"seq": sequence, "calibration_valid": True, "calibration_revision": 1,
              "field": asdict(field), "goals": goals, "robots": {"striker": robot}, "ball": ball,
              "captured_at": capture, "stamp": capture, "frame_age_s": 0.,
              "_transport": {**ingress.identity(), "clock_valid": True, "live": True,
                             "protocol": STATE_PROTOCOL, "wire_seq": sequence, "sent_mono": 10. + now,
                             "sent_wall": 1000. + now, "pair_span_s": 0., "clock_start_wall": 1000.}}
    return json.dumps(packet, separators=(",", ":"), allow_nan=False).encode()


def endpoint(sim, side):
    """Independent truth measurements use the nominal fixture's dimensions."""
    c, s = math.cos(sim.robot.yaw), math.sin(sim.robot.yaw)
    dx, dy = sim.ball.x - sim.robot.x, sim.ball.y - sim.robot.y
    yaw = math.atan2(-sim.ball.y, sim.field.attack_sign * sim.field.length / 2 - sim.ball.x)
    yaw_error = math.atan2(math.sin(yaw - sim.robot.yaw), math.cos(yaw - sim.robot.yaw))
    return {"forward_error_m": round(dx * c + dy * s - .258, 5),
            "lateral_error_m": round(-dx * s + dy * c - (.040 if side == "left" else -.046), 5),
            "heading_error_deg": round(math.degrees(yaw_error), 3),
            "body_speed_m_s": round(math.hypot(sim.velocity.vx, sim.velocity.vy), 5)}


def quantile(samples, probability):
    ordered = sorted(samples)
    return ordered[math.ceil(len(ordered) * probability) - 1] if ordered else None


def run_case(scenario, side, speed, *, fault=None, attack_sign=1, camera_delay=CAMERA_DELAY,
             noise=0., walk_after=False):
    field = Field(goal_width=.6096, ball_radius=.11, attack_sign=attack_sign)
    limits = Limits(vx=speed, vy=speed, wz=1., acceleration=.65, yaw_acceleration=2.)
    sim = Simulator(field, limits, scenario=scenario, bumper_contact=False)
    ingress = instrument_camera(field)
    executor = Executor(fields={ROBOT: field}, limits=limits, kick_side=side)
    executor.arm(EPOCH, ingress.clock_name, sim.time)
    handoff = KickHandoff(ROBOT)
    actuator = SimulatedKick(handoff)
    # Populate camera history before walking. Only induced delay, on the virtual clock.
    for _ in range(math.ceil(camera_delay / DT) + 2):
        sim.step(zero(sim.time), DT)
    recovery = recovery_origin = None
    filters = (CVFilter2D(2., .01), CVFilter2D(4., .01), YawFilter()) if noise else None
    ball_origin = (sim.ball.x, sim.ball.y)
    max_ball_shift = max_speed = 0.
    moving_during_clip = requests = 0
    commit = finished = hold_at = reacquired_at = None
    end_pose, last_phase = None, "initial"
    close_speeds, input_ages, compute_ms = [], [], []
    max_v_change = 0.
    prev_v = (0., 0.)
    motion_in_silent_hold = 0
    for tick in range(1500):
        now = sim.time
        elapsed = round(tick * DT, 6)
        if fault == "ball_reposition" and tick == 50:
            sim.move_ball(sim.ball.x, sim.ball.y + .32)
            ball_origin = (sim.ball.x, sim.ball.y)
        if tick % 25 == 0:
            sync_clocks(ingress, now)
        if tick % 10 == 0 or (walk_after and finished is not None
                              and math.isclose(elapsed, finished + DT, abs_tol=1e-8)):
            action = ({"kind": "move", "intent_id": "leave-the-ball",
                       "target": {"x": -.54, "y": .20, "yaw": .40}}
                      if recovery_origin else {"kind": "attack", "intent_id": "one-shot"})
            result = executor.handle({"event": "intent", "epoch": EPOCH, "clock": ingress.clock_name,
                                      "seq": tick, "now": now, "robot_id": ROBOT, **action,
                                      "issued_at": now, "valid_until": now + .40})
            if not result["accepted"]:
                raise RuntimeError(result["reason"])
        silent = fault == "camera_silence" and DROPOUT[0] <= elapsed < DROPOUT[1]
        timed_start = None
        if tick % 2 == 0 and not silent:
            obs = sim.observe(delay=camera_delay, noise=noise)
            datagram = camera_datagram(ingress, obs, field, tick, now, filters)
            timed_start = perf_counter_ns()
            frame = ingress.normalize(json.loads(datagram), now)
            result = executor.handle({**frame, "now": now})
            if not result["accepted"]:
                raise RuntimeError(result["reason"])
            input_ages.append((now - frame["captured_at"]) * 1000.)
        else:
            expired = ingress.expire(now)
            if expired:
                executor.handle({**expired, "now": now})
        step = executor.kick_step(handoff, now)
        if timed_start is not None:
            compute_ms.append((perf_counter_ns() - timed_start) / 1e6)
        actuator.update(step, now)
        command = step.decision.command
        planar = math.hypot(command.vx, command.vy)
        moving = planar > 1e-8 or abs(command.wz) > 1e-8
        if step.request:
            requests += 1
            if commit is None:
                commit, end_pose = elapsed, endpoint(sim, side)
        if commit is not None and finished is None and moving:
            moving_during_clip += 1
        last_phase = step.decision.phase
        if last_phase == "kick_complete":
            if finished is None:
                finished = elapsed
            if not walk_after:
                break
            recovery_origin = sim.robot
        if recovery_origin and last_phase == "target_reached":
            recovery = {"target": [-.54, .20, .40], "displacement_m": round(math.hypot(
                        sim.robot.x - recovery_origin.x, sim.robot.y - recovery_origin.y), 4),
                        "yaw_change_deg": round(math.degrees(math.remainder(
                        sim.robot.yaw - recovery_origin.yaw, 2 * math.pi)), 2),
                        "position_error_m": round(math.hypot(sim.robot.x + .54, sim.robot.y - .20), 4),
                        "yaw_error_deg": round(math.degrees(math.remainder(
                        sim.robot.yaw - .40, 2 * math.pi)), 2)}
            break
        if silent and last_phase == "tracking_lost" and not moving and hold_at is None:
            hold_at = elapsed
        if hold_at is not None and silent and moving:
            motion_in_silent_hold += 1
        if hold_at is not None and elapsed >= DROPOUT[1] and moving and reacquired_at is None:
            reacquired_at = elapsed
        if commit is None:
            max_speed = max(max_speed, planar)
            max_v_change = max(max_v_change, math.hypot(command.vx - prev_v[0], command.vy - prev_v[1]))
            metrics = endpoint(sim, side)
            if math.hypot(metrics["forward_error_m"], metrics["lateral_error_m"]) <= .10:
                close_speeds.append(planar)
            max_ball_shift = max(max_ball_shift, math.hypot(sim.ball.x - ball_origin[0],
                                                           sim.ball.y - ball_origin[1]))
        prev_v = command.vx, command.vy
        plant_command = command
        if fault == "reduced_gait_response" and elapsed >= 1.0:
            # The planner sees the under-travel only in subsequent camera frames.
            plant_command = replace(command, vx=.70 * command.vx,
                                    vy=.70 * command.vy, wz=.65 * command.wz)
        sim.step(plant_command, DT)
    if end_pose is None:
        end_pose = endpoint(sim, side)
    valid = (finished is not None and requests == 1 and moving_during_clip == 0
             and abs(end_pose["forward_error_m"]) <= .025 and abs(end_pose["lateral_error_m"]) <= .018
             and abs(end_pose["heading_error_deg"]) <= 5 and end_pose["body_speed_m_s"] <= .04
             and max_ball_shift <= .01)
    if fault == "camera_silence":
        valid = (valid and hold_at is not None and hold_at <= DROPOUT[0] + MAX_FRAME_AGE_S + DT
                 and reacquired_at is not None and motion_in_silent_hold == 0)
    if walk_after:
        valid = (valid and recovery is not None and recovery["displacement_m"] >= .15
                 and recovery["position_error_m"] <= .075 and abs(recovery["yaw_error_deg"]) <= 10)
    return {"scenario": scenario, "side": side, "speed_cap_m_s": speed, "attack_sign": attack_sign,
            "fault": fault, "capture_delay_s": camera_delay, "position_noise_std_m": noise,
            "post_kick_move": recovery, "passed": valid, "terminal_phase": last_phase,
            "time_to_commit_s": commit, "mock_walker_restored_s": finished, "requests": requests,
            "motion_commands_during_kick": moving_during_clip, "truth_at_commit_or_cutoff": end_pose,
            "peak_command_m_s": round(max_speed, 4),
            "median_command_within_10cm_m_s": round(quantile(close_speeds, .50), 4) if close_speeds else None,
            "max_planar_command_step_m_s": round(max_v_change, 4),
            "max_uncommanded_ball_shift_m": round(max_ball_shift, 4),
            "camera_hold_s": hold_at, "resumed_steering_s": reacquired_at,
            "moving_commands_during_silent_hold": motion_in_silent_hold,
            "admitted_frames": len(compute_ms),
            "capture_age_p95_ms": round(quantile(input_ages, .95), 3),
            "decode_to_decision_p95_ms": round(quantile(compute_ms, .95), 3)}


def run_matrix():
    runs = [run_case(start, side, speed, walk_after=(start, side, speed) == ("center", "left", .30))
            for start in SCENARIOS
            for side in ("left", "right") for speed in SPEEDS]
    runs.extend(run_case("offset", "right", .30, fault=fault, attack_sign=-1)
                for fault in ("camera_silence", "ball_reposition"))
    runs.append(run_case("offset", "right", .30, camera_delay=.10, noise=.002))
    runs.append(run_case("offset", "left", .45, fault="reduced_gait_response"))
    project_dir = Path(__file__).resolve().parents[1]
    sources = ("behaviors/overhead_ingress.py", "overhead_perception/fieldcam/filters.py",
               "overhead_perception/fieldcam/net.py", "pupper_fc/model.py", "pupper_fc/controller.py",
               "pupper_fc/paw_approach.py", "pupper_fc/integration.py", "pupper_fc/kick_handoff.py",
               "pupper_fc/simulation.py", "pupper_fc/kick_rehearsal.py")
    return {"source_sha256": {path: sha256((project_dir / path).read_bytes()).hexdigest() for path in sources},
            "scope": "Single-robot 2D acceleration-limited body, passive disc collision, synthetic camera tracks, real clock/frame admission, executor and handoff, simulated clip and walker receipts. No optical detection, native gait, kicks, ball impacts or network measured.",
            "source": "python -m final_project.pupper_fc.kick_rehearsal",
            "control_hz": 1 / DT, "camera_hz": 1 / (2 * DT),
            "nominal_virtual_capture_delay_ms": CAMERA_DELAY * 1000,
            "reduced_gait_response": "From 1 s onward the plant delivers 70% of commanded translation and 65% of yaw rate. Illustrative sensitivity check.",
            "plant": "3.66 × 2.44 m, 0.6096 m goals, 0.11 m ball radius, 0.16 m body radius. Test assumptions.",
            "criteria": "One kick, exclusive legs through restored walker, true fore/aft error ≤25 mm, lateral ≤18 mm, yaw ≤5°, speed ≤0.04 m/s and precommit ball drift ≤10 mm. Silent-source case must stop and reacquire. One case commands a fresh field pose after walker restoration; one adds 2 mm Gaussian coordinate noise with fieldcam tracking and 100 ms capture delay.",
            "timing": "Per-case host p95 from decoding synthetic fieldcam JSON to issuing the handoff decision. Excludes network, image processing, actuator and instrumentation outside decision path.",
            "passed": sum(row["passed"] for row in runs), "total": len(runs), "runs": runs}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write the complete JSON evidence to this path")
    args = parser.parse_args()
    report = run_matrix()
    data = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(data)
    else:
        print(data, end="")
    return int(report["passed"] != report["total"])


if __name__ == "__main__":
    raise SystemExit(main())
