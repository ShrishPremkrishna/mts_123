"""Compare released reactive steering and current intercept through the FC policy-cycle harness.

Run from the repository root with PYTHONPATH=pupper-mjlab/src and the numerical
Python environment. The original release and candidate consume identically timed
camera observations, identical noise and identical ball velocities at each pass.
"""
import argparse
import hashlib
import asyncio
from collections import deque
from dataclasses import asdict, dataclass, replace
from importlib import import_module
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import ModuleType
from unittest.mock import patch

import mujoco
import numpy as np

import verify_fc_policy_cycle as cycle
from final_project.overhead_perception.fieldcam.filters import CVFilter2D, YawFilter
from final_project.pupper_fc import integration, model as field_model
from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, paw_pose

BASELINE = "55ecbd032b75dabdd6c3f42e1422504d721fec47"
START = 4.2  # Original policy-cycle harness starts the attack after supported startup.
GOAL = (field_model.Field().length / 2, 0.)
RADIUS = .11
ROLLING_FRICTION = .0009  # m, unmeasured rotational friction arm in the ball/terrain pair.


@dataclass(frozen=True)
class Shot:
    name: str
    side: str
    ball_xy: tuple
    velocity_xy: tuple
    second_pass: tuple = ()
    noise_m: float = 0.
    extra_camera_ticks: int = 0


SHOTS = (
    Shot("stock_stationary", "left", (0., .04), (0., 0.)),
    Shot("stationary", "left", (0., .04), (0., 0.)),
    Shot("crossing", "left", (-.045, -.23), (0., .21)),
    Shot("diagonal", "right", (-.06, .21), (.085, -.18)),
    Shot("redirect_noise", "left", (-.075, -.22), (.025, .22), (1.15, .015, -.205), .002, 40),
    Shot("fast_crossing", "right", (-.04, -.35), (.02, .42)),
)


# Optional one-factor probes isolate camera age and localization error for the
# deflection case. They do not enter the six predeclared comparison scenarios.
REDIRECT = next(s for s in SHOTS if s.name == "redirect_noise")
PROBES = (
    replace(REDIRECT, name="redirect_short_latency", extra_camera_ticks=0),
    replace(REDIRECT, name="redirect_exact_delayed", noise_m=0.),
    replace(SHOTS[0], name="stock_stationary_delayed", extra_camera_ticks=40),
)


def released_controller():
    """Isolate both old planner modules while sharing the unchanged message types."""
    name = "pupper_fc_released_intercept_test"
    package = ModuleType(name)
    package.__path__ = []
    sys.modules[name] = package
    sys.modules[name + ".model"] = field_model
    for part in ("paw_approach", "controller"):
        path = f"final_project/pupper_fc/{part}.py"
        source = subprocess.check_output(["git", "show", f"{BASELINE}:{path}"], cwd=cycle.ROOT)
        module = ModuleType(f"{name}.{part}")
        module.__package__ = name
        sys.modules[module.__name__] = module
        exec(compile(source, f"{BASELINE}:{path}", "exec"), module.__dict__)
    return sys.modules[name + ".controller"].Controller


def scene(spec):
    cycle.add_soccer_ball(spec)
    # Apply resistance only to the ball/floor contact; retain release paw/ball
    # geometry, deformation parameters, ground friction and actuator dynamics.
    spec.add_pair(name="rolling_ball_ground", geomname1="terrain", geomname2="ball_surface",
                  condim=6, friction=[.6, .6, .005, ROLLING_FRICTION, ROLLING_FRICTION],
                  solref=[.01, .5])


class RollingTrial(cycle.PolicyPlant):
    def __init__(self, model, binary, shot, params):
        super().__init__(model, binary, shot.side, params)
        self.shot = shot
        self.data.qpos[self.bq:self.bq + 2] = shot.ball_xy
        mujoco.mj_forward(model, self.data)
        self.trace = []
        self.sensor_history = deque(maxlen=16)
        self.decision = None

    def roll(self, vx, vy):
        # Consistent no-slip initial spin: the bottom point is stationary.
        self.data.qvel[self.bv:self.bv + 6] = [vx, vy, 0., -vy / RADIUS, vx / RADIUS, 0.]

    def advance(self, command):
        elapsed = self.now - START
        if self.ticks == round(START * cycle.HZ):
            self.roll(*self.shot.velocity_xy)
        if self.shot.second_pass:
            t, vx, vy = self.shot.second_pass
            if self.ticks == round((START + t) * cycle.HZ):
                self.roll(vx, vy)
        if self.ticks % 10 == 0:
            self.sensor_history.append((self.ticks, self.pose(), self.ball()))
        if elapsed >= 0 and self.ticks % 10 == 0:
            ball, pose = self.ball(), self.pose()
            aim = paw_pose(ball, GOAL, self.shot.side, forward_m=COMBINED_PAW_FORWARD_M[self.shot.side])
            self.trace.append(dict(t=round(elapsed, 5), ball=asdict(ball), torso=asdict(pose),
                                   true_paw_stance_error_m=math.hypot(aim.x - pose.x, aim.y - pose.y),
                                   true_yaw_error_rad=abs(math.remainder(aim.yaw - pose.yaw, math.tau)),
                                   target=self.decision.target if self.decision else None,
                                   phase=self.decision.phase if self.decision else None,
                                   steering=self.decision.reason if self.decision else None,
                                   owner=self.controller))
        return super().advance(command)


def outcome(plant, controller):
    shot = plant.shot
    durations_us = []
    observation_ages_ms = []
    source = cycle.camera_datagram
    rng = np.random.default_rng(711)
    filters = (CVFilter2D(2., .01), CVFilter2D(4., .01), YawFilter()) if shot.noise_m else None

    # Two extra camera periods reproduce approximately 135 ms source age in
    # the noisy redirection case. No captured_at value is renumbered.
    def camera(ingress, observation, field, sequence, now):
        if shot.extra_camera_ticks:
            observed_tick = plant.ticks - round((now - observation.captured_at) * cycle.HZ) - shot.extra_camera_ticks
            stamp, pose, ball = min(plant.sensor_history, key=lambda sample: abs(sample[0] - observed_tick))
            observation = replace(observation, captured_at=now - (plant.ticks - stamp) / cycle.HZ, robot=pose, ball=ball)
        if shot.noise_m:
            x, y = rng.normal(0., shot.noise_m, 2)
            ball = replace(observation.ball, x=observation.ball.x + x, y=observation.ball.y + y)
            observation = replace(observation, ball=ball)
        return source(ingress, observation, field, sequence, now, filters)

    class MeasuredController(controller):
        def tick(self, observation, now, **options):
            begin = time.perf_counter_ns()
            decision = super().tick(observation, now, **options)
            elapsed = time.perf_counter_ns() - begin
            if observation.robot_id == cycle.ROBOT:
                plant.decision = decision
                durations_us.append(elapsed / 1000)
                observation_ages_ms.append((now - observation.captured_at) * 1000)
            return decision

    # Order command publications inside each physics tick. The frozen virtual
    # service clock otherwise gives a Stop and the following zero the same
    # nanosecond. Use the last microsecond before the guard's receipt, so the
    # original encoder and watchdog still reject future/expired/replayed sends.
    class PhysicsBridge(cycle.BridgeCore):
        def __init__(self, *args, **options):
            virtual_ns = options["monotonic_ns"]
            last_issue = 0

            def issue_ns():
                nonlocal last_issue
                observed = virtual_ns()
                last_issue = max(observed - 1000, last_issue + 1)
                if last_issue > observed:
                    raise RuntimeError("over 1000 command publications in one simulated instant")
                return last_issue

            super().__init__(*args, **dict(options, monotonic_ns=issue_ns))

    with (patch.object(integration, "Controller", MeasuredController), patch.object(cycle, "camera_datagram", camera),
          patch.object(cycle, "BridgeCore", PhysicsBridge)):
        result = asyncio.run(cycle.run_case(plant, shot.side, COMBINED_PAW_FORWARD_M[shot.side]))
    first = result["first_robot_ball"]
    stop = first["time_s"] - START if first else math.inf
    rolling = [row for row in plant.trace if row["t"] <= stop and row["owner"] == "gemini_walk" and
               math.hypot(row["ball"]["vx"], row["ball"]["vy"]) >= .035]
    near = next((r for r in rolling if r["true_paw_stance_error_m"] <= .08 and r["true_yaw_error_rad"] <= .20), None)
    early = [r["true_paw_stance_error_m"] for r in plant.trace if r["t"] <= min(2.5, stop)]
    selected_paw = bool(first and first["controller"] == "trick_leg_kick" and
                        first["geom"].startswith("leg_front_" + shot.side[0] + "_3"))
    def quantiles(values):
        if not values:
            return {}
        return {name: float(np.percentile(values, q))
                for name, q in [("p50", 50), ("p95", 95), ("p99", 99), ("max", 100)]}
    return dict(shot=asdict(shot), controller_calls=len(durations_us), host_planner_us=quantiles(durations_us),
                observation_age_at_control_ms=quantiles(observation_ages_ms), first_rolling_approach_within_8cm_s=near and near["t"],
                min_rolling_stance_error_m=min((r["true_paw_stance_error_m"] for r in rolling), default=None),
                mean_stance_error_first_2_5s_m=float(np.mean(early)) if early else None,
                max_ball_speed_prior_to_contact_m_s=max((math.hypot(r["ball"]["vx"], r["ball"]["vy"])
                                                        for r in plant.trace if r["t"] <= stop), default=0.),
                first_contact=first, contact_by_selected_kick=selected_paw, goal_crossed_s=result["goal_crossed_s"],
                walker_restored=result["operator_evidence"] is not None and result["queue_outcome"] == "succeeded",
                kick_requests=result["kick_requests"], failure=result["failure"], events=result["events"],
                peak_tilt_deg=result["max_tilt_deg"], last_phase=result["terminal_phase"])



def write_comparison(out):
    rows = []
    for shot in SHOTS:
        paths = [out / f"{name}-{shot.name}.json" for name in ("released", "intercept")]
        if not all(path.exists() for path in paths):
            continue
        released, candidate = [json.loads(path.read_text()) for path in paths]
        tracks = [[json.loads(line) for line in path.with_suffix(".jsonl").read_text().splitlines()] for path in paths]
        contact_times = [run["first_contact"]["time_s"] - START if run["first_contact"] else math.inf
                         for run in (released, candidate)]
        shared = [(a, b) for a, b in zip(*tracks)
                  if a["t"] == b["t"] and a["t"] < min(contact_times)]
        delta = max((math.hypot(a["ball"]["x"] - b["ball"]["x"], a["ball"]["y"] - b["ball"]["y"])
                     for a, b in shared), default=0.)
        if not shared or delta > 1e-7:
            raise ValueError(f"Ball trajectories differed before any robot contact on {shot.name}")
        def metrics(run, track):
            contact = run["first_contact"]
            touch = contact["time_s"] - START if contact else math.inf
            after = [r["ball"]["vx"] for r in track if touch <= r["t"] < touch + 1.5]
            return dict(rolling_8cm_s=run["first_rolling_approach_within_8cm_s"],
                        min_rolling_stance_error_m=run["min_rolling_stance_error_m"],
                        mean_stance_error_first_2_5s_m=run["mean_stance_error_first_2_5s_m"],
                        first_contact_s=contact["time_s"] - START if contact else None,
                        first_contact_geom=contact["geom"] if contact else None,
                        first_contact_owner=contact["controller"] if contact else None,
                        sampled_peak_eastward_ball_speed_after_first_contact_m_s=max(after, default=None),
                        requested_kicks=run["kick_requests"], correct_paw_kick=run["contact_by_selected_kick"],
                        goal_s=run["goal_crossed_s"] - START if run["goal_crossed_s"] is not None else None,
                        restored_walk=run["walker_restored"], failure=run["failure"])
        reasons = [r.get("steering") or "" for r in tracks[1] if r["t"] < contact_times[1]]
        rows.append(dict(shot=shot.name, matched_ball_positions_until_first_contact=len(shared),
                         maximum_precontact_ab_ball_path_difference_m=delta, released=metrics(released, tracks[0]),
                         intercept=metrics(candidate, tracks[1]),
                         predictive_steering_trace_ticks=sum("moving-ball" in reason for reason in reasons)))
    if not rows:
        return
    outcomes = {}
    moving = [r for r in rows if "stationary" not in r["shot"]]
    for mode in ("released", "intercept"):
        results = [r[mode] for r in moving]
        outcomes[mode] = dict(
            moving_trials=len(results), correct_paw_kicks=sum(r["correct_paw_kick"] for r in results),
            walking_contacts=sum(r["first_contact_owner"] == "gemini_walk" for r in results),
            no_contact=sum(r["first_contact_owner"] is None for r in results),
            goals=sum(r["goal_s"] is not None for r in results),
        )
    summary=dict(moving_outcomes=outcomes, baseline=BASELINE, cases=rows, times="Simulated seconds since the attack opened after 4.2 seconds of startup.",
                 success_measure="Actual first contact by the selected leg during its native clip; goal uses complete ball crossing inside goalposts. A walking touch is reported separately.",
                 proxy_measure="Distance from the actual torso to the correct instantaneous paw stance. The 8 cm approach also requires at most 0.20 rad heading error while the uncontacted ball is moving at least 0.035 m/s. This is not a kick or possession.",
                 assumptions=["Released RTNeural walker and kick, 520 Hz free-body MuJoCo, 16 ms modeled actuator delay, calibrated field frame synthesized from model state.",
                              "All trials use the same field ingress, executor, queue, command guard and emulated switch services. No ROS network, physical actuator, or image detector is used.",
                              "Simulated field is 3.66 by 2.44 m, goal width 0.6096 m, ball radius 0.11 m and mass 0.43 kg. Moving trials assume additional rolling resistance; stock stationary control uses the released contact model.",
                              "Noisy redirection injects a second independent ball velocity at 1.15 s, 2 mm Gaussian planar localization noise, production fieldcam filters and approximately 135 ms capture age. Other cases use delayed exact model state and speed.",
                              "Only six deterministic scenarios. Numbers do not estimate match or real-world physical success probabilities."])
    (out / "closed-loop-comparison.json").write_text(json.dumps(summary,indent=2) + "\n")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--strategy", choices=("released", "intercept", "both"), default="both")
    parser.add_argument("--native-host", type=Path, help="reuse an already built handoff_controller")
    parser.add_argument("--staged-release", type=Path, help="verify that the controller/gait selection matches a staged release")
    parser.add_argument("--shots", nargs="+", choices=[s.name for s in (*SHOTS, *PROBES)])
    args = parser.parse_args()
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    os.environ.update(PUPPER_SESSION_MOTION_MODE="fc_walk_kick", PUPPER_SESSION_SKILLS="leg_kick",
                      PUPPER_FOLLOWER_MODE="disabled", PUPPER_FC_RECORDING="events")
    shots = tuple(s for s in (*SHOTS, *PROBES) if s.name in args.shots) if args.shots else SHOTS
    models = {}
    def surface_for(shot):
        return "stock" if shot.name.startswith("stock_stationary") else "rolling"

    for shot in shots:
        surface = surface_for(shot)
        if surface not in models:
            configure = cycle.add_soccer_ball if surface == "stock" else scene
            models[surface] = cycle.make_model("full_hulls", configure_scene=configure)
    with tempfile.TemporaryDirectory() as scratch:
        work = Path(scratch)
        params, release = cycle.selected_profile(work, args.staged_release, cycle.selection())
        binary = args.native_host or cycle.build_controller(work)[0]
        for strategy in (("released", "intercept") if args.strategy == "both" else (args.strategy,)):
            controller = released_controller() if strategy == "released" else import_module("final_project.pupper_fc.controller").Controller
            source_hashes = {}
            for filename in ("controller.py", "paw_approach.py"):
                path = "final_project/pupper_fc/" + filename
                data = (subprocess.check_output(["git", "show", f"{BASELINE}:{path}"], cwd=cycle.ROOT)
                        if strategy == "released" else (cycle.ROOT / path).read_bytes())
                source_hashes[filename] = hashlib.sha256(data).hexdigest()
            for shot in shots:
                surface = surface_for(shot)
                plant_model, contact = models[surface]
                plant = RollingTrial(plant_model, binary, shot, params)
                try:
                    result = outcome(plant, controller)
                    result.update(
                        strategy=strategy, baseline=BASELINE, planner_sha256=source_hashes, release=release,
                        policy_host_sha256=cycle.sha(binary), walking_policy_sha256=cycle.sha(cycle.WALK),
                        kick_policy_sha256=cycle.sha(cycle.KICK), mujoco=mujoco.__version__, contact_model=contact,
                        rolling_friction_m=0. if surface == "stock" else ROLLING_FRICTION,
                    )
                    (out / f"{strategy}-{shot.name}.json").write_text(json.dumps(result, indent=2) + "\n")
                    (out / f"{strategy}-{shot.name}.jsonl").write_text("".join(json.dumps(r) + "\n" for r in plant.trace))
                    print(strategy, shot.name, "first", result["first_contact"] and result["first_contact"]["geom"],
                          "rolling range", result["min_rolling_stance_error_m"], "goal", result["goal_crossed_s"],
                          "failure", result["failure"], flush=True)
                finally:
                    plant.close()

    write_comparison(out)


if __name__ == "__main__":
    main()
