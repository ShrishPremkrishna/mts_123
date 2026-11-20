"""One continuous MuJoCo body under the released RTNeural walker and leg-kick policies.

Run from the repo root with the numerical environment and PYTHONPATH=pupper-mjlab/src.
This host diagnostic uses the production policy/clip math in handoff_controller.cpp.
The field executor, operator queue and C++ policy/clip math run on one virtual clock.
Camera tracks come from MuJoCo. ROS transport, hardware and actuator timing are modeled.
Use --staged-release to require identical bytes for every file in a prepared release.
"""

from __future__ import annotations

import argparse
import asyncio
import os
from collections import Counter, deque
from dataclasses import asdict, replace
from hashlib import sha256
from types import SimpleNamespace
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

import mujoco
import numpy as np
import yaml

from check_spider_handoff import build_controller, make_model, numerical_failures, sha

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from final_project.behaviors import body_command_guard as guard_api
from final_project.behaviors.fc_executor_bridge import BridgeCore, DEADLINE_CONTRACT, TICK_S
from final_project.pupper_fc.kick_rehearsal import (
    EPOCH,
    ROBOT,
    camera_datagram,
    instrument_camera,
    sync_clocks,
)
from final_project.pupper_fc.model import Ball, Command, Field, Observation, Pose
from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, PAW_FORWARD_M, PAW_LEFT_M

WALK = ROOT / "pupper_gait_deploy/policies/default_policy.json"
KICK = ROOT / "gemini_pupper/tricks/operator/leg_kick/policy.json"
DRIVER = ROOT / "foundation/configs/class-driver-joint-limits.json"
HZ = 520


def add_soccer_ball(spec):
    # Approximates a net backstop behind the two-foot goal mouth.
    spec.worldbody.add_geom(
        name="goal_back",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=(2.12, 0.0, 0.15),
        size=(0.02, 0.32, 0.15),
        contype=1,
        conaffinity=1,
    )
    body = spec.worldbody.add_body(name="soccer_ball")
    body.add_freejoint(name="ball_joint")
    body.add_geom(
        name="ball_surface",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=(0.11, 0, 0),
        mass=0.43,
        condim=3,
        friction=(0.6, 0.005, 0.0001),
        solref=(0.01, 0.5),
        priority=2,
        contype=1,
        conaffinity=1,
    )


class PolicyPlant:
    """MuJoCo state is reset once per trial, before any command. The ball always stays live."""

    def __init__(self, model, binary, side, params, *, y=0.0, heading=0.0):
        from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
        from mjlab.tasks.pupper.deployment_contract import load_driver_target_limits

        self.model, self.data = model, mujoco.MjData(model)
        joints = [model.joint(name).id for name in JOINT_NAMES]
        self.qadr, self.vadr = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
        self.controls = [int(np.flatnonzero(model.actuator_trnid[:, 0] == j)[0]) for j in joints]
        self.bq = int(model.joint("ball_joint").qposadr[0])
        self.bv = int(model.joint("ball_joint").dofadr[0])
        (root,) = [
            j
            for j in range(model.njnt)
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE and j != model.joint("ball_joint").id
        ]
        self.rq, self.rv = map(int, (model.jnt_qposadr[root], model.jnt_dofadr[root]))
        bounds, _ = load_driver_target_limits(DRIVER)
        self.lower, self.upper = np.asarray([bounds[n] for n in JOINT_NAMES]).T
        q = np.array(json.loads(WALK.read_text())["default_joint_pos"])
        self.data.qpos[self.rq : self.rq + 7] = [
            -0.6,
            y,
            0.18,
            math.cos(heading / 2),
            0,
            0,
            math.sin(heading / 2),
        ]
        self.data.qpos[self.bq : self.bq + 7] = [0.0, PAW_LEFT_M[side], 0.11, 1, 0, 0, 0]
        self.data.qpos[self.qadr] = q
        mujoco.mj_forward(model, self.data)
        self.ticks = 0
        self.delayed = deque([(-1.0, q), (0.0, q)])
        self.inferences = Counter()
        self.contact = {}
        self.first_robot_ball = None
        self.command = Command()
        self.peak_tilt = 0.0
        self.low_height = self.data.qpos[self.rq + 2]
        self.controller = None
        self.params = params
        self.proc = subprocess.Popen(
            [
                str(binary),
                params["trick_leg_kick"]["model_path"],
                params["trick_leg_kick"]["model_path"],
                "--return-default-no-jump",
                params["gemini_walk"]["model_path"],
            ],
            text=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            bufsize=1,
        )

    def send(self, command):
        self.command = command

    def close(self):
        self.proc.stdin.close()
        if self.proc.wait(timeout=5) != 0:
            raise RuntimeError("policy host failed")
        self.proc.stdout.close()

    @property
    def now(self):
        return self.ticks / HZ

    def rpc(self, request):
        self.proc.stdin.write(json.dumps(request) + "\n")
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError("policy host exited")
        return json.loads(line)

    def activate(self, side=None, execution_id="none"):
        self.controller = "trick_leg_kick" if side else "gemini_walk"
        self.activated = self.now
        self.bound_zero = None
        self.rpc(
            dict(
                op="activate",
                controller="trick" if side else "default",
                joint_pos=self.data.qpos[self.qadr].tolist(),
                start_clip=f"kick_{side}" if side else "",
                clip_anchor=bool(side),
                execution_id=execution_id,
                init_s=self.params[self.controller]["init_duration"],
                fade_s=self.params[self.controller]["fade_in_duration"],
            )
        )

    def pose(self):
        x, y, _, w, a, b, c = self.data.qpos[self.rq : self.rq + 7]
        return Pose(x, y, math.atan2(2 * (w * c + a * b), 1 - 2 * (b * b + c * c)))

    def ball(self):
        x, y = self.data.qpos[self.bq : self.bq + 2]
        vx, vy = self.data.qvel[self.bv : self.bv + 2]
        return Ball(x, y, vx, vy)

    def advance(self, command):
        d = self.data
        quat = d.qpos[self.rq + 3 : self.rq + 7]
        state = self.rpc(
            dict(
                op="step",
                elapsed_s=self.now - self.activated,
                command=command,
                quaternion=quat.tolist(),
                angular_velocity=d.qvel[self.rv + 3 : self.rv + 6].tolist(),
                joint_pos=d.qpos[self.qadr].tolist(),
                joint_vel=d.qvel[self.vadr].tolist(),
            )
        )
        if state["inference"]:
            self.inferences[self.controller] += 1
        self.delayed.append((self.now, np.clip(state["target"], self.lower, self.upper)))
        cutoff = self.now - 0.016
        while len(self.delayed) > 2 and self.delayed[1][0] <= cutoff:
            self.delayed.popleft()
        (t0, v0), (t1, v1) = self.delayed[0], self.delayed[1]
        d.ctrl[self.controls] = v0 + np.clip((cutoff - t0) / (t1 - t0), 0, 1) * (v1 - v0)
        self.model.actuator_gainprm[self.controls, 0] = state["kp"]
        self.model.actuator_biasprm[self.controls, 1] = -state["kp"]
        self.model.actuator_biasprm[self.controls, 2] = -state["kd"]
        mujoco.mj_step(self.model, d)
        self.ticks += 1
        for contact in d.contact:
            a, b = self.model.geom(int(contact.geom1)).name, self.model.geom(int(contact.geom2)).name
            other = b if a == "ball_surface" else a if b == "ball_surface" else None
            if other and other != "terrain":
                self.contact.setdefault(
                    other, [round(self.now, 4), self.controller, state.get("clip"), state.get("frame")]
                )
                if self.first_robot_ball is None and other != "goal_back":
                    self.first_robot_ball = dict(
                        time_s=round(self.now, 4),
                        geom=other,
                        controller=self.controller,
                        clip=state.get("clip"),
                        frame=state.get("frame"),
                        robot=asdict(self.pose()),
                        ball=asdict(self.ball()),
                        requested_velocity=command,
                        base_height_m=float(d.qpos[self.rq + 2]),
                    )
        _, x, y, _ = d.qpos[self.rq + 3 : self.rq + 7]
        self.peak_tilt = max(self.peak_tilt, math.degrees(math.acos(np.clip(1 - 2 * (x * x + y * y), -1, 1))))
        self.low_height = min(self.low_height, d.qpos[self.rq + 2])
        return state


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


class PlantClock:
    """Advance queued coroutines only as simulated body time advances."""

    def __init__(self, clock):
        self.clock = clock
        self.origin = clock.now
        self.sleepers = []
        clock.sleep = self.sleep

    async def sleep(self, seconds, result=None):
        if seconds > 0:
            future = asyncio.get_running_loop().create_future()
            self.sleepers.append((self.clock.now + seconds, future))
            await future
        else:
            await asyncio.sleep(0)
        return result

    async def advance(self, seconds):
        self.clock.now = self.origin + seconds
        for deadline, fut in self.sleepers:
            if deadline <= self.clock.now and not fut.done():
                fut.set_result(None)
        self.sleepers = [(d, f) for d, f in self.sleepers if not f.done()]
        # Service callbacks and background queue work execute without advancing
        # physics or adding wall-clock latency to the modeled controller.
        for _ in range(3):
            await asyncio.sleep(0)


def relay_native_status(rig, plant, output):
    if plant.controller != "trick_leg_kick" or not output["inference"]:
        return
    phase = "holding" if output["holding"] else "playing"
    payload = (
        f"{output['clip']} {phase} frame {output['frame']} queued {output['queued']} "
        f"execution {output['execution_id']} request {output['clip_request_id']}"
    )
    rig.robot.send(f"/{plant.controller}/clip_status", SimpleNamespace(data=payload))


async def run_case(plant, side, forward_m, *, duration_s=45):
    # Install the maintained no-ROS service transport before importing Robot.
    # Its synthetic clip encoder is disabled: all frames below come from RTNeural.
    sys.path.insert(0, str(ROOT / "gemini_pupper/robot"))
    sys.path.insert(0, str(ROOT / "gemini_pupper/lab"))
    import test_tools as harness
    from fc_walk_kick import CombinedRuntime
    from tricks import load_tricks

    catalog = load_tricks()
    rig = harness.Rig(tricks=lambda: {"leg_kick": catalog["leg_kick"]})
    try:
        if not await rig.settle():
            raise RuntimeError("queue did not initialize")
        rig.robot.emit_interactive_status = rig.robot.emit_finite_status = lambda: None
        return await drive(plant, side, rig, CombinedRuntime, forward_m, duration_s=duration_s)
    finally:
        rig.close()
        await asyncio.sleep(0)


async def drive(plant, side, rig, runtime_type, forward_m, *, duration_s=45):
    clock = PlantClock(rig.clock)
    field = Field(ball_radius=0.11, goal_width=0.6096)
    ingress = instrument_camera(field)

    def mono_ns():
        return round(rig.ts.now() * 1e9)

    bridge = BridgeCore(
        ROBOT,
        ingress.producer_id,
        "virtual-pi",
        {ROBOT: field, "pupper-2": replace(field, attack_sign=-1)},
        clock=rig.ts.now,
        command_contract=DEADLINE_CONTRACT,
        monotonic_ns=mono_ns,
        command_clock_id=ingress.clock_name,
        kick_side=side,
        paw_forward_m=forward_m,
    )
    guard = guard_api.DeadlineBodyCommandGuard(
        clock_id=ingress.clock_name, started_mono_ns=mono_ns(), arena_ball_contact=True
    )
    executor = bridge.executor

    def send(command):
        payload, positive, problem = bridge.deadline.encode(command)
        if payload is None or (problem and any((command.vx, command.vy, command.wz))):
            raise RuntimeError(problem)
        accepted = guard.receive_command(payload, mono_ns=mono_ns(), ros_ns=mono_ns())
        if positive and (accepted.envelope is None or not accepted.envelope.allowed):
            raise RuntimeError(accepted.status["reason"])
        envelope = accepted.envelope
        if (
            plant.controller == "gemini_walk"
            and plant.bound_zero is None
            and envelope
            and envelope.allowed
            and envelope.is_zero
            and envelope.issued_mono_ns > round((clock.origin + plant.activated) * 1e9)
        ):
            plant.bound_zero = (envelope.source_instance, plant.now)
        allowed = accepted.command
        plant.send(
            Command(
                allowed.vx,
                allowed.vy,
                allowed.wz,
                accepted.envelope.expires_mono_ns / 1e9 if accepted.envelope else 0.0,
                ROBOT,
            )
        )

    rig.ts.follow_available = False

    def path_problem(policy_may_own):
        active = [name for name in rig.ts.leg_controllers if rig.robot.controllers.get(name) == "active"]
        owners = ["gemini_walk", "trick_leg_kick"] if policy_may_own else ["gemini_walk"]
        return None if len(active) == 1 and active[0] in owners else "unexpected controller owner"

    def restored_walker_ready():
        return (
            path_problem(False) is None
            and plant.controller == "gemini_walk"
            and plant.bound_zero is not None
            and plant.bound_zero[0] == bridge.deadline.source_instance
        )

    runtime = runtime_type(
        rig.ts,
        executor,
        ROBOT,
        velocity_sink=send,
        path_problem=path_problem,
        walker_ready=restored_walker_ready,
    )
    rig.robot.emit_stop_status()
    await runtime.open()
    if not bridge.operator({"command": "arm", "epoch": EPOCH})["accepted"]:
        raise RuntimeError("bridge refused field arm")
    plant.activate()
    frames = deque(maxlen=4)
    start = staged = restored = endpoint = None
    born = plant.ball()
    pre_kick_ball_motion = max_ball_speed = peak_forward_speed = 0.0
    events, switches = [], []
    phase = failure = None
    request_count = walk_inputs_after = kicks_driven_by_walker = 0
    goal_crossed_at = goal_crossing_y = None
    prior_ball = born
    goal_plane = field.length / 2 + field.ball_radius
    field_hz = round(1 / TICK_S)

    for n in range(round(duration_s * HZ)):
        await clock.advance(plant.now)
        now = rig.ts.now()
        if n % 10 == 0:
            rig.robot.emit_stop_status()
            frames.append(Observation(now, plant.pose(), plant.ball()))
        if n % 20 == 0:
            frame = dict(
                schema=guard_api.SCHEMA,
                producer_instance="sim-front",
                detector_identity=guard_api.ARENA_DETECTOR_IDENTITY,
                sequence=n // 20 + 1,
                captured_at_ros_ns=mono_ns(),
                valid=True,
                healthy=True,
                format="normalized_xyxy_v1",
                image_width=700,
                image_height=572,
                horizontal_fov_deg=220,
                detections=[],
            )
            guard.receive_frame(json.dumps(frame), mono_ns=mono_ns(), ros_ns=mono_ns())
        if n == HZ:
            send(Command(expires_at=now + 0.15, robot_id=ROBOT))
        if n >= round(4.2 * HZ):
            if start is None:
                start = plant.pose()
            if n % 100 == 0 or not events:
                sync_clocks(ingress, now)
                action = (
                    dict(kind="move", intent_id="recovered-walk", target=asdict(endpoint))
                    if endpoint
                    else dict(kind="attack", intent_id="one-kick")
                )
                result = bridge.ingest(
                    dict(
                        event="intent",
                        epoch=EPOCH,
                        clock=ingress.clock_name,
                        robot_id=ROBOT,
                        producer_id=ingress.producer_id,
                        seq=n,
                        issued_at=now,
                        valid_until=now + 0.4,
                        **action,
                    )
                )
                if not result["accepted"]:
                    raise ValueError(result["reason"])
            if n % 20 == 0 or not events:
                packet = camera_datagram(ingress, frames[0], field, n, now)
                normalized = ingress.normalize(json.loads(packet), now)
                result = bridge.ingest(normalized)
                if not result["accepted"]:
                    failure = result["reason"]
                    break
        if n >= round(4.2 * HZ) and (n * field_hz // HZ != (n - 1) * field_hz // HZ):
            step = runtime.tick(now)
            if step.decision.phase != phase:
                phase = step.decision.phase
                events.append([round(plant.now, 3), phase, step.decision.reason])
            if step.request:
                request_count += 1
                staged = plant.pose()
            if phase == "stopped":
                failure = step.decision.reason
                break
            if phase == "kick_complete" and restored is None:
                restored = plant.pose()
                endpoint = Pose(restored.x - 0.24, restored.y + 0.07, restored.yaw)
            if phase == "target_reached" and restored:
                break

        if not staged:
            pre_kick_ball_motion = max(pre_kick_ball_motion, distance(born, plant.ball()))
        (active,) = [name for name in rig.ts.leg_controllers if rig.robot.controllers.get(name) == "active"]
        if active != plant.controller:
            switches.append([round(plant.now, 4), active])
            if active == "trick_leg_kick":
                clip = rig.robot.start_clips[active][-1]
                assert clip == f"kick_{side}"
                plant.activate(side, rig.robot.active_execution_tokens[active])
            elif active == "gemini_walk":
                plant.activate()
            else:
                raise RuntimeError(f"unmodeled controller {active}")
        command = plant.command
        velocity = [command.vx, command.vy, command.wz] if command.expires_at > now else [0.0, 0.0, 0.0]
        if active != "gemini_walk" and any(velocity):
            kicks_driven_by_walker += 1
        if restored and any(velocity):
            walk_inputs_after += 1
        output = plant.advance(velocity)
        relay_native_status(rig, plant, output)
        ball = plant.ball()
        if staged:
            max_ball_speed = max(max_ball_speed, math.hypot(ball.vx, ball.vy))
            peak_forward_speed = max(peak_forward_speed, ball.vx)
        if goal_crossed_at is None and prior_ball.x < goal_plane <= ball.x:
            fraction = (goal_plane - prior_ball.x) / (ball.x - prior_ball.x)
            crossing_y = prior_ball.y + fraction * (ball.y - prior_ball.y)
            if abs(crossing_y) + field.ball_radius <= field.goal_width / 2:
                goal_crossed_at = round(plant.now - (1 - fraction) / HZ, 4)
                goal_crossing_y = round(crossing_y, 4)
        prior_ball = ball
        if plant.first_robot_ball and plant.first_robot_ball["controller"] == "gemini_walk":
            failure = "walker touched ball before kick"
            break
        if plant.peak_tilt > 45 or plant.low_height < 0.06 or numerical_failures(plant.data):
            failure = "unstable body, fall threshold or numerical warning"
            break

    if phase != "target_reached" and not failure:
        failure = f"{duration_s:g} s simulated budget elapsed"
    end = plant.pose()
    travel = distance(start, staged) if start and staged else 0.0
    after = distance(restored, end) if restored else 0.0
    final = rig.ts.last_outcome or {}
    evidence = final.get("operator_kick")
    confirmed = bool(
        final.get("success")
        and evidence
        and evidence.get("confirmed_clip") == f"kick_{side}"
        and evidence.get("restored_controller") == "gemini_walk"
    )
    first = plant.first_robot_ball or {}
    paw_contact = (
        first.get("geom", "").startswith(f"leg_front_{side[0]}_3")
        and first.get("controller") == "trick_leg_kick"
        and first.get("clip") == f"kick_{side}"
    )
    # Cleanup can wait on the ROS service clock after revocation. Continue
    # advancing virtual time so a failed trial cannot hang the whole sweep.
    close = asyncio.create_task(runtime.close())
    for tick in range(1, 251):
        await clock.advance(plant.now + tick * 0.02)
        if close.done():
            break
    if not close.done():
        close.cancel()
        raise TimeoutError("combined runtime did not close within 5 simulated seconds")
    await close
    if not failure and peak_forward_speed <= 0.2:
        failure = "forward ball speed at most 0.2 m/s"
    return dict(
        side=side,
        forward_offset_m=PAW_FORWARD_M if forward_m is None else forward_m,
        docking_assist_enabled=forward_m is not None,
        motion_limits=asdict(bridge.limits),
        first_robot_ball=plant.first_robot_ball,
        passed=bool(
            not failure
            and phase == "target_reached"
            and travel > 0.20
            and after >= 0.15
            and walk_inputs_after
            and request_count == 1
            and confirmed
            and paw_contact
            and peak_forward_speed > 0.2
            and not kicks_driven_by_walker
            and pre_kick_ball_motion < 0.01
        ),
        failure=failure,
        terminal_phase=phase,
        events=events,
        policy_inferences=dict(plant.inferences),
        pre_kick_walk_m=round(travel, 4),
        post_kick_walk_m=round(after, 4),
        post_kick_nonzero_walker_ticks=walk_inputs_after,
        nonzero_walk_ticks_with_trick_active=kicks_driven_by_walker,
        kick_requests=request_count,
        controller_switches=switches,
        restored_source_zero_s=round(plant.bound_zero[1], 4) if restored and plant.bound_zero else None,
        operator_evidence=evidence,
        queue_outcome=final.get("terminal_state"),
        queue_message=final.get("message"),
        recovery_target=asdict(endpoint) if endpoint else None,
        recovery_position_error_m=round(distance(endpoint, end), 4) if endpoint else None,
        recovery_heading_error_deg=round(
            abs(math.degrees(math.remainder(end.yaw - endpoint.yaw, math.tau))), 2
        )
        if endpoint
        else None,
        end_pose=asdict(end),
        ball=asdict(plant.ball()),
        ball_motion_before_request_m=round(pre_kick_ball_motion, 4),
        max_ball_speed_after_request_m_s=round(max_ball_speed, 3),
        peak_forward_ball_m_s=round(peak_forward_speed, 3),
        goal_crossed_s=goal_crossed_at,
        goal_crossing_y_m=goal_crossing_y,
        first_ball_geom_contacts=plant.contact,
        max_tilt_deg=round(plant.peak_tilt, 2),
        min_root_height_m=round(plant.low_height, 3),
    )


def selection():
    from foundation.hardware.fc_walk_kick_release import selected_paths

    return [{"path": path, "sha256": sha(ROOT / path)} for path in selected_paths(ROOT)]


def selected_profile(folder, staged, entries):
    from foundation.hardware import fc_walk_kick_release as release

    selection_digest = sha256(json.dumps(entries, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    if staged:
        stage_receipt = release.verify(staged)
        manifest = json.loads((staged / release.MANIFEST).read_text())
        if manifest["final_selected_files"] != entries:
            raise ValueError("staged field release differs from the sources under test")
    else:
        stage_receipt = None
    env = dict(
        os.environ,
        **release.ENV,
        PUPPER_SESSION_MANIFEST=str(folder / "runtime-manifest.json"),
        PUPPER_GAIT_DEPLOY_DIR=str(ROOT / "pupper_gait_deploy"),
        PUPPER_GEMINI_DIR=str(ROOT / "gemini_pupper"),
        FOLLOW_USE_COLOR="0",
        FOLLOW_ALWAYS_DETECT="0",
        FOLLOW_AVOID_OBSTACLES="0",
    )
    profile = json.loads(
        subprocess.check_output(
            [sys.executable, str(ROOT / "foundation/hardware/fc_walk_kick_release.py"), "_profile"],
            env=env,
            text=True,
        )
    )
    config = yaml.safe_load((folder / "persistent_controllers.yaml").read_text())
    params = {name: config[name]["ros__parameters"] for name in profile["controllers"]}
    for name, policy in (("gemini_walk", WALK), ("trick_leg_kick", KICK)):
        p = params[name]
        if (
            Path(p["model_path"]) != policy
            or p["repeat_action"] != 10
            or p["gain_multiplier"] != 1
            or set(p["init_kps"]) != {7.5}
            or set(p["init_kds"]) != {0.25}
            or set(p["kp_scale"]) != {1.0}
        ):
            raise ValueError(f"controller settings incompatible with host transcription: {name}")
    receipt = {
        "controllers": profile["controllers"],
        "mode": profile["motion_mode"],
        "walker_contract": profile["walker_command_contract"],
        "init_and_fade_s": {k: [v["init_duration"], v["fade_in_duration"]] for k, v in params.items()},
        "selected_files_sha256": selection_digest,
        "selected_file_count": len(entries),
        "staged_manifest_sha256": stage_receipt["manifest_sha256"] if stage_receipt else None,
        "staged_source_commit": stage_receipt["source_commit"] if stage_receipt else None,
    }
    return params, receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--staged-release", type=Path)
    args = parser.parse_args()
    os.environ.update(
        PUPPER_SESSION_MOTION_MODE="fc_walk_kick",
        PUPPER_SESSION_SKILLS="leg_kick",
        PUPPER_FOLLOWER_MODE="disabled",
    )
    model, contract = make_model("full_hulls", configure_scene=add_soccer_ball)
    cases = [("nominal", side, forward, 0.0, 0.0) for side, forward in COMBINED_PAW_FORWARD_M.items()]
    cases += [
        ("offset_start", side, forward, 0.025, 0.04) for side, forward in COMBINED_PAW_FORWARD_M.items()
    ]
    cases += [
        ("original_right", "right", None, 0.0, 0.0),
        ("unassisted_offset", "left", None, 0.025, 0.04),
        ("longer_left", "left", 0.270, 0.0, 0.0),
        ("longer_right", "right", 0.280, 0.0, 0.0),
    ]
    results = []
    release_inputs = selection()
    with tempfile.TemporaryDirectory() as folder:
        params, release = selected_profile(Path(folder), args.staged_release, release_inputs)
        binary, _ = build_controller(Path(folder))
        for scenario, side, forward, y, yaw in cases:
            plant = PolicyPlant(model, binary, side, params, y=y, heading=yaw)
            try:
                outcome = asyncio.run(run_case(plant, side, forward))
                outcome.update(scenario=scenario, initial_y_m=y, initial_yaw_rad=yaw)
                results.append(outcome)
                print(
                    scenario,
                    side,
                    outcome["passed"],
                    outcome["terminal_phase"],
                    outcome["peak_forward_ball_m_s"],
                    flush=True,
                )
            finally:
                plant.close()
    if release_inputs != selection():
        raise RuntimeError("release files changed while the simulation was running")
    report = dict(
        scope="One unreset MuJoCo body and rigid ball. Learned RTNeural locomotion and kicking, real field ingress, "
        "field bridge, 50 Hz combined runtime, operator queue, deadline encoder, body-command guard and correlated receipts. Synthetic measured tracks at 26 Hz, "
        "delayed ~58 ms. Front camera supplies synthetic clear frames. Native lifecycle transcribed; ROS switch/Stop transport, graph and live camera localization modeled. "
        "Contact evidence comes from MuJoCo; the clip receipt alone does not establish ball contact.",
        versions={"mujoco": mujoco.__version__, "numpy": np.__version__},
        contact_model=contract,
        ball_assumptions={"radius_m": 0.11, "mass_kg": 0.43, "friction": 0.6},
        rate_hz=HZ,
        actuator_delay_s=0.016,
        release=release,
        reproduction={
            "python": sys.executable,
            "from": "repository root",
            "environment": {"PYTHONPATH": "pupper-mjlab/src"},
            "command": "python foundation/scripts/verify_fc_policy_cycle.py --staged-release RELEASE_DIR --output REPORT.json",
        },
        policies={str(p.relative_to(ROOT)): sha(p) for p in (WALK, KICK)},
        diagnosis="The nominal cases exercise each selected paw offset. Offset starts and alternate stand-offs "
        "are sensitivity checks. A passing cycle requires first contact by the selected front paw, "
        "forward ball speed above 0.2 m/s, and actual walking before and after the matched kick.",
        sources={
            str(p.relative_to(ROOT)): sha(p)
            for p in (
                DRIVER,
                ROOT / "foundation/cpp/handoff_controller.cpp",
                ROOT / "pupper_gait_deploy/src/trick_motion.hpp",
                ROOT / "gemini_pupper/lab/operator_kick.py",
                ROOT / "final_project/pupper_fc/integration.py",
                ROOT / "final_project/pupper_fc/paw_approach.py",
                ROOT / "final_project/pupper_fc/controller.py",
                ROOT / "gemini_pupper/robot/fc_walk_kick.py",
                ROOT / "final_project/behaviors/combined_fc_runner.py",
                ROOT / "final_project/behaviors/overhead_ingress.py",
                ROOT / "final_project/pupper_fc/kick_rehearsal.py",
                ROOT / "foundation/scripts/check_spider_handoff.py",
                Path(__file__).resolve(),
            )
        },
        selected_summary={
            "cycles_completed": sum(
                r["passed"] for r in results if r["scenario"] in ("nominal", "offset_start")
            ),
            "full_ball_goal_crossings": sum(
                r["goal_crossed_s"] is not None
                for r in results
                if r["scenario"] in ("nominal", "offset_start")
            ),
            "cases": 4,
            "qualification": "Two deterministic starts per foot, ideal delayed pose and static ball. Endpoint errors are relative "
            "to the supplied return target in model coordinates. Camera errors, physical repeatability and actual ball performance remain unmeasured.",
        },
        cases=results,
    )
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return int(not all(r["passed"] for r in results if r["scenario"] in ("nominal", "offset_start")))


if __name__ == "__main__":
    raise SystemExit(main())
