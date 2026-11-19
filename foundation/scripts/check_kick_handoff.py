#!/usr/bin/env python3
"""Leg kick entry and exit timing in simulation: walker stance -> leg kick started at
the kick clip (start_clip) -> walker init phase.

No robot and no ROS. The trick controller is the host build of check_spider_handoff.py
(production RTNeural, clip player and fade arithmetic; the lifecycle is a transcription).
The walker is not simulated as a policy: before the kick the legs hold the walker's
default pose with its policy gains, and after the kick the walker's init phase (a linear
ease to its default pose with the init gains) takes the legs, which is what the native
controller does for the first init_duration seconds of an activation.

For each case the script reports when the ball first moves (from the activation), the
ball speed 0.1 s later, the largest body tilt, and whether the robot stands 2 s after
the walker's init phase takes the legs.

    PYTHONPATH=pupper-mjlab/src MUJOCO_GL=egl python foundation/scripts/check_kick_handoff.py \\
        --output /tmp/kick-handoff [--side left]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
from collections import deque
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
POLICY = ROOT / "gemini_pupper/tricks/leg_kick/policy.json"
BALL = dict(radius=0.075, mass=0.10, friction=(0.6, 0.005, 0.0001), solref=(0.01, 0.5))  # leg_kick_ball.py
BALL_XY = {"right": (0.24, -0.046), "left": (0.24, 0.040)}  # sim ball centre from base_link (radius 0.075 m)
WALK_KP, WALK_KD, INIT_KP, INIT_KD = 5.0, 0.25, 7.5, 0.25  # pupper_gait_deploy/config.yaml
RATE = 520  # controller_manager update rate; inference every 10 updates
DELAY_S = 0.016  # actuator delay, as in check_spider_handoff.py


def spider_module():
    spec = importlib.util.spec_from_file_location("check_spider_handoff", Path(__file__).with_name("check_spider_handoff.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_model():
    """check_spider_handoff.make_model('full_hulls') plus a free ball."""
    from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import PUPPER_MUJOCO_OPTIONS
    from mjlab.entity import Entity
    from mjlab.sim import MujocoCfg
    from mjlab.tasks.tracking.config.pupper.contact_contract import enable_tracking_ground_hulls
    from mjlab.tasks.tracking.config.pupper.env_cfgs import pupper_tracking_env_cfg

    cfg = pupper_tracking_env_cfg(play=True)
    enable_tracking_ground_hulls(cfg)
    spec = Entity(cfg.scene.entities["robot"]).spec
    spec.worldbody.add_geom(name="terrain", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1],
                            contype=1, conaffinity=1, friction=[1, 0.005, 0.0001])
    ball = spec.worldbody.add_body(name="ball", pos=[5, 0, BALL["radius"]])
    ball.add_freejoint(name="ball_joint")
    ball.add_geom(name="ball_geom", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=(BALL["radius"], 0, 0),
                  mass=BALL["mass"], condim=3, friction=BALL["friction"], solref=BALL["solref"],
                  priority=2, contype=1, conaffinity=1)
    model = spec.compile()
    MujocoCfg(**PUPPER_MUJOCO_OPTIONS).apply(model)
    model.opt.timestep = 1 / RATE
    return model


class Sim:
    def __init__(self, model, binary, policy_path):
        from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES

        self.model, self.data = model, mujoco.MjData(model)
        joints = [model.joint(name).id for name in JOINT_NAMES]
        self.qadr, self.vadr = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
        self.controls = [int(np.flatnonzero(model.actuator_trnid[:, 0] == j)[0]) for j in joints]
        free = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        ball = model.joint("ball_joint").id
        root = next(j for j in free if j != ball)
        self.root_q, self.root_v = int(model.jnt_qposadr[root]), int(model.jnt_dofadr[root])
        self.ball_q, self.ball_v = int(model.jnt_qposadr[ball]), int(model.jnt_dofadr[ball])
        self.default = np.asarray(json.loads(policy_path.read_text())["default_joint_pos"], dtype=float)
        self.proc = subprocess.Popen([str(binary), str(policy_path), str(policy_path)], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, text=True, bufsize=1)
        self.reset()

    def reset(self):
        d = self.data
        mujoco.mj_resetData(self.model, d)
        d.qpos[self.root_q:self.root_q + 7] = [0, 0, 0.18, 1, 0, 0, 0]
        d.qpos[self.qadr] = self.default
        mujoco.mj_forward(self.model, d)
        self.t = 0.0
        self.delayed = deque([(-1.0, self.default.copy()), (0.0, self.default.copy())])
        self.max_tilt = 0.0

    def rpc(self, request):
        self.proc.stdin.write(json.dumps(request) + "\n")
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError("the host controller exited")
        return json.loads(line)

    def tilt(self):
        _, x, y, _ = self.data.qpos[self.root_q + 3:self.root_q + 7]
        return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1))))

    def step(self, target, kp, kd):
        """One 520 Hz update with the delayed position target and the PD gains."""
        d, m = self.data, self.model
        self.delayed.append((self.t, np.asarray(target, dtype=float).copy()))
        cutoff = self.t - DELAY_S
        while len(self.delayed) > 2 and self.delayed[1][0] <= cutoff:
            self.delayed.popleft()
        (t0, v0), (t1, v1) = self.delayed[0], self.delayed[1]
        a = np.clip((cutoff - t0) / (t1 - t0), 0, 1)
        d.ctrl[self.controls] = (1 - a) * v0 + a * v1
        m.actuator_gainprm[self.controls, 0] = kp
        m.actuator_biasprm[self.controls, 1] = -kp
        m.actuator_biasprm[self.controls, 2] = -kd
        mujoco.mj_step(m, d)
        self.t += 1 / RATE
        self.max_tilt = max(self.max_tilt, self.tilt())

    def hold(self, seconds, pose, kp, kd):
        for _ in range(round(seconds * RATE)):
            self.step(pose, kp, kd)

    def ease(self, seconds, hold_after, kp, kd):
        """The walker's init phase: a linear ease from the measured pose to its default."""
        start = self.data.qpos[self.qadr].copy()
        for i in range(round((seconds + hold_after) * RATE)):
            a = min(i / (seconds * RATE), 1.0)
            self.step((1 - a) * start + a * self.default, kp, kd)

    def place_ball(self, side):
        d = self.data
        pos, quat = d.qpos[self.root_q:self.root_q + 3], d.qpos[self.root_q + 3:self.root_q + 7]
        yaw = np.arctan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
        bx, by = BALL_XY[side]
        d.qpos[self.ball_q:self.ball_q + 7] = [pos[0] + np.cos(yaw) * bx - np.sin(yaw) * by,
                                              pos[1] + np.sin(yaw) * bx + np.cos(yaw) * by, BALL["radius"], 1, 0, 0, 0]
        d.qvel[self.ball_v:self.ball_v + 6] = 0
        mujoco.mj_forward(self.model, d)

    def kick(self, side, init_s, fade_s, leave_s, walker_init_s, arrive_speed=0.0, start_clip=True, settle_s=0.0):
        """One case. leave_s: seconds into the kick clip at which the walker's init phase
        takes the legs. arrive_speed: forward speed of the body when the walk command ends."""
        self.reset()
        d = self.data
        self.hold(1.0, self.default, WALK_KP, WALK_KD)  # stand on the walker
        x_walk_end = float(d.qpos[self.root_q])
        if arrive_speed:
            # The end of a walk, coarsely: the body still moves forward when the command goes to zero.
            d.qvel[self.root_v] = arrive_speed
            self.hold(settle_s, self.default, WALK_KP, WALK_KD)  # zero speed on the walker (KICK_SETTLE_S)
        self.place_ball(side)
        x0 = float(d.qpos[self.root_q])
        self.max_tilt, t_act = 0.0, self.t
        clip = f"kick_{side}"
        self.rpc(dict(op="activate", controller="trick", joint_pos=d.qpos[self.qadr].tolist(), init_s=init_s,
                      fade_s=fade_s, start_clip=clip if start_clip else "", clip_anchor=True))
        if not start_clip:
            wait = init_s + fade_s + 1.0  # the earlier sequence: 1.0 s in stand_idle, then a clip request
        requested, contact, speed_after, clip_t0, fell = start_clip, None, None, None, False
        travel_to_contact = None
        leave_at = None
        while True:
            el = self.t - t_act
            if not requested and el >= wait:
                self.rpc(dict(op="request_clip", clip=clip))
                requested = True
            state = self.rpc(dict(op="step", elapsed_s=el, joint_pos=d.qpos[self.qadr].tolist(),
                                  joint_vel=d.qvel[self.vadr].tolist(), quaternion=d.qpos[self.root_q + 3:self.root_q + 7].tolist(),
                                  angular_velocity=d.qvel[self.root_v + 3:self.root_v + 6].tolist(), command=[0, 0, 0]))
            if clip_t0 is None and state.get("clip") == clip and state.get("stage") == "policy":
                clip_t0 = el
                leave_at = el + leave_s
            self.step(state["target"], state["kp"], state["kd"])
            ball_speed = float(np.linalg.norm(d.qvel[self.ball_v:self.ball_v + 2]))
            if contact is None and ball_speed > 0.05:
                contact = self.t - t_act
                travel_to_contact = float(d.qpos[self.root_q]) - x_walk_end
            if contact is not None and speed_after is None and self.t - t_act >= contact + 0.10:
                v = d.qvel[self.ball_v:self.ball_v + 2]
                speed_after = (ball_speed, float(np.degrees(np.arctan2(v[1], v[0]))))
            if self.tilt() > 45 or d.qpos[self.root_q + 2] < 0.06:
                fell = True
                break
            if leave_at is not None and el >= leave_at:
                break
        kick_tilt = self.max_tilt
        self.max_tilt = 0.0
        if not fell:
            self.ease(walker_init_s, 2.0, INIT_KP, INIT_KD)
            fell = self.tilt() > 20 or d.qpos[self.root_q + 2] < 0.08
        return dict(side=side, init_s=init_s, fade_s=fade_s, start_clip=start_clip, arrive_speed=arrive_speed,
                    settle_s=settle_s, leave_s=leave_s, walker_init_s=walker_init_s,
                    contact_after_activation_s=None if contact is None else round(contact, 2),
                    ball_speed_m_s=None if speed_after is None else round(speed_after[0], 2),
                    ball_direction_deg=None if speed_after is None else round(speed_after[1], 1),
                    kick_max_tilt_deg=round(kick_tilt, 1), return_max_tilt_deg=round(self.max_tilt, 1),
                    travel_walk_end_to_contact_m=None if travel_to_contact is None else round(travel_to_contact, 3),
                    base_travel_m=round(float(d.qpos[self.root_q]) - x0, 3), stands=not fell)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, required=True, help="new directory for the host build and the report")
    parser.add_argument("--side", choices=["right", "left", "both"], default="both")
    parser.add_argument("--policy", type=Path, default=POLICY, help="leg kick policy JSON (default: the installed trick)")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    binary, _ = spider_module().build_controller(output)
    sim = Sim(make_model(), binary, args.policy.resolve())
    sides = ["right", "left"] if args.side == "both" else [args.side]
    cases = []
    for side in sides:
        # (init, fade, leave, walker init, arrive speed, start_clip, settle)
        for row in [(0.5, 1.0, 3.5, 2.0, 0.0, False, 0.0),   # the earlier sequence
                    (0.5, 1.0, 2.6, 2.0, 0.0, True, 0.0),    # direct clip start, early exit, installed blend
                    (0.5, 1.0, 2.6, 2.0, 0.26, True, 0.5),   # ... arriving from a walk, the earlier 0.5 s dwell
                    (0.5, 1.0, 2.6, 2.0, 0.26, True, 0.15),  # ... and the new 0.15 s
                    (0.2, 0.3, 2.6, 2.0, 0.0, True, 0.0),    # short blend
                    (0.2, 0.3, 2.6, 2.0, 0.26, True, 0.15),
                    (0.2, 0.3, 2.6, 2.0, 0.26, True, 0.0),   # no settle at all
                    (0.1, 0.2, 2.6, 2.0, 0.26, True, 0.15),
                    (0.1, 0.1, 2.6, 2.0, 0.26, True, 0.15),
                    (0.2, 0.3, 2.1, 2.0, 0.0, True, 0.0),    # leave when the paw is down, weight not yet back
                    (0.2, 0.3, 2.6, 0.3, 0.0, True, 0.0),    # a walker with a short init phase
                    (0.2, 0.3, 2.1, 0.3, 0.0, True, 0.0)]:
            init_s, fade_s, leave_s, walker_init_s, speed, start_clip, settle_s = row
            case = sim.kick(side, init_s, fade_s, leave_s, walker_init_s, speed, start_clip, settle_s)
            cases.append(case)
            print(json.dumps(case), flush=True)
    sim.proc.stdin.close()
    (output / "report.json").write_text(json.dumps(dict(policy=str(args.policy), ball=BALL, cases=cases), indent=2) + "\n")


if __name__ == "__main__":
    main()
