"""Compare ideal drive-local damping with disabled motors on the supported Pupper model."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
MODEL = Path("pupperv3-monorepo/ros2_ws/src/pupper_v3_description/description/mujoco_xml/pupper_v3_complete.xml")


def supported_model():
    xml = ET.parse(ROOT / MODEL).getroot()
    for parent in xml.iter():
        for child in list(parent):
            if child.tag in ("freejoint", "keyframe", "mesh") or (child.tag == "geom" and child.get("mesh")):
                parent.remove(child)
    # Weld the torso above the floor. Each link retains its explicit mass, inertia,
    # and collision geometry. Removed meshes are visual-only and have zero density.
    xml.find("./worldbody/body").set("pos", "0 0 2")
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    model.opt.timestep = .001
    return model


def trial(model, damping):
    joint_ids = model.actuator_trnid[:, 0]
    velocities = model.jnt_dofadr[joint_ids]
    state = mujoco.MjData(model)
    state.qpos[model.jnt_qposadr[joint_ids]] = [.26, 0, -.52, -.26, 0, .52] * 2
    state.qvel[velocities] = [1, -2, .5] * 4
    mujoco.mj_forward(model, state)
    work, peak_torque, peak_speed = 0., 0., 0.
    for _ in range(3000):
        speed = state.qvel[velocities]
        state.ctrl[:] = -damping * speed
        power = float(np.dot(state.ctrl, speed))
        if power > 0:
            raise AssertionError("damping added joint energy")
        work += power * model.opt.timestep
        peak_torque = max(peak_torque, float(np.abs(state.ctrl).max()))
        peak_speed = max(peak_speed, float(np.abs(speed).max()))
        mujoco.mj_step(model, state)
    return {"damping_work_j": work, "peak_ideal_torque_nm": peak_torque, "peak_joint_speed_rad_s": peak_speed}


if __name__ == "__main__":
    model = supported_model()
    # Match the pinned FC URDF (all effort_max=3) and the reference SPIneV1 CAN packing.
    host_gain = (.50 - 225/4095) / (65 + 130/4095)
    decoded_gain = np.floor(host_gain * 4095/5) * 5/4095
    print(json.dumps({"mujoco": mujoco.__version__, "model": str(MODEL),
                      "model_sha256": hashlib.sha256((ROOT / MODEL).read_bytes()).hexdigest(),
                      "duration_s": 3., "host_kd": host_gain, "nominal_decoded_kd": decoded_gain,
                      "disabled": trial(model, 0.), "damping": trial(model, decoded_gain)}, indent=2))
