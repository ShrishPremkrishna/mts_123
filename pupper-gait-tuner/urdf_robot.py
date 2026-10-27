"""Optional URDF visualization of the Pupper in the Viser scene.

Loads the real pupper_v3 URDF (with meshes) and poses it live from the same
12-vector joint cache that drives the real robot, so the on-screen robot mirrors
the hardware. Degrades gracefully: if yourdfpy / the URDF / meshes are missing,
load_pupper_urdf() returns None and the app falls back to foot-path-only viz.
"""

import os
import numpy as np

# Resolved package share dir for pupper_v3_description on the robot image.
_SHARE = "/home/pi/pupperv3-monorepo/ros2_ws/install/pupper_v3_description/share"
_URDF = os.path.join(
    _SHARE, "pupper_v3_description/description/urdf/pupper_v3.edited.fixed.urdf")


def _package_handler(fname):
    if fname.startswith("package://"):
        return os.path.join(_SHARE, fname[len("package://"):])
    return fname


class PupperUrdf:
    """Wraps viser.extras.ViserUrdf with name-keyed joint updates."""

    def __init__(self, server, urdf_model, root="/robot"):
        from viser.extras import ViserUrdf
        self.viser_urdf = ViserUrdf(server, urdf_model, root_node_name=root)
        self.actuated = list(urdf_model.actuated_joint_names)
        self._cfg = np.zeros(len(self.actuated))
        # index of each controller joint within the URDF actuated config vector
        self._idx = {name: (self.actuated.index(name) if name in self.actuated else None)
                     for name in JOINT_ORDER}
        self.update_from_vector(np.zeros(12))

    def update_from_vector(self, joints12):
        """joints12 in controller JOINT_ORDER -> pose the URDF."""
        for name, val in zip(JOINT_ORDER, joints12):
            i = self._idx[name]
            if i is not None:
                self._cfg[i] = val
        self.viser_urdf.update_cfg(self._cfg)


JOINT_ORDER = [
    'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
    'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3',
    'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3',
    'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3',
]


def load_pupper_urdf(server, root="/robot"):
    """Return a PupperUrdf, or None if anything required is unavailable."""
    if not os.path.exists(_URDF):
        print(f"[urdf] not found: {_URDF} — using foot-path viz only")
        return None
    try:
        import yourdfpy
        model = yourdfpy.URDF.load(_URDF, filename_handler=_package_handler)
        robot = PupperUrdf(server, model, root=root)
        print(f"[urdf] loaded Pupper mesh model ({len(robot.actuated)} joints)")
        return robot
    except Exception as e:  # noqa: BLE001 — viz is optional, never block the app
        print(f"[urdf] disabled ({type(e).__name__}: {e}) — using foot-path viz only")
        return None
