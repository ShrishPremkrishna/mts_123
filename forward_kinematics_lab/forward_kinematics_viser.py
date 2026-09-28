#!/usr/bin/env python3
"""Show the Pupper and the end-effector markers in a viser web viewer.

This is the browser-based alternative to RViz2 for the forward kinematics lab. It
runs on the robot and serves a 3D view that anyone on the same network can open:

    http://<pupper-ip>:8080

The view contains:
  * the Pupper URDF, posed from /joint_states (so it moves as you move the legs)
  * one sphere per leg at the end-effector position your forward_kinematics.py
    publishes on /marker (same message RViz2 shows, so both viewers agree)
  * a panel listing the joint angles and the end-effector positions

It comes up automatically with the lab stack:

    ros2 launch forward_kinematics.launch.py        # add viser:=false to skip it

or can be started on its own:

    python3 forward_kinematics_viser.py [--port 8080]

Viewing over SSH (e.g. if the robot is not reachable directly)? Forward the port
from your laptop, then browse to http://localhost:8080 there:

    ssh -N -L 8080:localhost:8080 pi@<pupper-ip>

The script prints the exact command (with this Pi's IP filled in) at startup.
"""

import argparse
import os
import socket
import subprocess
import tempfile
import threading
import time

import numpy as np
import rclpy
import viser
import yourdfpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from viser.extras import ViserUrdf

URDF_PACKAGE = "pupper_v3_description"
URDF_XACRO = os.path.join("description", "pupper_v3.urdf.xacro")

# Legs in the order the joints appear in forward_kinematics.yaml.
LEGS = ["front_r", "front_l", "back_r", "back_l"]
JOINT_NAMES = [f"leg_{leg}_{i}" for leg in LEGS for i in (1, 2, 3)]


def load_pupper_urdf():
    """Expand the Pupper xacro and load it with yourdfpy (meshes included)."""
    from ament_index_python.packages import get_package_share_directory

    share = get_package_share_directory(URDF_PACKAGE)
    xacro_path = os.path.join(share, URDF_XACRO)
    urdf_xml = subprocess.check_output(["xacro", xacro_path], text=True)

    prefix = f"package://{URDF_PACKAGE}/"

    def resolve(fname):
        # yourdfpy does not understand package:// URIs; point them at the share dir.
        if fname.startswith(prefix):
            return os.path.join(share, fname[len(prefix) :])
        return fname

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(urdf_xml)
        tmp_path = f.name
    try:
        return yourdfpy.URDF.load(tmp_path, filename_handler=resolve, load_meshes=True, build_scene_graph=True)
    finally:
        os.unlink(tmp_path)


def local_ips():
    """Every non-loopback IPv4 address this Pi is reachable at."""
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True, timeout=5)
    except Exception:
        return []
    return [ip for ip in out.split() if ":" not in ip and not ip.startswith("127.")]


def print_access_banner(port):
    ips = local_ips()
    user = subprocess.getoutput("whoami").strip() or "pi"
    host = socket.gethostname()
    print("\n" + "=" * 64)
    print(f"  Forward kinematics viewer is live on port {port}")
    print("=" * 64)
    if ips:
        print("\n  Same network as the Pupper? Open one of these in a browser:")
        for ip in ips:
            print(f"      http://{ip}:{port}")
    print("\n  Viewing over SSH? Run this on your LAPTOP (new terminal),")
    print("  then open http://localhost:%d there:" % port)
    target = ips[0] if ips else f"{host}.local"
    print(f"\n      ssh -N -L {port}:localhost:{port} {user}@{target}")
    print("\n" + "=" * 64 + "\n", flush=True)


class ForwardKinematicsViewer(Node):
    """ROS side: keeps the latest joint angles and markers; the viser side reads them."""

    def __init__(self):
        super().__init__("forward_kinematics_viser")
        self.lock = threading.Lock()
        self.joint_angles = {}  # joint name -> angle
        self.markers = {}  # (ns, id) -> Marker
        self.last_joint_state_time = None
        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.create_subscription(Marker, "marker", self.marker_callback, 10)

    def joint_state_callback(self, msg):
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                self.joint_angles[name] = position
            self.last_joint_state_time = time.time()

    def marker_callback(self, msg):
        key = (msg.ns, msg.id)
        with self.lock:
            if msg.action in (Marker.DELETE, Marker.DELETEALL):
                if msg.action == Marker.DELETEALL:
                    self.markers.clear()
                else:
                    self.markers.pop(key, None)
            else:
                self.markers[key] = msg

    def snapshot(self):
        with self.lock:
            return dict(self.joint_angles), dict(self.markers), self.last_joint_state_time


def spin_in_background(node):
    def _spin():
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return thread


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--rate", type=float, default=30.0, help="viewer update rate in Hz")
    args = parser.parse_args()

    rclpy.init()
    node = ForwardKinematicsViewer()
    spin_in_background(node)

    server = viser.ViserServer(port=args.port, label="Pupper Forward Kinematics")
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/grid", width=1.0, height=1.0, position=(0.0, 0.0, -0.2))
    server.scene.add_frame("/base_link", axes_length=0.08, axes_radius=0.003)

    @server.on_client_connect
    def _(client):
        client.camera.position = (0.45, -0.45, 0.25)
        client.camera.look_at = (0.0, 0.0, -0.05)

    print("Loading Pupper URDF...", flush=True)
    urdf = load_pupper_urdf()
    robot = ViserUrdf(server, urdf, root_node_name="/robot")
    robot_joint_names = list(robot.get_actuated_joint_names())

    with server.gui.add_folder("Status"):
        gui_joint_state = server.gui.add_text("Joint states", initial_value="waiting...", disabled=True)
        gui_show_robot = server.gui.add_checkbox("Show robot", True)
        gui_show_markers = server.gui.add_checkbox("Show end effectors", True)
        gui_show_labels = server.gui.add_checkbox("Show labels", True)
    with server.gui.add_folder("Joint angles [rad]"):
        gui_joints = {leg: server.gui.add_text(leg, initial_value="--", disabled=True) for leg in LEGS}
    with server.gui.add_folder("End-effector positions [m]"):
        gui_ee = {leg: server.gui.add_text(leg, initial_value="(no marker yet)", disabled=True) for leg in LEGS}

    @gui_show_robot.on_update
    def _(_):
        robot.show_visual = gui_show_robot.value

    sphere_handles = {}  # (ns, id) -> (sphere, label)

    print_access_banner(args.port)

    period = 1.0 / args.rate
    try:
        while rclpy.ok():
            loop_start = time.time()
            joint_angles, markers, last_js = node.snapshot()

            # --- robot pose ---
            if joint_angles:
                cfg = np.array([joint_angles.get(name, 0.0) for name in robot_joint_names])
                robot.update_cfg(cfg)
            if last_js is None:
                gui_joint_state.value = "waiting for /joint_states..."
            elif time.time() - last_js > 1.0:
                gui_joint_state.value = "STALE (no /joint_states for %.0fs)" % (time.time() - last_js)
            else:
                gui_joint_state.value = "ok"
            for leg in LEGS:
                names = [f"leg_{leg}_{i}" for i in (1, 2, 3)]
                if all(n in joint_angles for n in names):
                    gui_joints[leg].value = "  ".join(f"{joint_angles[n]:+.3f}" for n in names)

            # --- end-effector markers ---
            for key, marker in markers.items():
                if marker.type != Marker.SPHERE:
                    continue
                name = f"/end_effectors/{marker.ns or 'marker'}_{marker.id}"
                pos = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
                color = (
                    int(255 * marker.color.r),
                    int(255 * marker.color.g),
                    int(255 * marker.color.b),
                )
                radius = max(marker.scale.x, 1e-3) / 2.0
                label_text = marker.ns or f"marker {marker.id}"
                if key not in sphere_handles:
                    sphere = server.scene.add_icosphere(name, radius=radius, color=color, position=pos)
                    label = server.scene.add_label(name + "/label", text=label_text, position=(0.0, 0.0, radius * 1.5))
                    sphere_handles[key] = (sphere, label)
                else:
                    sphere, label = sphere_handles[key]
                    sphere.position = pos
                    sphere.color = color
                sphere.visible = gui_show_markers.value
                label.visible = gui_show_markers.value and gui_show_labels.value
                if marker.ns in gui_ee:
                    gui_ee[marker.ns].value = "x=%+.3f  y=%+.3f  z=%+.3f" % pos

            # markers that were deleted
            for key in list(sphere_handles):
                if key not in markers:
                    sphere, label = sphere_handles.pop(key)
                    label.remove()
                    sphere.remove()

            time.sleep(max(0.0, period - (time.time() - loop_start)))
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
