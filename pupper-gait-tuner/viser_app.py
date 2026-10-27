"""Interactive Pupper gait tuner — Viser front end (visualize-only).

Drag sliders, watch the four foot trajectories and live foot markers update in
real time. The gait math lives in gait.py; this file is just UI + 3D viz + an
animation loop. The ROS bridge (streaming to the real robot) plugs in via the
`on_robot_send` hook and the "Send to robot" checkbox, wired up in ros_bridge.py.

Run on the robot Pi:
    python3 viser_app.py
then open http://<pi-ip>:8080 in a browser.
"""

import ipaddress
import socket
import subprocess
import threading
import time
import numpy as np
import viser


def get_lan_ip():
    """Best-effort LAN IP of this machine (the address on the local network)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))      # no packets sent; just picks the route
        ip = s.getsockname()[0]
    except OSError:
        ip = None
    finally:
        s.close()
    return ip


def get_tailscale_ip():
    """Tailscale IPv4 (100.64.0.0/10), or None. Used when browsing over the tailnet."""
    # Preferred: ask the tailscale CLI directly.
    try:
        out = subprocess.run(["tailscale", "ip", "-4"],
                             capture_output=True, text=True, timeout=3)
        lines = [ln.strip() for ln in out.stdout.splitlines() if ln.strip()]
        if lines:
            return lines[0]
    except (FileNotFoundError, subprocess.SubprocessError):
        pass
    # Fallback: scan all local IPs for one in the Tailscale CGNAT range.
    ts_net = ipaddress.ip_network("100.64.0.0/10")
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=3)
        for tok in out.stdout.split():
            try:
                if ipaddress.ip_address(tok) in ts_net:
                    return tok
            except ValueError:
                continue
    except subprocess.SubprocessError:
        pass
    return None


def print_url_banner(port, name=None):
    ts_ip = get_tailscale_ip()
    lan_ip = get_lan_ip()
    line = "=" * 56
    print(f"\n{line}")
    label = f" [{name}]" if name else ""
    print(f"  Open the gait tuner{label} in a browser:")
    if ts_ip:
        print(f"    • Tailscale:  http://{ts_ip}:{port}   (works anywhere on your tailnet)")
    if lan_ip:
        print(f"    • Local net:  http://{lan_ip}:{port}")
    if not ts_ip and not lan_ip:
        print(f"    • http://<this-pi-ip>:{port}   (run `hostname -I` to find it)")
    print(f"    • On the Pi:  http://localhost:{port}")
    print(f"{line}\n", flush=True)

import gait
from gait import (GaitParams, LEGS, GAIT_PATTERNS, foot_position, generate_cache,
                  effective_offsets)
from urdf_robot import load_pupper_urdf


LEG_COLORS = {
    "front_r": (235, 64, 52),    # red
    "front_l": (52, 199, 89),    # green
    "back_r":  (0, 122, 255),    # blue
    "back_l":  (255, 149, 0),    # orange
}


class GaitTunerApp:
    def __init__(self, robot=None, port=8080, name=None):
        self.params = GaitParams()
        self.robot = robot                 # optional RobotBridge (step 3)
        self.id = name or f"Group@{port}"  # identity for robot arbitration
        self.phase = 0.0
        self._dirty = True
        self._lock = threading.Lock()
        self.joints = None                 # latest (n_frames, 12) cache
        self.feet = None                   # latest (n_frames, 4, 3) foot cache

        self.server = viser.ViserServer(port=port)
        self.server.scene.set_up_direction("+z")
        self.traj_nodes = {}
        self._loading_preset = False
        self._suppress_toggle = False
        self._build_gui()
        self._build_scene()
        # default to showing the mesh robot if it loaded
        if self.urdf is not None:
            self.cb_robot.value = True
            self._on_show_robot()
        self.recompute()

        if self.robot is not None:
            self.robot.register_listener(self._on_owner_change)
            self._refresh_occupancy()
        threading.Thread(target=self._animate, daemon=True).start()
        print_url_banner(port, name=self.id)

    # ---------------------------------------------------------------- GUI
    def _build_gui(self):
        s = self.server.gui
        s.add_markdown("## 🐕 Pupper Gait Tuner\nDrag a slider — the gait rebuilds live.")

        with s.add_folder("Walk shape"):
            self.sl_height = s.add_slider("Body height (m)", 0.08, 0.18, 0.005, self.params.body_height)
            self.sl_steplen = s.add_slider("Step length (m)", 0.0, 0.16, 0.005, self.params.step_length)
            self.sl_stepht = s.add_slider("Step height (m)", 0.0, 0.12, 0.005, self.params.step_height)
            self.sl_width = s.add_slider("Stance width (m)", -0.04, 0.06, 0.005, self.params.stance_width)

        with s.add_folder("Timing"):
            self.sl_freq = s.add_slider("Frequency (Hz)", 0.2, 4.0, 0.1, self.params.frequency)
            self.sl_duty = s.add_slider("Duty (ground %)", 0.4, 0.9, 0.01, self.params.duty)

        with s.add_folder("Leg timing — phase offset (cycles)"):
            self.dd_pattern = s.add_dropdown(
                "Preset", tuple(GAIT_PATTERNS) + ("custom",),
                initial_value=self.params.pattern)
            preset = GAIT_PATTERNS[self.params.pattern]
            # one slider per leg: when does this leg's step start, 0..1 of a cycle
            self.sl_phase = {
                leg.name: s.add_slider(f"{leg.name}", 0.0, 1.0, 0.01, preset[leg.name])
                for leg in LEGS
            }

        with s.add_folder("Per-leg step tuning (× of global)", expand_by_default=False):
            self.sl_len_scale = {}
            self.sl_ht_scale = {}
            for leg in LEGS:
                with s.add_folder(leg.name):
                    self.sl_len_scale[leg.name] = s.add_slider(
                        "step length ×", 0.0, 2.0, 0.05, 1.0)
                    self.sl_ht_scale[leg.name] = s.add_slider(
                        "step height ×", 0.0, 2.0, 0.05, 1.0)

        for h in (self.sl_height, self.sl_steplen, self.sl_stepht, self.sl_width,
                  self.sl_freq, self.sl_duty):
            h.on_update(self._on_param_change)
        for h in self.sl_phase.values():
            h.on_update(self._on_phase_slider)
        for h in list(self.sl_len_scale.values()) + list(self.sl_ht_scale.values()):
            h.on_update(self._on_param_change)
        self.dd_pattern.on_update(self._on_pattern_preset)

        with s.add_folder("Robot"):
            self.txt_occupancy = s.add_text("Robot", "checking…", disabled=True)
            self.cb_send = s.add_checkbox("Send to robot", False)
            self.cb_send.on_update(self._on_send_toggle)
            self.btn_stand = s.add_button("Stand (reset pose)")
            self.btn_stand.on_click(self._on_stand)
            self.btn_reset = s.add_button("Reset sliders")
            self.btn_reset.on_click(self._on_reset)
            self.txt_status = s.add_text("You", "visualize-only", disabled=True)

        with s.add_folder("Display"):
            self.cb_robot = s.add_checkbox("Show robot mesh", False)
            self.cb_robot.on_update(self._on_show_robot)
            self.cb_paths = s.add_checkbox("Show foot paths", True)
            self.cb_paths.on_update(self._on_show_paths)

        self.txt_speed = s.add_text("Est. body speed", "—", disabled=True)

    def _on_show_robot(self, _=None):
        show = self.cb_robot.value
        if self.urdf is None:
            self.cb_robot.value = False
            return
        self.robot_root.visible = show
        self.body.visible = not show

    def _on_show_paths(self, _=None):
        for leg in LEGS:
            self.traj_nodes[leg.name].visible = self.cb_paths.value

    # -------------------------------------------------------------- Scene
    def _build_scene(self):
        sc = self.server.scene
        sc.add_grid("/ground", width=1.2, height=1.2, cell_size=0.05,
                    position=(0.0, 0.0, -self.params.body_height))
        # schematic body block (shown when the mesh robot is hidden)
        self.body = sc.add_box("/body", color=(120, 120, 130),
                               dimensions=(0.20, 0.11, 0.06), position=(0.0, 0.0, 0.0))
        # optional mesh robot (mirrors the real hardware pose)
        self.robot_root = sc.add_frame("/robot", show_axes=False)
        self.urdf = load_pupper_urdf(self.server, root="/robot")
        self._urdf_accum = 0.0    # throttle URDF updates (mesh repose is ~17 ms)
        if self.urdf is not None:
            self.body.visible = False          # prefer the mesh over the box
        self.markers = {}
        for leg in LEGS:
            # hip mounts as small frames for reference
            sc.add_icosphere(f"/hip/{leg.name}", radius=0.008, color=(80, 80, 80),
                             position=tuple(leg.base[:3, 3]))
            self.markers[leg.name] = sc.add_icosphere(
                f"/foot/{leg.name}", radius=0.013, color=LEG_COLORS[leg.name],
                position=(0, 0, 0))

    # ------------------------------------------------------------ Callbacks
    def _read_params(self):
        return GaitParams(
            body_height=self.sl_height.value,
            step_length=self.sl_steplen.value,
            step_height=self.sl_stepht.value,
            stance_width=self.sl_width.value,
            frequency=self.sl_freq.value,
            duty=self.sl_duty.value,
            pattern=self.dd_pattern.value if self.dd_pattern.value in GAIT_PATTERNS else "trot",
            # the four per-leg sliders are the source of truth for timing
            phase_offsets=tuple(self.sl_phase[leg.name].value for leg in LEGS),
            step_length_scale=tuple(self.sl_len_scale[leg.name].value for leg in LEGS),
            step_height_scale=tuple(self.sl_ht_scale[leg.name].value for leg in LEGS),
        )

    def _on_param_change(self, _=None):
        with self._lock:
            self.params = self._read_params().clamped()
            self._dirty = True

    def _on_pattern_preset(self, _=None):
        """Dropdown picked: load a preset's offsets into the four leg sliders."""
        name = self.dd_pattern.value
        if name in GAIT_PATTERNS:
            self._loading_preset = True
            for leg in LEGS:
                self.sl_phase[leg.name].value = GAIT_PATTERNS[name][leg.name]
            self._loading_preset = False
        self._on_param_change()

    def _on_phase_slider(self, _=None):
        """A leg's phase was dragged by hand -> mark the preset as 'custom'."""
        if self._loading_preset:
            return
        if self.dd_pattern.value != "custom":
            self.dd_pattern.value = "custom"   # fires _on_pattern_preset -> recompute
        else:
            self._on_param_change()

    def _on_send_toggle(self, _=None):
        if self._suppress_toggle:
            return
        if self.robot is None:
            self.cb_send.value = False
            self.txt_status.value = "no robot bridge"
            return
        if self.cb_send.value:
            # cooperative lock: only deploy if the robot is free
            if self.robot.try_acquire(self.id):
                self.robot.set_body_height(self.params.body_height)
                self.robot.update_cache(self.joints, self.params.frequency, self.id)
                self.txt_status.value = "● LIVE — driving motors"
            else:
                # occupied — refuse, revert the checkbox, tell them who has it
                self._suppress_toggle = True
                self.cb_send.value = False
                self._suppress_toggle = False
                who = self.robot.owner or "another process"
                self.txt_status.value = f"⛔ robot is busy ({who}) — wait for FREE"
        else:
            self.robot.release(self.id)
            self.txt_status.value = "visualize-only"
        self._refresh_occupancy()

    def _on_owner_change(self, owner):
        """Bridge told us control changed hands."""
        if owner == self.id:
            self.txt_status.value = "● LIVE — driving motors"
        elif self.cb_send.value:
            # we lost (or never had) control — reflect it in the UI
            self._suppress_toggle = True
            self.cb_send.value = False
            self._suppress_toggle = False
            self.txt_status.value = "visualize-only"
        self._refresh_occupancy()

    def _refresh_occupancy(self):
        """Update the shared occupancy banner + enable/disable our deploy box."""
        if self.robot is None:
            return
        ext = self.robot.external_publishers()
        owner = self.robot.owner
        if ext > 0:
            self.txt_occupancy.value = "🔴 OCCUPIED — external process driving robot"
            busy = True
        elif owner is None:
            self.txt_occupancy.value = "🟢 FREE — available to deploy"
            busy = False
        elif owner == self.id:
            self.txt_occupancy.value = "🟢 YOU are driving the robot"
            busy = False
        else:
            self.txt_occupancy.value = f"🔴 OCCUPIED by {owner}"
            busy = True
        # lock the deploy box when someone else (or an external node) has it
        self.cb_send.disabled = busy

    def _on_stand(self, _=None):
        # only the current driver can stand the robot; others just reset locally
        if self.robot is not None and self.robot.owner == self.id:
            self.robot.stand()
        if self.cb_send.value:
            self._suppress_toggle = True
            self.cb_send.value = False
            self._suppress_toggle = False
        self.txt_status.value = "standing"
        self._refresh_occupancy()

    def _on_reset(self, _=None):
        d = GaitParams()
        self.sl_height.value = d.body_height
        self.sl_steplen.value = d.step_length
        self.sl_stepht.value = d.step_height
        self.sl_width.value = d.stance_width
        self.sl_freq.value = d.frequency
        self.sl_duty.value = d.duty
        self._loading_preset = True
        for leg in LEGS:
            self.sl_phase[leg.name].value = GAIT_PATTERNS[d.pattern][leg.name]
            self.sl_len_scale[leg.name].value = 1.0
            self.sl_ht_scale[leg.name].value = 1.0
        self._loading_preset = False
        self.dd_pattern.value = d.pattern
        self._on_param_change()

    # ------------------------------------------------------------- Compute
    def recompute(self):
        with self._lock:
            p = self.params
        joints, feet = generate_cache(p)
        with self._lock:
            self.joints, self.feet = joints, feet
            self._dirty = False
        self._draw_trajectories(p, feet)
        self.body.position = (0.0, 0.0, 0.0)
        # ground follows body height
        self.server.scene.add_grid("/ground", width=1.2, height=1.2, cell_size=0.05,
                                    position=(0.0, 0.0, -p.body_height))
        # rough body speed estimate: stance foot displacement * frequency
        self.txt_speed.value = f"{p.step_length * p.frequency:.2f} m/s (ideal)"
        # only push to the robot if we currently hold control
        if self.robot is not None and self.robot.owner == self.id:
            self.robot.set_body_height(p.body_height)
            self.robot.update_cache(joints, p.frequency, self.id)

    def _draw_trajectories(self, p, feet):
        sc = self.server.scene
        for li, leg in enumerate(LEGS):
            pts = feet[:, li, :]
            pts = np.vstack([pts, pts[0]])    # close the loop
            self.traj_nodes[leg.name] = sc.add_spline_catmull_rom(
                f"/traj/{leg.name}", points=pts, closed=True,
                color=LEG_COLORS[leg.name], line_width=3.0,
                visible=self.cb_paths.value)

    # ----------------------------------------------------------- Animation
    def _animate(self):
        last = time.time()
        occ_accum = 0.0
        while True:
            now = time.time()
            dt = now - last
            last = now
            if self._dirty:
                self.recompute()
            # poll shared occupancy so waiting groups see FREE/OCCUPIED live
            occ_accum += dt
            if self.robot is not None and occ_accum >= 0.5:
                occ_accum = 0.0
                self._refresh_occupancy()
            with self._lock:
                p = self.params
            self.phase = (self.phase + p.frequency * dt) % 1.0
            offsets = effective_offsets(p)
            for leg in LEGS:
                pos = foot_position(self.phase + offsets[leg.name], leg.name, p)
                self.markers[leg.name].position = tuple(pos)
            # pose the mesh robot from the joint cache, throttled to ~15 Hz
            self._urdf_accum += dt
            if (self.urdf is not None and self.cb_robot.value
                    and self._urdf_accum >= 1 / 15):
                self._urdf_accum = 0.0
                self.urdf.update_from_vector(self._sample_joints(self.phase))
            time.sleep(1 / 30)

    def _sample_joints(self, phase):
        """Interpolate the joint cache at a cycle phase in [0,1)."""
        with self._lock:
            cache = self.joints
        n = cache.shape[0]
        f = (phase % 1.0) * n
        i0 = int(f) % n
        i1 = (i0 + 1) % n
        a = f - int(f)
        return cache[i0] * (1 - a) + cache[i1] * a


def main():
    app = GaitTunerApp()
    print("Gait tuner running — open http://<pi-ip>:8080")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
