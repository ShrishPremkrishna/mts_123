"""Reference-offset discontinuity when the command flips a gait branch between two ticks (port of gait_reference.hpp)."""
import json, sys, numpy as np
j = json.load(open(sys.argv[1])); g = j["gait_reference"]; d = np.array(j["default_joint_pos"])
T = {k: np.array(g[k]).reshape(g["n_samples"], -1) for k in ("trot_table", "gallop_table", "lift_table", "gallop_back_table")}
def ref(t, vx, vy, yaw):
    gal = abs(vx) >= g["gallop_speed"]; f = g["frequency"] * (g["gallop_freq_mult"] if gal else 1.0)
    ph = np.modf(t * f)[0]; tr = abs(vx) >= g["dir_threshold"]; ds = vx if tr else yaw
    ep = ph if ds >= 0 else -ph; pos = ep * g["n_samples"]; pf = np.floor(pos)
    i0 = int(pf) % g["n_samples"]; i1 = (i0 + 1) % g["n_samples"]; a = pos - pf
    slow = T["trot_table"] if tr else T["lift_table"]; fast = T["gallop_table"] if ds >= 0 else T["gallop_back_table"]
    tab = fast if gal else slow; sp = np.sqrt(vx*vx + vy*vy + yaw*yaw); b = min(max(sp / g["blend_speed"], 0), 1)
    return b * ((tab[i0] * (1 - a) + tab[i1] * a) - d)
ts = np.arange(0, 6, 1/52)
def jump(c1, c2):
    return np.array([np.abs(ref(t, *c2) - ref(t, *c1)).max() for t in ts])
def step(c):  # normal per-tick change on one branch
    return np.array([np.abs(ref(t + 1/52, *c) - ref(t, *c)).max() for t in ts])
gs = g["gallop_speed"]
print("normal per-tick ref change, trot vx=0.45:", step((0.45, 0, 0)).mean().round(3), " gallop vx=0.55:", step((0.55, 0, 0)).mean().round(3))
print("gallop<->trot flip at vx %.2f/%.2f: mean max-joint jump %.3f rad" % (gs - 0.01, gs, jump((gs - 0.01, 0, 0), (gs, 0, 0)).mean()))
print("trot<->lift flip at vx 0.049/0.050 (yaw 0.02): %.3f rad" % jump((0.049, 0, 0.02), (0.05, 0, 0.02)).mean())
print("lift dir flip yaw +0.02/-0.02 (vx 0): %.3f rad" % jump((0, 0, 0.02), (0, 0, -0.02)).mean())
print("fwd/back flip vx +0.06/-0.06: %.3f rad" % jump((0.06, 0, 0), (-0.06, 0, 0)).mean())
