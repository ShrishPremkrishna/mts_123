import math, sys, time
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/imu_sensor_broadcaster/imu", "/joint_states", "/llm_cmd_vel"]))
W = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
t0 = None; win = {}; n = {"i": 0, "j": 0}
def slot(t):
    return win.setdefault(int((t - t0) / W), {"tilt": [], "yaw": [], "gyro": [], "qd": [], "q": [], "cmd": 0})
while r.has_next():
    topic, data, t = r.read_next(); t *= 1e-9
    if t0 is None: t0 = t
    if topic.endswith("imu"):
        n["i"] += 1
        if n["i"] % 25: continue
        m = deserialize_message(data, Imu); q = m.orientation
        zz = 1 - 2 * (q.x * q.x + q.y * q.y)
        s = slot(t)
        s["tilt"].append(math.degrees(math.acos(max(-1, min(1, zz)))))
        s["yaw"].append(math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        g = m.angular_velocity; s["gyro"].append(math.sqrt(g.x**2 + g.y**2 + g.z**2))
    elif topic == "/joint_states":
        n["j"] += 1
        if n["j"] % 10: continue
        m = deserialize_message(data, JointState); s = slot(t)
        s["qd"].append(max(abs(v) for v in m.velocity)); s["q"].append(list(m.position))
    else:
        m = deserialize_message(data, Twist)
        if abs(m.linear.x) + abs(m.linear.y) + abs(m.angular.z) > 1e-6: slot(t)["cmd"] += 1
print("start", time.strftime("%H:%M:%S", time.localtime(t0)))
for k in sorted(win):
    s = win[k]
    if not s["tilt"] or not s["qd"]: continue
    qr = max(max(c) - min(c) for c in zip(*s["q"]))
    qd = sorted(s["qd"]); gy = sorted(s["gyro"])
    print(time.strftime("%H:%M:%S", time.localtime(t0 + k * W)),
          "tilt %5.1f..%5.1f deg  yaw %+7.1f->%+7.1f  gyro med %.3f max %.2f rad/s  joint |qd| med %.2f max %5.2f rad/s  joint range %.2f rad  moving cmds %d"
          % (min(s["tilt"]), max(s["tilt"]), s["yaw"][0], s["yaw"][-1], gy[len(gy)//2], gy[-1], qd[len(qd)//2], qd[-1], qr, s["cmd"]))
