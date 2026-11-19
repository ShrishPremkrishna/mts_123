"""Leg-kick bag timeline: 1 s windows with clip, joint speed, IMU rate, tilt and
accel spread; kick summaries. Local times are printed."""
import datetime
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
jt, jv, names = [], [], None
it, iw, ia, itilt = [], [], [], []
st = []
while r.has_next():
  topic, data, t = r.read_next()
  t *= 1e-9
  if topic == "/joint_states":
    m = deserialize_message(data, JointState); names = list(m.name); jt.append(t); jv.append(m.velocity)
  elif topic.endswith("/imu"):
    m = deserialize_message(data, Imu)
    q = m.orientation
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))))
    it.append(t); iw.append(np.linalg.norm([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]))
    ia.append(np.linalg.norm([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]))
    itilt.append(math.hypot(roll, pitch))
  elif topic.endswith("clip_status"):
    st.append((t, deserialize_message(data, String).data.split(" frame")[0]))
jt, jv = np.array(jt), np.abs(np.array(jv)); it, iw, ia, itilt = map(np.array, (it, iw, ia, itilt))
stt = np.array([s[0] for s in st]); stn = [s[1] for s in st]
fmt = lambda t: datetime.datetime.fromtimestamp(t).strftime("%I:%M:%S")
# kicks
kicks, cur = [], None
for t, s in st:
  if s.startswith("kick") and (cur is None):
    cur = [t, t, s]
  elif s.startswith("kick"):
    cur[1] = t
  elif cur is not None:
    kicks.append(cur); cur = None
for a, b, s in kicks:
  m = (jt >= a) & (jt <= b); mi = (it >= a) & (it <= b)
  j = int(np.argmax(jv[m].max(0)))
  print(f"{s} {fmt(a)}-{fmt(b)} ({b - a:.2f} s): max|qd| {jv[m].max():.1f} rad/s ({names[j]}), IMU |w| max {iw[mi].max():.2f} rad/s, tilt max {itilt[mi].max():.1f} deg")
print("time     | clip          | max|qd| (joint)      | rms qd | |w| max | tilt | accel mean/std m/s2")
t0, t1 = jt[0], jt[-1]
prev = None
for w in np.arange(math.floor(t0), t1, 1.0):
  m = (jt >= w) & (jt < w + 1); mi = (it >= w) & (it < w + 1); ms = (stt >= w) & (stt < w + 1)
  if m.sum() < 10 or mi.sum() < 10: continue
  clip = stn[int(np.flatnonzero(ms)[-1])] if ms.any() else "(walker/off)"
  v = jv[m]; j = int(np.argmax(v.max(0)))
  row = (clip, round(float(v.max()), 0) >= 2, round(float(np.std(ia[mi])), 0) >= 1)
  line = f"{fmt(w)} | {clip:13s} | {v.max():5.1f} ({names[j][4:]:9s}) | {np.sqrt((v**2).mean()):5.2f} | {iw[mi].max():5.2f} | {itilt[mi].max():4.1f} | {ia[mi].mean():5.2f}/{np.std(ia[mi]):4.2f}"
  if row != prev or v.max() > 2 or np.std(ia[mi]) > 0.8:
    print(line)
  prev = row
