#!/usr/bin/env python3
"""Replay the real follower with no ROS, camera or robot.

Two modes:

    python3 robot/follow_replay.py                 # every scripted scene, color off and on
    python3 robot/follow_replay.py crossing --color on
    python3 robot/follow_replay.py --log run.jsonl  # a detection log from the robot
    python3 robot/follow_replay.py --log run.jsonl --table   # its numbers, second by second

A scene is a small planar world: a robot that obeys the follower's commands,
and people on scripted paths. The world makes detections in the 700 x 572
reference frame and feeds them to the actual StateMachineNode from
lab/follow_me.py, so a scene closes the loop through your state machine.

This checks decision logic only. The camera geometry below is assumed, not
measured, so a scene's distances and speeds are not claims about the robot.
"""

import argparse
import json
import math
import os
import random
import sys
from dataclasses import dataclass, field
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import target_lock  # noqa: E402
from boxes import IMAGE_HEIGHT, IMAGE_WIDTH, box_bottom  # noqa: E402
from lab_checks import FakeDetection, FakeTime, load_follow_me  # noqa: E402

fm = load_follow_me()

# --- Assumed geometry (calibrate on the robot before trusting a distance) -----
H_FOV_DEG = 220.0
PX_PER_DEG = IMAGE_WIDTH / H_FOV_DEG
CAMERA_HEIGHT = 0.20      # m
CAMERA_PITCH_UP = 10.0    # deg: not in the robot model; STOP_BOTTOM's real distance depends on it
PERSON_HEIGHT = 1.70      # m
PERSON_WIDTH = 0.45       # m
MAX_RANGE = 6.0           # m: further than this the detector misses a person
# The projection spans 180 degrees over the image height, so row 286 is the
# camera axis, and tilting the camera up moves the horizon down the image.
HORIZON_ROW = IMAGE_HEIGHT / 2 + CAMERA_PITCH_UP * PX_PER_DEG

PHYSICS_DT = 0.02         # s
TICK_DT = 0.1             # s: the follower's timer
MUX_TIMEOUT = 0.5         # s: cmd_vel_mux forgets a speed it has not heard for this long
VELOCITY_LAG = 0.15       # s: the walking policy's response to a new speed
BOX_JITTER_PX = 2.0
COLOR_JITTER = 6.0        # RGB units
NMS_OVERLAP = 0.6         # the nearer box hides the further one above this overlap

RED = (200, 40, 40)
BLUE = (40, 60, 200)
GRAY = (120, 120, 120)


class Twist:
    """Stands in for geometry_msgs/Twist."""

    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


def _null(*_args, **_kwargs):
    return None


class SimLogger:
    info = warning = error = staticmethod(_null)


@dataclass
class Person:
    """Somebody in the world. path is [(t, x, y), ...]; hidden is [(t0, t1), ...]."""
    name: str
    color: tuple
    path: list
    hidden: list = field(default_factory=list)
    cropped: list = field(default_factory=list)  # [(t0, t1), ...] legs hidden, box 60% height

    def at(self, t):
        path = self.path
        if t <= path[0][0]:
            return path[0][1], path[0][2]
        for (t0, x0, y0), (t1, x1, y1) in zip(path, path[1:]):
            if t <= t1:
                s = (t - t0) / (t1 - t0)
                return x0 + s * (x1 - x0), y0 + s * (y1 - y0)
        return path[-1][1], path[-1][2]

    def is_hidden(self, t):
        return any(t0 <= t < t1 for t0, t1 in self.hidden)

    def is_cropped(self, t):
        return any(t0 <= t < t1 for t0, t1 in self.cropped)


@dataclass
class Scene:
    """One scripted situation. The first person is the intended target."""
    name: str
    seconds: float
    people: list
    frame_hz: float = 10.0
    latency: float = 0.12              # s from capture to detections reaching the follower
    camera_stalls: list = field(default_factory=list)   # [(t0, t1), ...] no frames at all
    corrupt_at: list = field(default_factory=list)      # [t, ...] frames with corrupt geometry
    dropout: float = 0.0               # chance a person is missed in one frame
    note: str = ''


def box_for(person_xy, robot, color, name, rng, cropped=False):
    """The box a person makes in the reference frame, or None if out of view."""
    x, y, heading = robot
    dx, dy = person_xy[0] - x, person_xy[1] - y
    forward = math.cos(heading) * dx + math.sin(heading) * dy
    left = -math.sin(heading) * dx + math.cos(heading) * dy
    distance = math.hypot(forward, left)
    bearing = math.degrees(math.atan2(left, forward))
    half_width = math.degrees(math.atan2(PERSON_WIDTH / 2, max(distance, 0.05))) * PX_PER_DEG
    if distance > MAX_RANGE or abs(bearing) * PX_PER_DEG + half_width > IMAGE_WIDTH / 2:
        return None
    cx = IMAGE_WIDTH / 2 - bearing * PX_PER_DEG
    feet = HORIZON_ROW + math.degrees(math.atan2(CAMERA_HEIGHT, max(distance, 0.05))) * PX_PER_DEG
    head = HORIZON_ROW - math.degrees(math.atan2(PERSON_HEIGHT - CAMERA_HEIGHT, max(distance, 0.05))) * PX_PER_DEG
    feet = head + 0.6 * (feet - head) if cropped else feet
    bottom = min(feet, IMAGE_HEIGHT) + rng.gauss(0, BOX_JITTER_PX)
    top = max(head, 0.0) + rng.gauss(0, BOX_JITTER_PX)
    cx += rng.gauss(0, BOX_JITTER_PX)
    tint = [min(255, max(0, c + rng.gauss(0, COLOR_JITTER))) for c in color]
    det = FakeDetection(cx - half_width, top, cx + half_width, bottom, class_id=0, score=0.8,
                        color=[int(c) for c in tint])
    det.truth = name
    det.distance = distance
    return det


def close_ahead(person_xy, robot):
    """Is this person inside the stop distance and within IN_THE_WAY_BAND of straight ahead?"""
    x, y, heading = robot
    dx, dy = person_xy[0] - x, person_xy[1] - y
    forward = math.cos(heading) * dx + math.sin(heading) * dy
    left = -math.sin(heading) * dx + math.cos(heading) * dy
    distance = math.hypot(forward, left)
    down = HORIZON_ROW + math.degrees(math.atan2(CAMERA_HEIGHT, max(distance, 0.05))) * PX_PER_DEG
    offset = math.degrees(math.atan2(left, forward)) * PX_PER_DEG / IMAGE_WIDTH
    return down >= target_lock.STOP_BOTTOM and abs(offset) <= target_lock.IN_THE_WAY_BAND


def overlap(a, b):
    """Fraction of the narrower box covered by the other, across the image."""
    a0, a1 = a.bbox.center.position.x - a.bbox.size_x / 2, a.bbox.center.position.x + a.bbox.size_x / 2
    b0, b1 = b.bbox.center.position.x - b.bbox.size_x / 2, b.bbox.center.position.x + b.bbox.size_x / 2
    shared = max(0.0, min(a1, b1) - max(a0, b0))
    return shared / max(min(a1 - a0, b1 - b0), 1e-6)


def detect(scene, robot, t, capture_t, rng):
    """The people the detector would report for a frame taken at capture_t."""
    boxes = []
    for person in scene.people:
        if person.is_hidden(capture_t) or rng.random() < scene.dropout:
            continue
        box = box_for(person.at(capture_t), robot, person.color, person.name, rng, person.is_cropped(capture_t))
        if box is not None:
            boxes.append(box)
    boxes.sort(key=lambda d: d.distance)
    kept = []
    for box in boxes:
        if not any(overlap(box, near) > NMS_OVERLAP for near in kept):
            kept.append(box)
    return kept


def make_node(world, use_color):
    target_lock.USE_COLOR = use_color
    fm.ENABLE_OBSTACLE_AVOIDANCE = False
    fm.Twist = Twist
    node = object.__new__(fm.StateMachineNode)
    node.state, node.tracking_enabled = fm.State.IDLE, False
    node.class_names = ['person']
    node.class_name_to_id = {'person': 0}
    node.target_class_id, node.target_object = 0, 'person'
    node.target_pos, node.target_bottom = 0.0, 0.0
    node.last_detection_time = node.last_frame_time = None
    node.lock = target_lock.Lock()
    node.obstacle_pos, node.crowding = None, (0.0, 0.0)
    node.avoid_dir, node.avoid_done_time = 1.0, None
    node.phase_start = FakeTime(0.0)
    node.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime(world['t']))
    node.get_logger = lambda: SimLogger
    node.command_publisher = SimpleNamespace(publish=world['commands'].append)
    # Remember who the follower chose, for the score.
    choose = node.process_detections

    def process_and_note(detections):
        chosen = choose(detections)
        world['chosen'] = getattr(chosen, 'truth', None) if chosen is not None else None
        if chosen is not None:
            world['locked_on'] = world['chosen']
        return chosen
    node.process_detections = process_and_note
    return node


def run_scene(scene, use_color=False, seed=7):
    """Run one scene. Returns the score as a dict."""
    rng = random.Random(seed)
    world = {'t': 0.0, 'commands': [], 'chosen': None, 'locked_on': None}
    node = make_node(world, use_color)
    node.start_tracking('person')

    x = y = heading = 0.0
    vx = wz = 0.0
    cmd = (0.0, 0.0)
    last_cmd_t = -1e9
    next_tick = next_frame = 0.0
    pending = []  # (delivery time, detections)
    intended = scene.people[0].name
    score = dict(scene=scene.name, color=use_color, wrong_s=0.0, wrong_first_t=None, walked_blind_m=0.0,
                 spin_s=0.0, spin_longest_s=0.0, switches=[], states={}, min_gap_m=float('inf'),
                 pushed_person_s=0.0)
    spin_run = 0.0
    previous_lock = None
    steps = int(round(scene.seconds / PHYSICS_DT))
    for step in range(steps):
        t = step * PHYSICS_DT
        world['t'] = t
        if t + 1e-9 >= next_frame:
            next_frame += 1.0 / scene.frame_hz
            if not any(t0 <= t < t1 for t0, t1 in scene.camera_stalls):
                detections = detect(scene, (x, y, heading), t, t, rng)
                if any(abs(t - c) < 0.5 / scene.frame_hz for c in scene.corrupt_at):
                    for det in detections:
                        det.bbox.size_x = float('nan')
                pending.append((t + scene.latency, detections))
        while pending and pending[0][0] <= t + 1e-9:
            _, detections = pending.pop(0)
            node.detection_callback(SimpleNamespace(detections=detections))
        if t + 1e-9 >= next_tick:
            next_tick += TICK_DT
            node.timer_callback()
            while world['commands']:
                msg = world['commands'].pop(0)
                cmd = (msg.linear.x, msg.angular.z)
                last_cmd_t = t

            name = node.state.name
            score['states'][name] = round(score['states'].get(name, 0.0) + TICK_DT, 3)
            if world['locked_on'] != previous_lock:
                score['switches'].append((round(t, 2), previous_lock, world['locked_on']))
                previous_lock = world['locked_on']
            moving = abs(cmd[0]) > 1e-6 or abs(cmd[1]) > 1e-6
            if node.state == fm.State.TRACK and moving and world['locked_on'] not in (None, intended):
                score['wrong_s'] = round(score['wrong_s'] + TICK_DT, 3)
                if score['wrong_first_t'] is None:
                    score['wrong_first_t'] = round(t, 2)
            ahead = [person for person in scene.people if not person.is_hidden(t)
                     and close_ahead(person.at(t), (x, y, heading))]
            if ahead and cmd[0] > 1e-6:
                score['pushed_person_s'] = round(score['pushed_person_s'] + TICK_DT, 3)
            spinning = node.state == fm.State.SEARCH and abs(cmd[1]) > 1e-6
            spin_run = spin_run + TICK_DT if spinning else 0.0
            score['spin_s'] = round(score['spin_s'] + (TICK_DT if spinning else 0.0), 3)
            score['spin_longest_s'] = round(max(score['spin_longest_s'], spin_run), 3)
        if t - last_cmd_t > MUX_TIMEOUT:
            cmd = (0.0, 0.0)
        step_gain = PHYSICS_DT / VELOCITY_LAG
        vx += (cmd[0] - vx) * step_gain
        wz += (cmd[1] - wz) * step_gain
        heading += wz * PHYSICS_DT
        x += vx * math.cos(heading) * PHYSICS_DT
        y += vx * math.sin(heading) * PHYSICS_DT
        last_seen = node.last_detection_time
        age = float('inf') if last_seen is None else t - last_seen.seconds
        if age > 0.5:
            score['walked_blind_m'] = round(score['walked_blind_m'] + abs(vx) * PHYSICS_DT, 4)
        for person in scene.people:
            if not person.is_hidden(t):  # a person who left is not an obstacle
                px, py = person.at(t)
                score['min_gap_m'] = min(score['min_gap_m'], math.hypot(px - x, py - y))

    tx, ty = scene.people[0].at(scene.seconds)
    score.update(final_state=node.state.name, final_gap_m=round(math.hypot(tx - x, ty - y), 3),
                 final_locked_on=world['locked_on'], min_gap_m=round(score['min_gap_m'], 3),
                 final_cmd=(round(cmd[0], 3), round(cmd[1], 3)),
                 first_lock=score['switches'][0][2] if score['switches'] else None)
    return score


# --- The scenes ---------------------------------------------------------------
def still(name, color, x, y, **kw):
    return Person(name, color, [(0.0, x, y)], **kw)


def scenes():
    me = 'me'
    other = 'other'
    return [
        Scene('approach_ahead', 40.0, [still(me, RED, 3.0, 0.0)],
              note='target stands still, straight ahead'),
        Scene('approach_off_axis', 40.0, [still(me, RED, 1.6, 2.0)],
              note='target stands still, 50 degrees to the left'),
        Scene('walk_away', 40.0, [Person(me, RED, [(0.0, 1.2, 0.0), (10.0, 4.0, 0.0)])],
              note='target walks away at 0.28 m/s, then stops'),
        Scene('short_occlusion', 25.0, [still(me, RED, 1.5, 0.0, hidden=[(10.0, 10.6)])],
              note='target hidden for 0.6 s: inside the timeout'),
        Scene('long_occlusion_alone', 30.0, [still(me, RED, 1.5, 0.0, hidden=[(10.0, 13.0)])],
              note='target hidden for 3 s, nobody else in view'),
        Scene('long_occlusion_bystander', 30.0,
              [still(me, RED, 1.5, 0.0, hidden=[(10.0, 13.0)]), still(other, BLUE, 2.0, -1.0)],
              note='target hidden for 3 s while a bystander stands in view'),
        Scene('gone_forever_alone', 40.0, [still(me, RED, 1.5, 0.0, hidden=[(8.0, 1e9)])],
              note='target leaves at 8 s and never returns: the search must end'),
        Scene('gone_then_stranger', 45.0,
              [still(me, RED, 1.5, 0.0, hidden=[(8.0, 1e9)]),
               Person(other, BLUE, [(0.0, 8.0, 0.0), (20.0, 8.0, 0.0), (26.0, 2.0, 0.0)])],
              note='target leaves; a stranger walks in later, alone in view'),
        Scene('crossing', 25.0,
              [Person(me, RED, [(0.0, 2.0, 0.2), (3.0, 2.0, 0.2), (9.0, 2.0, -1.6), (25.0, 2.0, -1.6)]),
               Person(other, BLUE, [(0.0, 2.4, -1.8), (3.0, 2.4, -1.8), (9.0, 2.4, 1.6), (25.0, 2.4, 1.6)])],
              note='two people in different colors cross; the target stays in front'),
        Scene('crossing_behind', 25.0,
              [Person(me, RED, [(0.0, 2.4, 0.2), (3.0, 2.4, 0.2), (9.0, 2.4, -1.6), (25.0, 2.4, -1.6)]),
               Person(other, BLUE, [(0.0, 2.0, -1.8), (3.0, 2.0, -1.8), (9.0, 2.0, 1.6), (25.0, 2.0, 1.6)])],
              note='two people cross; the target passes behind the bystander'),
        Scene('crossing_same_color', 25.0,
              [Person(me, RED, [(0.0, 2.4, 0.2), (3.0, 2.4, 0.2), (9.0, 2.4, -1.6), (25.0, 2.4, -1.6)]),
               Person(other, RED, [(0.0, 2.0, -1.8), (3.0, 2.0, -1.8), (9.0, 2.0, 1.6), (25.0, 2.0, 1.6)])],
              note='the same crossing in the same color: color cannot help'),
        Scene('background_walker', 25.0,
              [still(me, RED, 1.5, 0.0),
               Person(other, BLUE, [(0.0, 5.0, 3.0), (6.0, 5.0, 3.0), (18.0, 5.0, -3.0)])],
              note='a far person walks across behind the target: no reason to stop'),
        Scene('side_by_side', 30.0,
              [Person(me, RED, [(0.0, 1.5, 0.1), (20.0, 5.5, 0.1)]),
               Person(other, BLUE, [(0.0, 1.5, -1.0), (20.0, 5.5, -1.0)])],
              note='target walks away next to a friend, 1.1 m apart'),
        Scene('stranger_in_front', 25.0,
              [still(me, RED, 3.0, 0.0),
               Person(other, BLUE, [(0.0, 1.2, 2.0), (3.0, 1.2, 2.0), (9.0, 1.2, -2.0)])],
              note='a near stranger walks between the robot and a far target'),
        Scene('stranger_same_color', 45.0,
              [still(me, RED, 1.5, 0.0, hidden=[(8.0, 1e9)]),
               Person(other, RED, [(0.0, 8.0, 0.0), (20.0, 8.0, 0.0), (26.0, 2.0, 0.0)])],
              note='target leaves; a stranger in the same color walks in'),
        Scene('legs_hidden', 25.0, [still(me, RED, 3.0, 0.0, cropped=[(4.0, 8.0)])],
              note='a table hides the legs for 4 s: the box is 60% as tall'),
        Scene('camera_freeze', 25.0, [Person(me, RED, [(0.0, 3.0, 0.0)])],
              camera_stalls=[(4.0, 9.0)], note='no frames for 5 s while walking in'),
        Scene('corrupt_frames', 25.0, [still(me, RED, 3.0, 0.0)],
              corrupt_at=[5.0, 5.1, 5.2, 5.3, 5.4], note='half a second of corrupt boxes'),
        Scene('slow_detector', 30.0,
              [Person(me, RED, [(0.0, 2.0, 1.5), (6.0, 2.0, 1.5), (9.0, 2.0, -1.5), (30.0, 2.0, -1.5)])],
              frame_hz=4.0, latency=0.3, note='detector at 4 Hz, target moves quickly across'),
        Scene('dropouts', 30.0, [still(me, RED, 3.0, 0.0)], dropout=0.3,
              note='the detector misses the target in 30 percent of frames'),
    ]


# --- Detection logs from the robot ---------------------------------------------
def log_frames(path):
    """Yield (t, detections) from a log follow_service.py wrote (-p record_path:=...)."""
    with open(path) as lines:
        for line in lines:
            entry = json.loads(line)
            if 'frame' not in entry:
                continue
            detections = []
            for cx, cy, w, h, class_id, score, color in entry['frame']:
                det = FakeDetection(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, class_id=class_id, score=score)
                det.id = color
                detections.append(det)
            yield entry['t'], detections


def log_table(path):
    """Second by second: the largest person's box bottom, width and mask color.

    Stand at a measured distance for a few seconds at a time to calibrate
    STOP_BOTTOM, and walk around in one shirt to see how far its color drifts.
    """
    seconds = {}
    start = None
    for t, detections in log_frames(path):
        start = t if start is None else start
        people = [d for d in detections if d.results[0].hypothesis.class_id == '0']
        seconds.setdefault(int(t - start), []).append(people)
    print(f'{"sec":>4}{"frames":>7}{"people":>7}{"bottom":>8}{"width":>7}  color')
    for second, frames in sorted(seconds.items()):
        tallest = [max(people, key=lambda d: d.bbox.size_y) for people in frames if people]
        count = max((len(people) for people in frames), default=0)
        if not tallest:
            print(f'{second:4}{len(frames):7}{count:7}')
            continue
        tallest.sort(key=box_bottom)
        middle = tallest[len(tallest) // 2]
        print(f'{second:4}{len(frames):7}{count:7}{box_bottom(middle):8.0f}{middle.bbox.size_x:7.0f}  {middle.id}')


def replay_log(path, use_color):
    """Feed a recorded log through the state machine, open loop.

    The robot's motion is whatever happened on the day, so this checks target
    selection and state changes, not closed-loop behavior. Replaying a log after
    changing a constant shows exactly which frames decide differently.
    """
    world = {'t': 0.0, 'commands': [], 'chosen': None, 'locked_on': None}
    node = make_node(world, use_color)
    node.start_tracking('person')
    pick = node.process_detections
    picked = []

    def keep(detections):
        picked.append(pick(detections))
        return picked[-1]
    node.process_detections = keep
    jumps, frames, misses = [], 0, 0
    next_tick = None
    previous = None
    for t, detections in log_frames(path):
        world['t'] = t
        next_tick = t if next_tick is None else next_tick
        node.detection_callback(SimpleNamespace(detections=detections))
        chosen = picked[-1] if picked else None
        picked.clear()
        frames += 1
        misses += chosen is None
        if chosen is not None:
            here = target_lock.box_position(chosen)
            if previous is not None and math.dist(here, previous) > target_lock.LOCK_GATE:
                jumps.append((round(t, 2), round(math.dist(here, previous), 3)))
            previous = here
        while next_tick <= t:
            node.timer_callback()
            world['commands'].clear()
            next_tick += TICK_DT
    return dict(frames=frames, frames_without_target=misses, jumps=jumps, final_state=node.state.name)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scene', nargs='?', help='one scene; all scenes if left out')
    parser.add_argument('--color', choices=['off', 'on', 'both'], default='both')
    parser.add_argument('--log', help='replay a detection log from the robot instead')
    parser.add_argument('--table', action='store_true', help="print the log's numbers, second by second")
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--json', action='store_true', help='print whole scores as JSON lines')
    args = parser.parse_args()
    colors = {'off': [False], 'on': [True], 'both': [False, True]}[args.color]
    if args.log and args.table:
        log_table(args.log)
        return
    if args.log:
        for use_color in colors:
            print(f'color {"on" if use_color else "off"}:', replay_log(args.log, use_color))
        return
    chosen = [s for s in scenes() if args.scene in (None, s.name)]
    if not chosen:
        raise SystemExit(f'unknown scene {args.scene!r}. Choose from: {", ".join(s.name for s in scenes())}')
    print(f'{"scene":26}{"color":6}{"wrong_s":>8}{"blind_m":>8}{"spin_s":>7}{"longest":>8}{"gap_m":>7}'
          f'{"min_gap":>8}  {"final":9}{"locked":7}')
    for scene in chosen:
        for use_color in colors:
            s = run_scene(scene, use_color, args.seed)
            if args.json:
                print(json.dumps(s))
                continue
            print(f'{s["scene"]:26}{"on" if use_color else "off":6}{s["wrong_s"]:8.1f}{s["walked_blind_m"]:8.2f}'
                  f'{s["spin_s"]:7.1f}{s["spin_longest_s"]:8.1f}{s["final_gap_m"]:7.2f}{s["min_gap_m"]:8.2f}'
                  f'  {s["final_state"]:9}{str(s["final_locked_on"]):7}')


if __name__ == '__main__':
    main()
