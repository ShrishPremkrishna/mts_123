#!/usr/bin/env python3
"""Desk-side tests for the whole lab -- no robot, no camera, no ROS needed.

    python3 lab/test.py           # Steps 2-4, the required lab
    python3 lab/test.py step2     # Step 2: follow
    python3 lab/test.py step3     # Step 3: follow *me*, specifically
    python3 lab/test.py step4     # Step 4: follow me, at a polite distance
    python3 lab/test.py step5     # Optional Step 5: follow me, very specifically
    python3 lab/test.py step6     # Optional Step 6: something in the way

Each scene is a few fake detections laid out like a real frame. A check whose
code you have not written yet is skipped ("--"), not failed, so run it after
every function. A failing check prints what your code returned and an
example that passes. The handout lists every check.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'robot'))

import avoidance  # noqa: E402
import boxes  # noqa: E402
import target_lock  # noqa: E402
from boxes import IMAGE_HEIGHT as H, IMAGE_WIDTH as W  # noqa: E402
from lab_checks import FakeDetection, FakeTime, check, fake_node, load_follow_me, need, run  # noqa: E402
from target_lock import Lock  # noqa: E402

fm = load_follow_me()
Node = fm.StateMachineNode
State = fm.State

# Shirt colors, for Optional Step 5.
RED = (200, 40, 40)
BLUE = (40, 60, 200)
REDDISH = (185, 55, 50)   # the red shirt under slightly different light

# Margins: answers this close to the expected one pass, so rounding or an
# equivalent formula never fails a check.
SPEED_TOL = 0.01      # m/s and rad/s
POSITION_TOL = 0.005  # image fractions
COLOR_TOL = 1.0       # RGB units (0-255)


def person(cx, bottom, color=None, width=120, height=300):
    """A fake person: box centred at x=cx with its bottom edge at `bottom`."""
    return FakeDetection(cx - width / 2, bottom - height, cx + width / 2, bottom,
                         class_id=0, color=color)


def tracker(**fields):
    """A fake StateMachineNode that just started chasing people."""
    defaults = dict(state=State.TRACK, target_class_id=0, lock=Lock(), target_pos=0.0,
                    target_bottom=0.0, last_detection_time=None, obstacle_pos=None,
                    avoid_done_time=None)
    defaults.update(fields)
    return fake_node(**defaults)


def locked_on(det):
    """A lock that has just been following `det` (uses your remember)."""
    lock = Lock()
    target_lock.remember(lock, det)
    return lock


def processed(frame, **fields):
    """A fake node after your process_detections has seen `frame`."""
    node = tracker(**fields)
    Node.process_detections(node, frame)
    return node


def pick(targets, lock_fn):
    """pick_target's choice (the lock is built lazily, so skips work)."""
    return target_lock.pick_target(targets, lock_fn())


def step2():
    centred = FakeDetection(W / 2 - 50, 100, W / 2 + 50, 300)
    check("box_offset of a centred box is 0",
          lambda: boxes.box_offset(centred), 0.0, tol=1e-6,
          passes="centre x = 350 of 700 -> 0.0")
    at_right = FakeDetection(W - 20, 100, W, 300)
    check("box_offset near the right edge is about +0.49",
          lambda: boxes.box_offset(at_right), 0.486, tol=0.01,
          passes="centre x = 690 of 700 -> 690 / 700 - 0.5 = 0.486")

    timeout = lambda: need("TIMEOUT", fm.TIMEOUT)  # noqa: E731
    check("choose_state: seen within TIMEOUT -> TRACK",
          lambda: Node.choose_state(tracker(), 0.5 * timeout()), State.TRACK,
          passes="State.TRACK")
    check("choose_state: not seen for longer than TIMEOUT -> SEARCH",
          lambda: Node.choose_state(tracker(), 2.0 * timeout()), State.SEARCH,
          passes="State.SEARCH")
    check("choose_state: never seen -> SEARCH",
          lambda: Node.choose_state(tracker(), float('inf')), State.SEARCH,
          passes="State.SEARCH")

    chair = FakeDetection(W / 2 - 40, 300, W / 2 + 40, 500, class_id=56)
    left, middle = person(200, 400), person(370, 400)
    check("process_detections: the most centred PERSON (not the more centred chair)",
          lambda: Node.process_detections(tracker(), [chair, left, middle]), middle,
          passes="the person at x=370")
    check("process_detections: stores the target's box_offset in self.target_pos",
          lambda: processed([chair, left, middle]).target_pos, 370 / W - 0.5, tol=1e-6,
          passes="370 / 700 - 0.5 = 0.029")
    check("process_detections: stamps self.last_detection_time with the clock",
          lambda: (lambda n: n.last_detection_time is n.clock_now)(processed([middle])), True,
          passes="self.last_detection_time = self.get_clock().now()")
    check("process_detections: a frame with no person -> None",
          lambda: Node.process_detections(tracker(), [chair]), None,
          passes="None (a chair is not our class)")
    check("process_detections: an empty frame -> None",
          lambda: Node.process_detections(tracker(), []), None,
          passes="None")

    search_yaw = lambda: need("SEARCH_YAW_VEL", fm.SEARCH_YAW_VEL)  # noqa: E731
    check("motion_command SEARCH, last seen on the right: turn right in place",
          lambda: Node.motion_command(tracker(state=State.SEARCH, target_pos=0.2)),
          lambda: (-search_yaw(), 0.0),
          passes="(-SEARCH_YAW_VEL, 0.0): a right turn is a negative yaw")
    check("motion_command SEARCH, last seen on the left: turn left in place",
          lambda: Node.motion_command(tracker(state=State.SEARCH, target_pos=-0.2)),
          lambda: (search_yaw(), 0.0),
          passes="(+SEARCH_YAW_VEL, 0.0)")
    check("motion_command SEARCH, never seen yet (target_pos 0.0): still turn",
          lambda: abs(Node.motion_command(tracker(state=State.SEARCH, target_pos=0.0))[0]),
          lambda: search_yaw(), tol=SPEED_TOL,
          passes="(+SEARCH_YAW_VEL, 0.0): pick a side, don't stand still")
    check("motion_command TRACK, target right of centre: steer right, -KP * offset",
          lambda: Node.motion_command(tracker(target_pos=0.1))[0],
          lambda: -need("KP", fm.KP) * 0.1, tol=SPEED_TOL,
          passes="-KP * 0.1 (a negative yaw)")
    check("motion_command TRACK, target far away: walk at TRACK_FORWARD_VEL",
          lambda: Node.motion_command(tracker(target_pos=0.1, target_bottom=0.0))[1],
          lambda: need("TRACK_FORWARD_VEL", fm.TRACK_FORWARD_VEL), tol=SPEED_TOL,
          passes="TRACK_FORWARD_VEL")


def stop_bottom():
    return need("STOP_BOTTOM", target_lock.STOP_BOTTOM)


def step3():
    a = person(W / 2, 400)
    check("position_distance from a detection to its own position is 0",
          lambda: target_lock.position_distance(a, target_lock.box_position(a)), 0.0,
          tol=POSITION_TOL, passes="0.0")
    moved = person(W / 2 + 70, 400)
    check("position_distance is in image fractions (70 px sideways = 0.1)",
          lambda: target_lock.position_distance(moved, target_lock.box_position(a)), 0.1,
          tol=POSITION_TOL, passes="hypot(70 / 700, 0) = 0.1")

    check("remember stores the position (box_position)",
          lambda: locked_on(a).position, lambda: target_lock.box_position(a),
          passes="lock.position == (0.0, 0.437...)  (offset, centre y / 572)")

    left, right = person(200, 400), person(W / 2 + 20, 400)
    check("pick_target, new chase: the most centred person",
          lambda: pick([left, right], Lock), right,
          passes="the person at x=370, not the one at x=200")
    off_centre = person(500, 400)
    off_centre_next = person(530, 402)         # walked a little further right
    bystander = person(370, 400)               # more centred, but far from us
    check("pick_target: nearest to where the target was, not the most centred",
          lambda: pick([bystander, off_centre_next], lambda: locked_on(off_centre)), off_centre_next,
          passes="the person at x=530, 30 px from where we were, not the centred one at x=370")
    me = person(W / 2, 400)
    far_away = person(120, 380)
    check("pick_target: nobody within LOCK_GATE of where the target was -> None",
          lambda: pick([far_away], lambda: locked_on(me)), None,
          passes="None (the target is gone; a bystander does not take over)")
    near, nearer = person(W / 2 - 40, 400), person(W / 2 + 10, 400)
    check("pick_target: of two people near where the target was, the nearer",
          lambda: pick([near, nearer], lambda: locked_on(me)), nearer,
          passes="the person 10 px away, not the one 40 px away")

    def after_search():
        lk = locked_on(me)
        lk.position = None   # lost and searched: the spot is forgotten
        return lk
    centred = person(W / 2 + 30, 380)
    check("pick_target after a search (no position): the most centred person again",
          lambda: pick([far_away, centred], after_search), centred,
          passes="the person at x=380")

    # Wiring Step 3 into follow_me.py. Each check first runs the function it
    # needs, so it is skipped until that function is written.
    def gone():
        lock = locked_on(me)
        pick([far_away], lambda: lock)
        return Node.process_detections(tracker(lock=lock), [far_away])
    check("process_detections uses pick_target: only a far-off person in frame -> None",
          gone, None,
          passes="target = target_lock.pick_target(targets, self.lock), and None if it is None")

    def remembered():
        lock = Lock()
        target_lock.remember(Lock(), me)
        Node.process_detections(tracker(lock=lock), [me])
        return lock.position
    check("process_detections calls remember: the lock holds the target's position",
          remembered, lambda: target_lock.box_position(me),
          passes="target_lock.remember(self.lock, target)")


def step4():
    box = FakeDetection(300, 200, 400, 500)
    check("box_bottom is the bottom edge (centre y + height / 2)",
          lambda: boxes.box_bottom(box), 500, tol=1e-6,
          passes="a box from y=200 to y=500 -> 500")

    check("STOP_BOTTOM is measured: a bottom edge inside the 572 px frame",
          lambda: 0 < stop_bottom() <= H, True,
          passes="STOP_BOTTOM = <the px value you measured>, a number between 0 and 572")
    tol = target_lock.STOP_TOLERANCE
    ramp = target_lock.SLOWDOWN_PX
    check("approach_speed at STOP_BOTTOM: stand still",
          lambda: target_lock.approach_speed(stop_bottom(), 0.3), 0.0, tol=SPEED_TOL,
          passes="0.0")
    check("approach_speed inside STOP_TOLERANCE: still stand still",
          lambda: target_lock.approach_speed(stop_bottom() + 0.5 * tol, 0.3), 0.0, tol=SPEED_TOL,
          passes=f"0.0 for a bottom edge {0.5 * tol:.1f} px past STOP_BOTTOM")
    check("approach_speed far away: walk in at the top speed (max_forward)",
          lambda: target_lock.approach_speed(stop_bottom() - 200, 0.3), 0.3, tol=SPEED_TOL,
          passes=f"0.3 (0.3 * 200 / {ramp:g} = {0.3 * 200 / ramp:.2f}, capped at 0.3)")
    check("approach_speed a little far: proportional, max_forward * error / SLOWDOWN_PX",
          lambda: target_lock.approach_speed(stop_bottom() - 40, 0.3), 0.3 * 40 / ramp, tol=SPEED_TOL,
          passes=f"{0.3 * 40 / ramp:.3f} for a bottom edge 40 px short")
    check("approach_speed scales with max_forward: twice the top speed, twice as fast",
          lambda: target_lock.approach_speed(stop_bottom() - 40, 0.6), 0.6 * 40 / ramp, tol=SPEED_TOL,
          passes=f"{0.6 * 40 / ramp:.3f} for 40 px short at max_forward 0.6")
    check("approach_speed too close: back away (negative)",
          lambda: target_lock.approach_speed(stop_bottom() + 30, 0.3) < 0, True,
          passes=f"{-0.3 * 30 / ramp:.3f} for a bottom edge 30 px past")
    check("approach_speed right on top of us: back away at most MAX_BACKUP_VEL",
          lambda: target_lock.approach_speed(stop_bottom() + 200, 0.3), -target_lock.MAX_BACKUP_VEL,
          tol=SPEED_TOL, passes=f"-{target_lock.MAX_BACKUP_VEL} (clamped)")

    # Wiring Step 4 into follow_me.py.
    me = person(W / 2, 400)
    check("process_detections stores the target's box_bottom in self.target_bottom",
          lambda: (boxes.box_bottom(me), processed([me]).target_bottom)[1], 400, tol=1e-6,
          passes="self.target_bottom = box_bottom(target)")
    check("motion_command TRACK at STOP_BOTTOM: stand still (approach_speed)",
          lambda: (target_lock.approach_speed(stop_bottom(), 0.3),
                   Node.motion_command(tracker(target_bottom=stop_bottom()))[1])[1], 0.0, tol=SPEED_TOL,
          passes="target_lock.approach_speed(self.target_bottom, TRACK_FORWARD_VEL) -> 0.0")


def colored_lock(det):
    """A lock that has just been following `det`, color and all (uses your remember_color)."""
    lock = locked_on(det)
    target_lock.remember_color(lock, det)
    return lock


def pick_c(targets, lock_fn):
    """pick_target_by_color's choice (the lock is built lazily, so skips work)."""
    return target_lock.pick_target_by_color(targets, lock_fn())


def step5():
    a = person(W / 2, 400, RED)
    check("color_distance of the same shirt is 0",
          lambda: target_lock.color_distance(a, np.array(RED, float)), 0.0, tol=COLOR_TOL,
          passes="0.0")
    check("color_distance with no color on the detection is None (no evidence)",
          lambda: target_lock.color_distance(person(W / 2, 400), np.array(RED, float)), None,
          passes="None for a detection with id == ''")
    check("color_distance with nothing remembered is None",
          lambda: target_lock.color_distance(a, None), None,
          passes="None when lock.color is None")

    check("remember_color on first sight stores the color",
          lambda: list(colored_lock(a).color), list(RED),
          passes="lock.color == [200, 40, 40]")

    def blended_error():
        lk = colored_lock(a)
        target_lock.remember_color(lk, person(W / 2, 400, REDDISH))
        want = (1 - target_lock.REID_BLEND) * np.array(RED) + target_lock.REID_BLEND * np.array(REDDISH)
        return float(np.abs(lk.color - want).max())
    check("remember_color after that blends the color by REID_BLEND",
          blended_error, 0.0, tol=COLOR_TOL,
          passes="0.9 * [200, 40, 40] + 0.1 * [185, 55, 50] = [198.5, 41.5, 41]")

    left, right = person(200, 400, BLUE), person(W / 2 + 20, 400, RED)
    check("pick_target_by_color, new chase: the most centred person",
          lambda: pick_c([left, right], Lock), right,
          passes="the person at x=370, not the one at x=200")
    me = person(W / 2, 400, RED)
    me_next = person(W / 2 + 15, 402, REDDISH)
    far_red = person(100, 300, RED)
    check("pick_target_by_color: nearest to the last position wins, even over a better color far away",
          lambda: pick_c([far_red, me_next], lambda: colored_lock(me)), me_next,
          passes="the reddish person 15 px from where we were")
    stranger_here = person(W / 2 + 10, 400, BLUE)
    check("pick_target_by_color: a stranger alone in our spot does not steal the lock",
          lambda: pick_c([stranger_here], lambda: colored_lock(me)), None,
          passes="None (blue is too far from red)")
    crossing_me = person(W / 2 + 25, 400, REDDISH)
    crossing_them = person(W / 2 - 10, 400, BLUE)   # nearer to where we were
    check("pick_target_by_color: two people crossing, the nearer one a stranger: ours wins",
          lambda: pick_c([crossing_them, crossing_me], lambda: colored_lock(me)), crossing_me,
          passes="the reddish one")
    elsewhere = person(120, 350, REDDISH)
    check("pick_target_by_color: stranger in our spot, us elsewhere: re-acquire us by color",
          lambda: pick_c([stranger_here, elsewhere], lambda: colored_lock(me)), elsewhere,
          passes="the reddish person at x=120")

    def after_search():
        lk = colored_lock(me)
        lk.position = None   # lost and searched: the spot is forgotten, the color is not
        return lk
    back = person(600, 350, REDDISH)
    check("pick_target_by_color after a search (no position): re-acquire by color anywhere",
          lambda: pick_c([person(300, 380, BLUE), back], after_search), back,
          passes="the reddish person at x=600")
    no_color_me = person(W / 2, 400)
    a2, b2 = person(W / 2 + 10, 400), person(W / 2 - 40, 400)
    check("pick_target_by_color with no masks: nearest position decides",
          lambda: pick_c([b2, a2], lambda: colored_lock(no_color_me)), a2,
          passes="the colorless person 10 px away, not the one 40 px away")


def step6():
    # The person we are chasing: mid-frame, bottom edge at 272.
    target = person(W / 2, H - 300)
    beyond = FakeDetection(W / 2 - 40, 80, W / 2 + 40, H - 350)
    check("find_blocking_obstacle: an object beyond the target does not block",
          lambda: avoidance.find_blocking_obstacle(target, [beyond]), None,
          passes="its bottom (222) is above the target's (272) -> None")
    between_far = FakeDetection(W / 2 - 40, 80, W / 2 + 40, H - 280)
    check("find_blocking_obstacle: between us and the target but still far does not block",
          lambda: avoidance.find_blocking_obstacle(target, [between_far]), None,
          passes="bottom 292 is 280 px from the image bottom (> 250) -> None")
    off_side = FakeDetection(30, 200, 150, H - 40)
    check("find_blocking_obstacle: near but off to a side does not block",
          lambda: avoidance.find_blocking_obstacle(target, [off_side]), None,
          passes="offset -0.37 is outside the 0.15 band -> None")
    in_the_way = FakeDetection(W / 2 - 50, 200, W / 2 + 30, H - 100)
    check("find_blocking_obstacle: near, centred and between blocks (returns its offset)",
          lambda: avoidance.find_blocking_obstacle(target, [in_the_way]),
          lambda: boxes.box_offset(in_the_way), tol=1e-6,
          passes="-0.014 (the blocker's box_offset), not None")
    nearer = FakeDetection(W / 2 - 20, 300, W / 2 + 60, H - 40)
    check("find_blocking_obstacle: of two blockers, the nearest wins",
          lambda: avoidance.find_blocking_obstacle(target, [in_the_way, nearer]),
          lambda: boxes.box_offset(nearer), tol=1e-6,
          passes="+0.029 (bottom 532 beats bottom 472)")

    now = 100.0
    cooldown = fm.AVOID_COOLDOWN
    check("ready_to_avoid: nothing in the way -> False",
          lambda: Node.ready_to_avoid(tracker(obstacle_pos=None), FakeTime(now)), False,
          passes="False")
    check("ready_to_avoid: something in the way, walking in, no detour yet -> True",
          lambda: Node.ready_to_avoid(tracker(obstacle_pos=0.0), FakeTime(now)), True,
          passes="True")
    check("ready_to_avoid: something in the way but we are at the stop distance -> False",
          lambda: Node.ready_to_avoid(tracker(obstacle_pos=0.0, target_bottom=stop_bottom()), FakeTime(now)),
          False, passes="False (approach_speed is 0: a detour would walk into the person)")
    check("ready_to_avoid: a detour ended less than AVOID_COOLDOWN ago -> False",
          lambda: Node.ready_to_avoid(tracker(obstacle_pos=0.0, avoid_done_time=FakeTime(now - 0.5 * cooldown)),
                                      FakeTime(now)),
          False, passes=f"False ({0.5 * cooldown:g} s < AVOID_COOLDOWN = {cooldown:g} s)")
    check("ready_to_avoid: the last detour ended long enough ago -> True",
          lambda: Node.ready_to_avoid(tracker(obstacle_pos=0.0, avoid_done_time=FakeTime(now - 2 * cooldown)),
                                      FakeTime(now)),
          True, passes="True")


SECTIONS = {
    "step2": ("Step 2 - follow", step2),
    "step3": ("Step 3 - follow me, specifically", step3),
    "step4": ("Step 4 - follow me, at a polite distance", step4),
}
OPTIONAL_SECTIONS = {
    "step5": ("Optional Step 5 - follow me, very specifically", step5),
    "step6": ("Optional Step 6 - something in the way", step6),
}

if __name__ == "__main__":
    run(SECTIONS, "lab/test.py", optional=OPTIONAL_SECTIONS)
