"""Follow *me*: which person is mine, and how close to stand. Pure functions, no ROS.

Detections in, decisions out, so ``python3 lab/test.py`` checks all of it at
your desk.

Three jobs, one per step
------------------------
Step 3. **Which person is mine?** To YOLO every person is just class
   ``person``, so "most centred" hops between people the moment two are in
   frame. But detections arrive about ten times a second, and between two
   frames (~0.1 s) a person barely moves in the image -- the same small-motion
   assumption optical flow is built on. So the person nearest to where the
   target just was is almost certainly the target. Nobody near that spot
   means the target is gone, not that a bystander should take over.

Step 4. **How close?** The bottom edge of the box is a distance sensor
   (boxes.box_bottom). Pick the bottom edge you want -- STOP_BOTTOM, measured
   with a person standing at a comfortable distance -- and drive the error to
   zero: walk in when they are far, stop when they are there, step back when
   they come too close.

Optional Step 5. **Which person is mine, even after losing them?** Position
   alone forgets the target the moment the robot has to search. Each
   detection also carries the mean color of its segmentation mask
   (``detection.id``, ``"r,g,b"``) -- mostly the color of their clothes. A
   person whose color is far from the remembered one is a stranger, wherever
   they stand; and when the target vanished and reappears somewhere else, the
   color finds them again. Switch it on with USE_COLOR.
"""

import numpy as np

from boxes import IMAGE_HEIGHT, box_bottom, box_offset, detection_color

# --- Step 3: keeping the lock -----------------------------------------------
# How far (in the units of position_distance: fractions of the image) a
# person may be from where the target was last frame and still count as
# "nearby". Generous enough for the robot's own turning, tight enough that a
# person across the room is not a candidate.
LOCK_GATE = 0.12

# --- Step 4: keeping the distance -------------------------------------------
# Box bottom edge (reference-frame px) with a person standing at the
# distance we want to keep. Measure it on the robot: stand where you would
# like Pupper to stop, save the viewer's "Live feed" image, and scale the
# y of the box's bottom edge to the 572 px frame:
#     STOP_BOTTOM = y_bottom / screenshot_height * IMAGE_HEIGHT
# ========== YOUR CODE HERE: Step 4: measure the stop distance ==========
STOP_BOTTOM = 414  # px -- measure it (see above)
# =======================================================================
# Closer to STOP_BOTTOM than this and we stand still: the box jitters by a
# few pixels every frame, and a robot that shuffles back and forth to chase
# that jitter looks nervous.
STOP_TOLERANCE = 15.0
# How far short of STOP_BOTTOM (px) the robot starts slowing down. Further
# than this it walks at full speed (follow_me.py's TRACK_FORWARD_VEL); closer,
# its speed falls in proportion to the error, reaching 0 at STOP_BOTTOM. So
# raising TRACK_FORWARD_VEL makes the whole approach faster, not just the
# far end of it. If a fast robot overshoots and bumps into you, raise this.
SLOWDOWN_PX = 120.0
# Fastest the robot backs away from someone too close (m/s). Slower than
# walking forward: it cannot see where it is backing into.
MAX_BACKUP_VEL = 0.2

# --- Optional Step 5: telling people apart by color -------------------------
# Off unless you do Optional Step 5: with it off, the lock is position only.
USE_COLOR = False  # Optional Step 5: True
# Mask colors further apart than this (RGB distance, 0..441) are different
# people. Under the camera's fixed white balance the same shirt stays within
# a few tens of this from frame to frame.
REID_MAX_DIST = 90.0
# Per-frame blend of the remembered color toward the target's current color,
# so the memory follows a slow change in lighting instead of being shaken off
# by it.
REID_BLEND = 0.1

# --- Engineering: when to stop instead of guessing ---------------------------
# Two people we cannot tell apart, with boxes this much overlapped on the
# target's spot, are about to swap: from here on, the lock cannot say which is
# ours. 0.3 means 30% of the narrower box is covered.
AMBIGUOUS_OVERLAP = 0.3
# A box this many times wider than the target's recent box is somebody nearer,
# however close to the target's spot it lands: a stranger walking in front.
# Width, not height: a table that hides the legs changes a box's height only.
MAX_WIDTH_RATIO = 1.5
# Per-frame decay of the remembered width, so one wide pose is soon forgotten.
WIDTH_MEMORY = 0.995
# Nobody may be walked into: anyone inside the stop distance, within this much
# of straight ahead (box_offset), blocks forward motion. They need not be the target.
IN_THE_WAY_BAND = 0.15


class Lock:
    """What we remember about the target between frames."""

    def __init__(self):
        self.position = None  # (box_offset, centre y / IMAGE_HEIGHT) last frame, or None
        self.color = None     # Optional Step 5: remembered mask color, a numpy array (r, g, b), or None
        self.seen = False     # has this chase had a target yet?
        self.gave_up = False  # the target is unknown until a new chase
        self.width = None     # recent widest box width (px), or None

    def reset(self):
        """A new chase: forget everything."""
        self.position = None
        self.color = None
        self.seen = False
        self.gave_up = False
        self.width = None

    def give_up(self):
        """Stop guessing: no target until a new chase."""
        self.position = None
        self.gave_up = True


def box_position(detection):
    """Where a detection is, as (offset, centre y / IMAGE_HEIGHT).

    Both are fractions of the image: offset -0.5..+0.5, centre y 0..1.
    """
    return (box_offset(detection), detection.bbox.center.position.y / IMAGE_HEIGHT)


# ======================================================== Step 3: which person
def position_distance(detection, position):
    """How far a detection is from a remembered position.

    Step 3 (follow me, specifically) -------------------------------------------
    detection:  one Detection2D.
    position:   an (offset, centre y fraction) tuple, as box_position returns
                and lock.position stores.
    Returns:    float >= 0, in fractions of the image (not pixels): 0.0 for
                the detection's own position; 70 px sideways -> 0.1
    Used by:    pick_target, against LOCK_GATE.
    Test:       python3 lab/test.py step3
    """
    # ========== YOUR CODE HERE: Step 3: position_distance ==========
    # The straight-line (Euclidean) distance between box_position(detection)
    # and position. Both are fractions of the image, so compare them directly.
    #prev_x, prev_y = position
    #new_x, new_y = box_position(detection)
    return np.linalg.norm(np.subtract(position, box_position(detection)))
    # ===============================================================


def pick_target(targets, lock):
    """Which of this frame's people is *our* target, or None.

    Step 3 (follow me, specifically) -------------------------------------------
    targets:    list of Detection2D, every detection of the tracked class in
                this frame (at least one).
    lock:       a Lock, what we remember about our target:
                  lock.position  (offset, centre y fraction) last frame, or
                                 None (a new chase, or the robot just searched)
                Only read it here: remember updates it afterwards.
    Returns:    one of `targets`, or None when nobody is within LOCK_GATE of
                where the target was (it is gone; a bystander does not take
                over).
    Used by:    follow_me.py's process_detections, in place of "most centred".
    Test:       python3 lab/test.py step3
    """
    if not targets:
        return None
    if USE_COLOR:
        return pick_target_by_color(targets, lock)  # Optional Step 5
    # ========== YOUR CODE HERE: Step 3: pick_target ==========
    # Two rules, in order:
    #   1. No position remembered (lock.position is None): the most
    #      centred person, as in Step 2.
    #   2. Otherwise: of the people within LOCK_GATE of lock.position,
    #      the nearest one. Nobody that close? None.
    # Tip: min(people, key=lambda d: position_distance(d, lock.position))
    # is the nearest of a non-empty list.
    pos = lock.position
    if (pos is None):
        return min(targets, key=lambda d: abs(box_offset(d)))
    nearby = [d for d in targets if position_distance(d, pos) <= LOCK_GATE]
    if (not nearby):
        return None
    return min(nearby, key=lambda d: position_distance(d, pos))
    
    # =========================================================


def remember(lock, detection):
    """Update the lock after choosing ``detection`` as the target.

    Step 3 (follow me, specifically) -------------------------------------------
    lock:       the Lock to update, in place.
    detection:  the Detection2D pick_target just chose (never None here).
    Returns:    nothing. Afterwards lock.position == box_position(detection).
    Test:       python3 lab/test.py step3
    """
    # ========== YOUR CODE HERE: Step 3: remember ==========
    # lock.position = where the detection is (box_position).
    lock.position = box_position(detection)
    # ======================================================
    lock.seen = True
    width = detection.bbox.size_x
    lock.width = width if lock.width is None else max(width, WIDTH_MEMORY * lock.width)
    if USE_COLOR:
        remember_color(lock, detection)  # Optional Step 5


# ======================================================== Engineering: recovery
def may_reacquire(lock):
    """May this frame's people be considered as the target at all?

    A new chase, and a lock that still knows the target's spot, are fine. After
    a loss the spot means nothing, so only identity evidence may bring the
    target back. That needs a remembered color; pick_target_by_color then
    accepts a color match and nobody else. Without color, "the most centred
    person" is a guess, and following the wrong person is worse than stopping.
    """
    if lock.gave_up:
        return False
    if lock.position is not None or not lock.seen:
        return True
    return USE_COLOR and lock.color is not None


def plausible_size(people, lock):
    """The people whose box is not too wide to be the target.

    Only while the lock knows the target's spot: after a search the robot may
    have walked, and the target's size is no longer a guide.
    """
    if lock.position is None or lock.width is None:
        return people
    return [d for d in people if d.bbox.size_x <= lock.width * MAX_WIDTH_RATIO]


def person_in_the_way(people, target):
    """Is someone other than the target inside the stop distance, ahead of us?"""
    return any(d is not target and box_bottom(d) >= STOP_BOTTOM and abs(box_offset(d)) <= IN_THE_WAY_BAND
               for d in people)


def box_overlap(a, b):
    """Fraction of the narrower box's width that the other box covers, 0..1."""
    a_left, a_right = box_edges(a)
    b_left, b_right = box_edges(b)
    shared = min(a_right, b_right) - max(a_left, b_left)
    return max(shared, 0.0) / min(a_right - a_left, b_right - b_left)


def box_edges(detection):
    half = detection.bbox.size_x / 2
    return detection.bbox.center.position.x - half, detection.bbox.center.position.x + half


def ambiguous(targets, lock):
    """Are two people we cannot tell apart overlapped on the target's spot?

    A crossing hides one person behind the other, and the nearest box to the
    lock can then be the wrong one. Different mask colors tell them apart, so
    this fires only without that evidence.
    """
    if lock.position is None:
        return False
    close = [d for d in targets if position_distance(d, lock.position) <= LOCK_GATE
             and not (USE_COLOR and is_stranger(d, lock))]
    close.sort(key=lambda d: position_distance(d, lock.position))
    return len(close) >= 2 and box_overlap(close[0], close[1]) >= AMBIGUOUS_OVERLAP


def is_stranger(detection, lock):
    """True when this person's mask color rules them out."""
    distance = color_distance(detection, lock.color)
    return distance is not None and distance > REID_MAX_DIST


# ======================================================== Step 4: how close
def approach_speed(target_bottom, max_forward):
    """Forward speed (m/s) that holds the target at STOP_BOTTOM.

    Step 4 (polite distance) ---------------------------------------------------
    target_bottom:  float, px: box_bottom of the target. Bigger = closer.
    max_forward:    float, m/s: the top speed (follow_me.py passes
                    TRACK_FORWARD_VEL).
    Returns:        float, m/s: positive walks in, negative backs away, 0.0
                    inside STOP_TOLERANCE. Never faster than max_forward
                    forward or MAX_BACKUP_VEL backward. With max_forward 0.3:
                    at STOP_BOTTOM -> 0.0; 40 px short -> 0.1;
                    200 px short -> 0.3 (capped); bottom 572 -> -0.2
    Used by:        the TRACK state's forward speed in follow_me.py.
    Test:           python3 lab/test.py step4
    """
    # ========== YOUR CODE HERE: Step 4: approach_speed ==========
    # error = STOP_BOTTOM - target_bottom (positive = target is too far:
    # its bottom edge is above where we want it). Inside STOP_TOLERANCE,
    # stand still. Otherwise a proportional command that reaches
    # max_forward at SLOWDOWN_PX of error, max_forward * error / SLOWDOWN_PX,
    # clamped to [-MAX_BACKUP_VEL, max_forward].
    error = STOP_BOTTOM-target_bottom
    if (abs(error) < STOP_TOLERANCE):
        return 0
    if (error > 0):
        return min(max_forward, max_forward * error / SLOWDOWN_PX)
    return max(-MAX_BACKUP_VEL, max_forward * error / SLOWDOWN_PX)
    
    # ============================================================


# ======================================================== Optional Step 5: color
def color_distance(detection, color):
    """How different a detection's mask color is from a remembered one.

    Optional Step 5 (follow me, very specifically) ------------------------------
    detection:  one Detection2D; boxes.detection_color(detection) gives its
                color as an RGB array, or None when it has none.
    color:      the remembered color (lock.color): an RGB array, or None.
    Returns:    float >= 0, the RGB (Euclidean) distance: 0.0 for the same
                shirt. None when either color is missing: that is "no
                evidence", which is not the same thing as "a match".
    Used by:    pick_target_by_color, against REID_MAX_DIST.
    Test:       python3 lab/test.py step5
    """
    # ========== OPTIONAL — YOUR CODE HERE: Step 5: color_distance ==========
    # Return None if either color is missing, else the distance between them
    # (np.linalg.norm of the difference).
    observed = detection_color(detection)
    if observed is None or color is None:
        return None
    remembered = np.asarray(color, dtype=float)
    if remembered.shape != (3,) or not np.all(np.isfinite(remembered)):
        return None
    return float(np.linalg.norm(observed - remembered))
    # =======================================================================


def pick_target_by_color(targets, lock):
    """pick_target, with color: which of this frame's people is ours, or None.

    Optional Step 5 (follow me, very specifically) ------------------------------
    targets:    list of Detection2D, every detection of the tracked class in
                this frame (at least one).
    lock:       a Lock: lock.position as in Step 3, and
                  lock.color  the target's mask color, an RGB array, or None
    Returns:    one of `targets`, or None when they are all strangers.
                A stranger is a person whose color_distance to lock.color is
                more than REID_MAX_DIST. (A missing color is no evidence:
                that person is not a stranger.)
    Used by:    pick_target, when USE_COLOR is on.
    Test:       python3 lab/test.py step5
    """
    # ========== OPTIONAL — YOUR CODE HERE: Step 5: pick_target_by_color ==========
    # Three rules, in order:
    #   1. Nothing remembered at all (a new chase): the most centred person.
    #   2. Where the target was is known (lock.position): of the people
    #      within LOCK_GATE of it who are not strangers, the nearest one.
    #   3. Otherwise, the person whose color is closest to lock.color, if
    #      it is within REID_MAX_DIST; else None (only strangers).
    if not targets:
        return None
    if lock.position is None and lock.color is None:
        return min(targets, key=lambda d: abs(box_offset(d)))
    distances = [(d, color_distance(d, lock.color)) for d in targets]
    if lock.position is not None:
        nearby = [d for d, distance in distances
                  if position_distance(d, lock.position) <= LOCK_GATE
                  and (distance is None or distance <= REID_MAX_DIST)]
        if nearby:
            return min(nearby, key=lambda d: position_distance(d, lock.position))
    # Without position evidence, a missing color cannot establish identity.
    matches = [(d, distance) for d, distance in distances
               if distance is not None and distance <= REID_MAX_DIST]
    return min(matches, key=lambda pair: pair[1])[0] if matches else None
    # =============================================================================


def remember_color(lock, detection):
    """Blend the chosen detection's mask color into the lock's memory.

    Optional Step 5 (follow me, very specifically) ------------------------------
    lock:       the Lock to update, in place.
    detection:  the Detection2D just chosen as the target.
    Returns:    nothing. Afterwards lock.color is the detection's color on
                first sight, then (1 - REID_BLEND) * old + REID_BLEND * new
                after that, so it follows slow lighting changes. A detection
                with no color leaves lock.color as it was.
                red [200, 40, 40], then reddish [185, 55, 50] -> [198.5, 41.5, 41]
    Used by:    remember, when USE_COLOR is on.
    Test:       python3 lab/test.py step5
    """
    # ========== OPTIONAL — YOUR CODE HERE: Step 5: remember_color ==========
    # 1. color = detection_color(detection). If it is None, stop here.
    # 2. First sight (lock.color is None): store it. Otherwise blend:
    #    lock.color = (1 - REID_BLEND) * lock.color + REID_BLEND * color
    color = detection_color(detection)
    if color is None:
        return
    lock.color = (color.copy() if lock.color is None else
                  (1.0 - REID_BLEND) * lock.color + REID_BLEND * color)
    # =======================================================================
