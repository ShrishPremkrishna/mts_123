"""Optional Step 6: is something in the way, and which way around it? Pure functions, no ROS.

You write one function here, find_blocking_obstacle. The rest of the file is
provided: read it, because Step 6's questions are about it.

The one big idea
----------------
The same one as Step 4: the bottom edge of a box is a distance sensor
(boxes.box_bottom). Everything in this file is built out of it. A box whose
bottom edge sits low in the frame is close to the robot; one whose bottom
edge sits up near the horizon is far away. (When does it lie to you? Anything
not standing on the floor -- a tabletop, a cat on a couch -- and anything cut
off by the frame edge. Keep that in mind when the robot does something
confusing.)
"""

from boxes import IMAGE_HEIGHT, IMAGE_WIDTH, box_bottom, box_offset

# --- When is something "in the way"? ---------------------------------------
# Being between us and the target is not enough to justify a detour: it also
# has to be close (bottom edge near the image bottom) and roughly ahead
# (near the middle of the frame).
OBSTACLE_BOTTOM_MARGIN = 250   # px: bottom edge must be within this of the image bottom
OBSTACLE_CENTER_BAND = 0.15    # |box_offset|: how far off-centre still counts as in the way
OBSTACLE_MIN_SCORE = 0.3       # ignore shaky detections; a false detour is expensive

# --- Which way to go around? ------------------------------------------------
# A question about the whole frame, not just the one thing in the way: every
# non-target box is tallied onto the side of the image it sits on, and the
# robot turns toward whichever side carries less. Nearer objects weigh more --
# they are the ones it would walk into first.
SIDE_WEIGHT_FAR = 0.5          # weight of an object up near the horizon
SIDE_WEIGHT_NEAR = 2.0         # ... and of one right at the robot's feet
SIDE_SCORE_TIE = 0.25          # closer than this and the two sides count as even


def find_blocking_obstacle(target, others):
    """Offset of the nearest object standing between us and the target, or None.

    Optional Step 6 (something in the way) -------------------------------------
    target:     the Detection2D we are chasing.
    others:     list of Detection2D: everything else in the frame (other
                classes, confidence at least OBSTACLE_MIN_SCORE). May be empty.
    Returns:    float or None: the box_offset (-0.5..+0.5) of the nearest
                blocking detection -- its offset, not the detection itself --
                or None when nothing blocks. With the target's bottom edge at
                272: a centred chair with bottom 472 -> its offset, -0.014;
                one with bottom 222 (beyond the target) -> None
    Used by:    follow_me.py (provided code) stores it in self.obstacle_pos;
                your ready_to_avoid reads it.
    Test:       python3 lab/test.py step6
    """
    # ========== OPTIONAL — YOUR CODE HERE: Step 6: find_blocking_obstacle ==========
    # An object blocks only if all three hold:
    #   1. between us and the target: its bottom edge is lower in the image
    #      (bigger) than the target's;
    #   2. close: its bottom edge is within OBSTACLE_BOTTOM_MARGIN of the
    #      image bottom (IMAGE_HEIGHT);
    #   3. ahead: abs(its box_offset) is at most OBSTACLE_CENTER_BAND.
    # Of the ones that block, return the box_offset of the nearest (the
    # biggest bottom edge). None if nothing blocks.
    target_bottom = box_bottom(target)
    blocking = [d for d in others
                if box_bottom(d) > target_bottom
                and box_bottom(d) >= IMAGE_HEIGHT - OBSTACLE_BOTTOM_MARGIN
                and abs(box_offset(d)) <= OBSTACLE_CENTER_BAND]
    return box_offset(max(blocking, key=box_bottom)) if blocking else None
    # ===============================================================================


def box_left_fraction(detection):
    """Fraction of a box's width in the left half of the image, 0.0..1.0.

    A box straddling the middle really is on both sides, so it is split
    between them instead of being forced onto whichever side its centre lands.
    """
    half = detection.bbox.size_x / 2.0
    left_edge = detection.bbox.center.position.x - half
    width = 2.0 * half
    middle = IMAGE_WIDTH / 2.0
    if width <= 0:
        return 1.0 if left_edge < middle else 0.0
    return min(max(middle - left_edge, 0.0), width) / width


def obstacle_weight(detection):
    """How much a detection counts against its side: SIDE_WEIGHT_FAR at the
    top of the frame, SIDE_WEIGHT_NEAR at the bottom, linear in between."""
    nearness = min(max(box_bottom(detection) / IMAGE_HEIGHT, 0.0), 1.0)
    return SIDE_WEIGHT_FAR + (SIDE_WEIGHT_NEAR - SIDE_WEIGHT_FAR) * nearness


def measure_crowding(others):
    """How much is in each half of the frame, as a (left, right) pair.

    Every detection that is not the target counts, wherever it is -- not just
    the one in the way -- split between the halves by box_left_fraction and
    weighted by obstacle_weight, so a chair at the robot's feet outweighs
    three of them across the room. The lighter side is the one worth
    detouring into.
    """
    left = right = 0.0
    for det in others:
        weight = obstacle_weight(det)
        left_fraction = box_left_fraction(det)
        left += weight * left_fraction
        right += weight * (1.0 - left_fraction)
    return left, right


def choose_side(crowding, obstacle_pos, target_pos):
    """Which way to detour: +1.0 for left, -1.0 for right, and a reason for the log.

    The emptier half of the frame wins -- walking around one chair into
    another is no better than standing still. Only when the two halves come
    out even does the blocking obstacle, and then the target, break the tie.
    """
    left, right = crowding
    if abs(left - right) >= SIDE_SCORE_TIE:
        return (1.0 if left < right else -1.0), f'crowding L {left:.2f} / R {right:.2f}'
    if abs(obstacle_pos) > 0.02:
        # Step off the obstacle's side of the line: it is right of centre, we go left.
        return (1.0 if obstacle_pos > 0 else -1.0), f'sides even, obstacle at {obstacle_pos:+.2f}'
    if target_pos != 0.0:
        # Go round on the side the target is on: the shorter way back to it.
        return (-1.0 if target_pos > 0 else 1.0), f'sides even, target at {target_pos:+.2f}'
    return 1.0, 'nothing to choose between the sides'
