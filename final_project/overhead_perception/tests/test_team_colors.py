import math
import unittest
from typing import NamedTuple, Optional, Tuple

import cv2
import numpy as np

from fieldcam.team_colors import (
    NOSE_OFFSET_L,
    NOSE_PATCH_SIDE_L,
    TEAM_PATCH_SIDE_L,
    TeamColorPose,
    detect_team_colors,
)

L = 160.0
W, H = 640, 480

BLUE = (200, 60, 20)
RED = (30, 30, 210)
LIME = (0, 255, 170)
BODY = (60, 60, 60)
GRASS = (40, 130, 50)
PINK = (147, 20, 255)
SKIN = (130, 160, 210)


class Robot(NamedTuple):
    center: Tuple[float, float]
    heading_deg: float
    color: Tuple[int, int, int]
    nose: bool = True
    disk: bool = False
    length: float = L


def field(seed=0):
    rng = np.random.default_rng(seed)
    img = np.empty((H, W, 3), np.float64)
    img[:] = GRASS
    img += rng.normal(0.0, 8.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def unit(heading_deg):
    t = math.radians(heading_deg)
    return math.cos(t), math.sin(t)


def offset(center, heading_deg, distance):
    fx, fy = unit(heading_deg)
    return center[0] + fx * distance, center[1] + fy * distance


def fill_rect(img, center, heading_deg, along, across, color):
    fx, fy = unit(heading_deg)
    sx, sy = -fy, fx
    corners = [
        (
            center[0] + fx * along / 2 * i + sx * across / 2 * j,
            center[1] + fy * along / 2 * i + sy * across / 2 * j,
        )
        for i, j in ((1, 1), (1, -1), (-1, -1), (-1, 1))
    ]
    pts = np.round(np.array(corners) * 16).astype(np.int32)
    cv2.fillPoly(img, [pts], color, lineType=cv2.LINE_8, shift=4)


def fill_disk(img, center, diameter, color):
    c = (int(round(center[0] * 16)), int(round(center[1] * 16)))
    cv2.circle(img, c, int(round(diameter / 2 * 16)), color, -1, cv2.LINE_8, 4)


def fill_patch(img, center, heading_deg, side, color, disk):
    if disk:
        fill_disk(img, center, side, color)
    else:
        fill_rect(img, center, heading_deg, side, side, color)


def draw(img, *robots):
    """Draw all bodies first so no body covers another robot's patches."""
    for r in robots:
        fill_rect(img, r.center, r.heading_deg, r.length, 0.45 * r.length, BODY)
    for r in robots:
        fill_patch(
            img, r.center, r.heading_deg, TEAM_PATCH_SIDE_L * r.length, r.color, r.disk
        )
        if r.nose:
            fill_patch(
                img,
                nose_of(r),
                r.heading_deg,
                NOSE_PATCH_SIDE_L * r.length,
                LIME,
                r.disk,
            )
    return img


def nose_of(robot):
    return offset(robot.center, robot.heading_deg, NOSE_OFFSET_L * robot.length)


def lime_square(img, center, side=NOSE_PATCH_SIDE_L * L):
    fill_rect(img, center, 0.0, side, side, LIME)


def angle_error_deg(a_rad, b_deg):
    d = math.degrees(a_rad) - b_deg
    return abs((d + 180.0) % 360.0 - 180.0)


class TeamColorTest(unittest.TestCase):
    def assertPose(
        self,
        pose: Optional[TeamColorPose],
        robot: Robot,
        *,
        heading: bool = True,
        center_tol: float = 1.5,
        heading_tol_deg: float = 3.0,
    ):
        self.assertIsNotNone(pose)
        self.assertLessEqual(math.dist(pose.center_px, robot.center), center_tol)
        self.assertGreaterEqual(pose.score, 0.0)
        self.assertLessEqual(pose.score, 1.0)
        if heading:
            self.assertIsNotNone(pose.front_px)
            self.assertLessEqual(
                angle_error_deg(pose.heading_rad, robot.heading_deg), heading_tol_deg
            )
        else:
            self.assertIsNone(pose.front_px)
            self.assertIsNone(pose.heading_rad)

    # Rotations and appearance

    def test_rotations_square_and_disk(self):
        for disk in (False, True):
            for heading in range(0, 360, 30):
                with self.subTest(disk=disk, heading=heading):
                    striker = Robot((220, 240), heading, BLUE, disk=disk)
                    keeper = Robot((450, 240), -heading + 15, RED, disk=disk)
                    poses = detect_team_colors(draw(field(heading), striker, keeper), L)
                    self.assertEqual(set(poses), {"striker", "keeper"})
                    self.assertPose(poses["striker"], striker)
                    self.assertPose(poses["keeper"], keeper)

    def test_blurred_frame(self):
        for heading in (0, 45, 135, 250):
            with self.subTest(heading=heading):
                striker = Robot((220, 240), heading, BLUE)
                keeper = Robot((450, 240), heading + 180, RED)
                img = cv2.GaussianBlur(draw(field(), striker, keeper), (5, 5), 0)
                poses = detect_team_colors(img, L)
                self.assertPose(poses.get("striker"), striker)
                self.assertPose(poses.get("keeper"), keeper)

    def test_body_length_mismatch_within_tolerance(self):
        for drawn in (140.0, 180.0):
            with self.subTest(drawn=drawn):
                striker = Robot((220, 240), 70, BLUE, length=drawn)
                keeper = Robot((450, 240), 200, RED, length=drawn)
                poses = detect_team_colors(draw(field(), striker, keeper), L)
                self.assertPose(poses.get("striker"), striker)
                self.assertPose(poses.get("keeper"), keeper)

    def test_score_is_high_for_clean_patches(self):
        striker = Robot((320, 240), 0, BLUE)
        poses = detect_team_colors(draw(field(), striker), L)
        self.assertPose(poses.get("striker"), striker)
        self.assertGreater(poses["striker"].score, 0.5)

    def test_empty_field(self):
        self.assertEqual(detect_team_colors(field(), L), {})

    # Distractors

    def test_distractors_grass_skin_ball_shirts_tape_confetti(self):
        img = field(3)
        striker = Robot((200, 300), 30, BLUE)
        keeper = Robot((440, 300), 200, RED)
        draw(img, striker, keeper)
        fill_disk(img, (320, 130), TEAM_PATCH_SIDE_L * L, PINK)  # ball
        cv2.ellipse(img, (560, 110), (60, 30), 20, 0, 360, SKIN, -1)  # hand
        cv2.rectangle(img, (0, 380), (150, 479), RED, -1)  # red shirt
        cv2.rectangle(img, (500, 400), (639, 479), BLUE, -1)  # blue shirt
        cv2.line(img, (20, 20), (300, 20), LIME, 4)  # lime tape
        rng = np.random.default_rng(7)
        for _ in range(30):  # tiny confetti of every team colour
            x, y = int(rng.integers(0, W - 4)), int(rng.integers(40, 80))
            color = (RED, BLUE, LIME)[int(rng.integers(0, 3))]
            cv2.rectangle(img, (x, y), (x + 2, y + 2), color, -1)
        poses = detect_team_colors(img, L)
        self.assertEqual(set(poses), {"striker", "keeper"})
        self.assertPose(poses["striker"], striker)
        self.assertPose(poses["keeper"], keeper)

    def test_pink_ball_is_not_a_keeper(self):
        img = draw(field(), Robot((200, 240), 0, BLUE))
        fill_disk(img, (420, 240), TEAM_PATCH_SIDE_L * L, PINK)
        fill_disk(img, (520, 120), 0.40 * L, PINK)
        poses = detect_team_colors(img, L)
        self.assertNotIn("keeper", poses)
        self.assertIn("striker", poses)

    def test_elongated_patch_is_not_a_team_and_gives_no_axis(self):
        img = field()
        fill_rect(img, (220, 240), 30, 0.5 * L, 0.18 * L, BLUE)
        lime_square(img, offset((220, 240), 30, NOSE_OFFSET_L * L))
        keeper = Robot((450, 240), 90, RED)
        poses = detect_team_colors(draw(img, keeper), L)
        self.assertNotIn("striker", poses)
        self.assertPose(poses.get("keeper"), keeper)

    # Duplicates and overlap

    def test_two_plausible_same_team_patches_rejected(self):
        keeper = Robot((320, 380), 0, RED)
        img = draw(
            field(), Robot((160, 160), 0, BLUE), Robot((480, 160), 90, BLUE), keeper
        )
        poses = detect_team_colors(img, L)
        self.assertNotIn("striker", poses)
        self.assertPose(poses.get("keeper"), keeper)

    def test_near_miss_duplicate_still_blocks(self):
        img = draw(field(), Robot((200, 240), 0, BLUE))
        fill_rect(img, (500, 120), 0, 0.18 * L, 0.18 * L, BLUE)  # small but patch-like
        self.assertNotIn("striker", detect_team_colors(img, L))

    def test_overlapping_same_team_patches_rejected(self):
        img = field()
        side = TEAM_PATCH_SIDE_L * L
        fill_rect(img, (300, 240), 0, side, side, BLUE)
        fill_rect(img, (300 + 0.2 * L, 240 + 0.2 * L), 0, side, side, BLUE)
        self.assertNotIn("striker", detect_team_colors(img, L))

    # Missing or ambiguous nose

    def test_missing_nose_gives_center_preview(self):
        striker = Robot((320, 240), 40, BLUE, nose=False)
        poses = detect_team_colors(draw(field(), striker), L)
        self.assertPose(poses.get("striker"), striker, heading=False)
        self.assertGreater(poses["striker"].score, 0.0)

    def test_far_lime_clutter_is_not_a_nose(self):
        striker = Robot((240, 240), 0, BLUE, nose=False)
        img = draw(field(), striker)
        lime_square(img, (240 + 0.8 * L, 240))
        poses = detect_team_colors(img, L)
        self.assertPose(poses.get("striker"), striker, heading=False)

    def test_two_nose_candidates_give_no_heading(self):
        striker = Robot((320, 240), 0, BLUE)
        img = draw(field(), striker)
        lime_square(img, offset(striker.center, 180, NOSE_OFFSET_L * L))  # behind
        poses = detect_team_colors(img, L)
        self.assertPose(poses.get("striker"), striker, heading=False)

    def test_nearby_lime_rival_gives_no_heading(self):
        striker = Robot((320, 240), 0, BLUE)
        img = draw(field(), striker)
        lime_square(img, (320, 240 + 0.5 * L))  # off to the side, within reach
        poses = detect_team_colors(img, L)
        self.assertPose(poses.get("striker"), striker, heading=False)

    def test_nose_merged_into_large_lime_area_gives_no_heading(self):
        striker = Robot((320, 240), 0, BLUE)
        img = draw(field(), striker)
        nose = nose_of(striker)
        cv2.rectangle(img, (int(nose[0]), 100), (int(nose[0]) + 60, 380), LIME, -1)
        poses = detect_team_colors(img, L)
        self.assertPose(poses.get("striker"), striker, heading=False)

    # Crossings and shared noses

    def test_shared_nose_between_teams_gives_no_heading(self):
        striker = Robot((250, 240), 0, BLUE, nose=False)
        keeper = Robot((250 + 0.6 * L, 240), 180, RED, nose=False)
        img = draw(field(), striker, keeper)
        lime_square(img, (250 + 0.3 * L, 240))
        poses = detect_team_colors(img, L)
        self.assertPose(poses.get("striker"), striker, heading=False)
        self.assertPose(poses.get("keeper"), keeper, heading=False)

    def test_close_crossing_gives_no_heading(self):
        striker = Robot((300, 240), 0, BLUE)
        keeper = Robot((300, 290), 180, RED)
        poses = detect_team_colors(draw(field(), striker, keeper), L)
        self.assertPose(poses.get("striker"), striker, heading=False)
        self.assertPose(poses.get("keeper"), keeper, heading=False)

    def test_side_by_side_robots_keep_heading(self):
        striker = Robot((300, 200), 0, BLUE)
        keeper = Robot((300, 200 + 0.5 * L), 180, RED)
        poses = detect_team_colors(draw(field(), striker, keeper), L)
        self.assertPose(poses.get("striker"), striker)
        self.assertPose(poses.get("keeper"), keeper)

    # ROI and image borders

    def test_roi_straddling_patch_keeps_unclipped_center(self):
        roi = np.zeros((H, W), np.uint8)
        roi[:, :320] = 255
        striker = Robot((312, 240), 90, BLUE)  # patch spans x 288..336
        keeper = Robot((120, 240), 0, RED)
        poses = detect_team_colors(draw(field(), striker, keeper), L, roi_mask=roi)
        self.assertPose(poses.get("striker"), striker)
        self.assertPose(poses.get("keeper"), keeper)

    def test_roi_ignores_duplicate_outside_field(self):
        roi = np.zeros((H, W), bool)
        roi[:, :320] = True
        striker = Robot((150, 240), 0, BLUE)
        keeper = Robot((150, 400), 0, RED)
        img = draw(field(), striker, keeper, Robot((500, 240), 0, BLUE))
        self.assertNotIn("striker", detect_team_colors(img, L))
        poses = detect_team_colors(img, L, roi_mask=roi)
        self.assertPose(poses.get("striker"), striker)
        self.assertPose(poses.get("keeper"), keeper)

    def test_roi_excludes_patch_centered_outside(self):
        roi = np.zeros((H, W), np.uint8)
        roi[:, :320] = 1
        img = draw(field(), Robot((330, 240), 90, BLUE))
        self.assertNotIn("striker", detect_team_colors(img, L, roi_mask=roi))

    def test_truncated_patch_at_image_border_not_trusted(self):
        img = draw(field(), Robot((10, 240), 90, BLUE))
        self.assertNotIn("striker", detect_team_colors(img, L))
        draw(img, Robot((320, 240), 0, BLUE))
        self.assertNotIn("striker", detect_team_colors(img, L))

    # Input validation

    def test_input_validation(self):
        img = field()
        for bad in (0, -5, float("nan"), float("inf")):
            with self.subTest(length=bad), self.assertRaises(ValueError):
                detect_team_colors(img, bad)
        with self.assertRaises(ValueError):
            detect_team_colors(img[:, :, 0], L)
        with self.assertRaises(ValueError):
            detect_team_colors(img.astype(np.float32), L)
        with self.assertRaises(ValueError):
            detect_team_colors(img, L, roi_mask=np.ones((H, W - 1), np.uint8))


if __name__ == "__main__":
    unittest.main()
