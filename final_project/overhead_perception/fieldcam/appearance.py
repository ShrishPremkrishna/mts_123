"""Anonymous body centers from pale limb pairs near a dark torso in an overhead view.

Input is the unannotated BGR camera frame. Scores are shape agreement on [0, 1],
not calibrated probabilities. An axis is an image-plane line modulo pi, never a
heading. Neither detection nor temporal association identifies a team or robot.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import combinations
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class BodyCandidate:
    center_px: tuple[float, float]
    score: float
    axis_rad: float | None
    consecutive_frames: int = 1
    ambiguous: bool = False


def _limb_pair_score(first, second, body_center):
    ax, ay, area_a = first
    bx, by, area_b = second
    distance = math.hypot(ax - bx, ay - by)
    offset = math.dist(((ax + bx) / 2, (ay + by) / 2), body_center)
    if not 8 <= distance <= 34 or offset > 13:
        return 0.
    size = min(1., min(area_a, area_b) / 22)
    balance = min(1., 2 * min(area_a, area_b) / max(area_a, area_b))
    return size * balance * math.exp(-.5 * (((distance - 20) / 10)**2 + (offset / 10)**2))


def detect_puppers(bgr, expected_length_px=None) -> list[BodyCandidate]:
    """Detect on the whole camera image. Supply body length in *input* pixels when known."""
    if not isinstance(bgr, np.ndarray) or bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
        raise ValueError("appearance detection requires an 8-bit BGR camera image")
    height, width = bgr.shape[:2]
    expected_length_px = width * .075 if expected_length_px is None else expected_length_px
    if not math.isfinite(expected_length_px) or expected_length_px <= 0:
        raise ValueError("expected body length must be positive and finite")
    if expected_length_px < 20:
        return []
    candidates = _at_scale(bgr, expected_length_px)
    # Perspective, crop and posture change the apparent footprint. Evaluate a
    # second nearby scale; neither pass is told where a robot should appear.
    candidates.extend(_at_scale(bgr, 1.25 * expected_length_px))
    kept = []
    for candidate in sorted(candidates, key=lambda item: -item.score):
        if all(math.dist(candidate.center_px, other.center_px) > .5 * expected_length_px for other in kept):
            kept.append(candidate)
    return kept


def _at_scale(bgr, expected_length_px):
    height, width = bgr.shape[:2]
    # Normalize the robot to 40 pixels. Areas, pair spacing and margins below
    # refer to this small image rather than a fixed camera resolution.
    scale = 40 / expected_length_px
    if min(width, height) * scale < 2 or width * height * scale**2 > 2_000_000:
        return []
    frame = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Phone auto-exposure changes the white level while the limb/body contrast
    # stays similar. Sample the entire image; an all-dark image stays dark.
    white_level = float(np.percentile(gray[::2, ::2], 99.8))
    if white_level < 40:
        return []
    if white_level < 220:
        gray = cv2.convertScaleAbs(gray, alpha=min(1.9, 220 / white_level))
    saturation = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 1]
    background = cv2.blur(gray, (41, 41)).astype(np.float32)
    dark = ((gray < 76) & (gray < background - 19)).astype(np.uint8)
    bright = ((saturation < 78) & (gray > 152) & (gray > background + 28)).astype(np.uint8)
    _, _, regions, centers = cv2.connectedComponentsWithStats(bright)
    limbs = [(cx, cy, int(area)) for (_, _, w, h, area), (cx, cy) in zip(regions[1:], centers[1:])
             if 3 <= area <= 105 and 3 <= max(w, h) <= 24 and min(w, h) <= 12]
    if len(limbs) < 2:
        return []
    # Two pale patches inside one continuous pale sheet are not independent
    # limbs. This rejects dark graphics on the white cards and markings.
    contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sheets = [contour for contour in contours if cv2.contourArea(contour) > 160]

    joined = cv2.morphologyEx(dark, cv2.MORPH_CLOSE,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    _, labels, regions, centers = cv2.connectedComponentsWithStats(joined)
    candidates = []
    limbs = np.asarray(limbs)
    for label, (region, center) in enumerate(zip(regions[1:], centers[1:]), 1):
        x, y, w, h, area = region
        if not 34 <= area <= 1250 or not 8 <= max(w, h) <= 56 or min(w, h) < 5:
            continue
        if any(cv2.pointPolygonTest(sheet, tuple(center), False) >= 0 for sheet in sheets):
            continue
        close = np.sum((limbs[:, :2] - center)**2, axis=1) < 25**2
        neighbors = limbs[close]
        # A dense cluster of white patches is not a resolved set of legs.
        if not 2 <= len(neighbors) <= 24:
            continue
        pair = max(_limb_pair_score(a, b, center) for a, b in combinations(neighbors, 2))
        density = dark[y:y + h, x:x + w].sum() / area
        score = float(pair * min(1., 1.5 * density))
        if score < .44:
            continue
        # A cast shadow can pull the dark centroid away from the animal. Use the
        # distribution of limb pixels as well, capping the weight of a single link.
        support = np.average(neighbors[:, :2], axis=0, weights=np.minimum(neighbors[:, 2], 45))
        cx, cy = (.35 * center + .65 * support) / scale
        ys, xs = np.nonzero(labels[y:y + h, x:x + w] == label)
        eigenvalues, vectors = np.linalg.eigh(np.cov(np.stack((xs, ys))))
        axis = None
        if eigenvalues[1] > 1.5 * eigenvalues[0]:
            axis = math.atan2(vectors[1, 1], vectors[0, 1]) % math.pi
        candidates.append(BodyCandidate((float(cx), float(cy)), score, axis))

    return candidates


class AppearanceTracker:
    """Count continuous, unambiguous center observations. Never coast through a missing frame."""
    def __init__(self):
        self.previous = []
        self.captured_at = None

    def update(self, candidates, captured_at, expected_length_px):
        if self.captured_at is None or not 0 < captured_at - self.captured_at <= .30:
            self.previous = []
        self.captured_at = captured_at
        neighbors = [[i for i, old in enumerate(self.previous)
                      if math.dist(new.center_px, old.center_px) < .65 * expected_length_px]
                     for new in candidates]
        counts = [sum(i in nearby for nearby in neighbors) for i in range(len(self.previous))]
        observations = []
        for new, nearby in zip(candidates, neighbors):
            ambiguous = len(nearby) > 1 or any(counts[i] > 1 for i in nearby)
            age = 1
            if nearby and not ambiguous and not self.previous[nearby[0]].ambiguous:
                age += self.previous[nearby[0]].consecutive_frames
            observations.append(replace(new, consecutive_frames=age, ambiguous=ambiguous))
        self.previous = observations
        return observations
