"""Find a visible pink ball in the fixed onboard RGB panorama. No motion output."""
import math

import cv2


class PinkBallTracker:
    """Associate current colour observations. Never extrapolate a missing box."""

    def __init__(self):
        self.center = self.stamp = None
        self.open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def reset(self):
        self.center = self.stamp = None

    @staticmethod
    def supported(box, yolo_boxes):
        x1, y1, x2, y2 = box
        area = (x2 - x1) * (y2 - y1)
        for a, b, c, d in yolo_boxes:
            intersect = max(0, min(x2, c) - max(x1, a)) * max(0, min(y2, d) - max(y1, b))
            union = area + (c - a) * (d - b) - intersect
            if union > 0 and intersect / union >= .25:
                return True
        return False

    def observe(self, rgb, detections, stamp_ns, mode):
        if mode == "off":
            self.reset()
            return {"state": "off"}
        # Use current pixels from the same panorama as the COCO boxes. Constrain
        # the search to the floor and immediately above camera height for close balls.
        h, w = rgb.shape[:2]
        top = int(h * .35)
        hsv = cv2.cvtColor(rgb[top:], cv2.COLOR_RGB2HSV)
        # The actual ball's pale patches cross OpenCV hue zero in the
        # archived field photo. Restrict the red/orange side to pale bright
        # pixels so ordinary saturated team red does not become pink.
        saturated = cv2.inRange(hsv, (140, 140, 105), (175, 255, 255))
        pale = cv2.inRange(hsv, (164, 30, 188), (179, 150, 255))
        pale |= cv2.inRange(hsv, (0, 30, 188), (7, 150, 255))
        # Keep the pale panels separate from highly saturated carpet or clothing.
        # A bright core can still form a round candidate when a pink cast joins
        # a dimmer part of the ball to its surroundings.
        contours = []
        for mask in (saturated, pale, cv2.bitwise_and(pale, cv2.inRange(hsv[:, :, 2], 210, 255))):
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.close_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel)
            contours.extend(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0])
        yolo_boxes = [d.box for d in detections if d.class_name == "sports ball"]
        recent = self.stamp is not None and 0 < stamp_ns - self.stamp <= 350_000_000
        ranked = []
        # YOLO can separate the object from a pink background even when the
        # colour component merges into it. Verify pink inside the object before
        # using that semantic box. Colour-only candidates use stricter shape checks.
        pink = cv2.bitwise_or(saturated, pale)
        for box in yolo_boxes:
            x1, y1, x2, y2 = box
            bw, bh = x2 - x1, y2 - y1
            if min(bw, bh) <= 0 or min(bw, bh) / max(bw, bh) < .45:
                continue
            left, right = max(0, int(x1 + bw * .2)), min(w, math.ceil(x2 - bw * .2))
            start, end = int(y1 + bh * .2) - top, math.ceil(y2 - bh * .2) - top
            if left >= right or start < 0 or end > h - top or start >= end:
                continue
            core = pink[start:end, left:right]
            if cv2.countNonZero(core) / core.size >= .55:
                center = ((x1 + x2) / (2 * w), (y1 + y2) / (2 * h))
                ranked.append((2., box, center, True))
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if (area < 28 * (w / 700) ** 2 or min(bw, bh) < 5
                    or min(bw, bh) / max(bw, bh) < .60 or area / (bw * bh) < .48):
                continue
            perimeter = cv2.arcLength(cv2.convexHull(contour), True)
            compactness = 4 * math.pi * area / max(1., perimeter ** 2)
            _, radius = cv2.minEnclosingCircle(contour)
            fill = area / max(1., math.pi * radius ** 2)
            if compactness < .48 or fill < .58:
                continue
            box = (x, y + top, x + bw, y + bh + top)
            confirmed = self.supported(box, yolo_boxes)
            if not confirmed and (mode == "confirmed" or compactness < .63 or fill < .72):
                continue
            center = ((x + bw / 2) / w, (top + y + bh / 2) / h)
            continuity = 0.
            if recent:
                distance = math.dist(center, self.center)
                continuity = max(0., 1 - distance / .16)
            quality = .5 * (compactness + fill) + .45 * confirmed + .30 * continuity
            ranked.append((quality, box, center, confirmed))
        if not ranked:
            return {"state": "searching"}
        ranked.sort(reverse=True)
        best_quality, best_box, center, confirmed = ranked[0]
        for quality, box, _, _ in ranked[1:]:
            if best_quality - quality >= .15:
                break
            if not self.supported(box, [best_box]):
                return {"state": "ambiguous"}
        x1, y1, x2, y2 = best_box
        # An isolated glint on the floor must not acquire a colour-only track.
        # YOLO corroboration or a second nearby measurement can acquire it.
        matched = recent and math.dist(center, self.center) < min(.12, .025 + (x2 - x1) / w)
        self.center, self.stamp = center, stamp_ns
        if not (matched or confirmed):
            return {"state": "searching"}
        return dict(state="tracked", box=[x1 / w, y1 / h, x2 / w, y2 / h],
                    source="pink+yolo" if confirmed else "pink")
