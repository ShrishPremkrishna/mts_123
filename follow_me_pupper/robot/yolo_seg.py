#!/usr/bin/env python3
"""YOLOv8n-seg on the Hailo-8L: boxes *and* per-object segmentation masks.

The .hef comes straight from the Hailo model zoo, so the chip hands back the raw
head tensors and the decoding is up to us:

    conv44/60/73  (80x80/40x40/20x20, 64ch)  box distributions (4 sides x 16 DFL bins)
    conv45/61/74  (       "         , 80ch)  class logits
    conv46/62/75  (       "         , 32ch)  mask coefficients
    conv48        (160x160         , 32ch)   mask prototypes

A detection's mask is a weighted sum of the 32 prototypes, squashed with a
sigmoid and cropped to its own box -- that's the whole trick behind YOLACT-style
instance segmentation, which is what YOLOv8-seg uses.

Grab the model with:  scripts/download_model.sh

Quick check on a still image:

    python3 robot/yolo_seg.py --image bus.jpg --out annotated.jpg
"""

import os
import time

import cv2
import numpy as np

MODEL_INPUT_SIZE = 640
PROTO_SIZE = 160
NUM_CLASSES = 80
DFL_BINS = 16
NUM_PROTOS = 32

DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolov8n_seg.hef")
DEFAULT_LABELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "coco.txt")


def load_labels(path=DEFAULT_LABELS):
    with open(path) as f:
        return [line.strip() for line in f if line.strip()]


def class_color(class_id):
    """Stable, well-spread BGR-agnostic RGB colour for a class id."""
    hue = int((class_id * 47) % 180)
    hsv = np.uint8([[[hue, 200, 255]]])
    r, g, b = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[0, 0]
    return int(r), int(g), int(b)


def letterbox(frame, size=MODEL_INPUT_SIZE):
    """Resize keeping aspect ratio, pad the rest grey. Returns (img, scale, pad)."""
    h, w = frame.shape[:2]
    scale = min(size / w, size / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, np.uint8)
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized
    return canvas, scale, (pad_x, pad_y)


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _softmax(x, axis=-1):
    x = x - x.max(axis=axis, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=axis, keepdims=True)


def _nms(boxes, scores, iou_threshold):
    """Plain greedy NMS. boxes are xyxy, already class-offset if needed."""
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / np.maximum(areas[i] + areas[rest] - inter, 1e-9)
        order = rest[iou <= iou_threshold]
    return keep


class Detection:
    """One detected object, in the coordinate frame of the image passed to infer()."""

    __slots__ = ("class_id", "class_name", "score", "box", "mask")

    def __init__(self, class_id, class_name, score, box, mask):
        self.class_id = class_id
        self.class_name = class_name
        self.score = score
        self.box = box  # (x1, y1, x2, y2) floats, in frame pixels
        self.mask = mask  # bool array the size of the frame, or None

    @property
    def center(self):
        x1, y1, x2, y2 = self.box
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0

    def __repr__(self):
        x1, y1, x2, y2 = (round(v, 1) for v in self.box)
        return f"<{self.class_name} {self.score:.2f} [{x1},{y1},{x2},{y2}]>"


class HailoYoloSeg:
    """Runs yolov8n_seg.hef on the Hailo accelerator.

    Not thread-safe: call infer() from a single thread (viser_camera.py gives it
    its own worker thread so the video stream never waits on the network).
    """

    def __init__(
        self,
        model_path=DEFAULT_MODEL,
        labels_path=DEFAULT_LABELS,
        score_threshold=0.4,
        iou_threshold=0.45,
        max_detections=20,
        want_masks=True,
    ):
        from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice

        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"{model_path} not found. Run scripts/download_model.sh to fetch it."
            )

        self.score_threshold = score_threshold
        self.iou_threshold = iou_threshold
        self.max_detections = max_detections
        self.want_masks = want_masks
        # Class ids to build masks for (None: every class). Each mask is a
        # full-frame resize, so skipping the classes nobody looks at saves CPU.
        self.mask_classes = None
        self.labels = load_labels(labels_path)

        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        self.target = VDevice(params)
        self.infer_model = self.target.create_infer_model(model_path)
        self.infer_model.set_batch_size(1)

        # Ask HailoRT to dequantize for us; the tensors are small enough that the
        # float conversion is cheap next to the decoding below.
        for name in self.infer_model.output_names:
            self.infer_model.output(name).set_format_type(FormatType.FLOAT32)

        # Sort the output layers by what they are, using their shapes: 64 channels
        # is the box head, 80 the class head, 32 at 160x160 the prototypes and 32
        # elsewhere the mask coefficients.
        self.levels = {}  # grid size -> {"box": name, "cls": name, "coef": name}
        self.proto_name = None
        self.output_shapes = {}
        for name in self.infer_model.output_names:
            shape = tuple(self.infer_model.output(name).shape)
            self.output_shapes[name] = shape
            grid, channels = shape[0], shape[2]
            if channels == NUM_PROTOS and grid == PROTO_SIZE:
                self.proto_name = name
                continue
            slot = {4 * DFL_BINS: "box", NUM_CLASSES: "cls", NUM_PROTOS: "coef"}.get(channels)
            if slot is None:
                raise RuntimeError(f"Unexpected output layer {name} with shape {shape}")
            self.levels.setdefault(grid, {})[slot] = name

        if self.proto_name is None or not self.levels:
            raise RuntimeError("HEF does not look like yolov8n_seg (missing heads)")

        # Anchor centres and DFL bin values, precomputed once per grid.
        self.grids = {}
        for grid in self.levels:
            stride = MODEL_INPUT_SIZE / grid
            ys, xs = np.meshgrid(np.arange(grid), np.arange(grid), indexing="ij")
            centers = np.stack([xs, ys], -1).reshape(-1, 2).astype(np.float32) + 0.5
            self.grids[grid] = (centers * stride, stride)
        self.dfl_bins = np.arange(DFL_BINS, dtype=np.float32)

        self._configured = self.infer_model.configure()
        self._bindings = self._configured.create_bindings(
            output_buffers={
                name: np.empty(shape, dtype=np.float32)
                for name, shape in self.output_shapes.items()
            }
        )
        self.last_inference_ms = 0.0

    def close(self):
        # Release the accelerator so another process (or a relaunch) can grab it.
        configured, self._configured = self._configured, None
        if configured is not None:
            configured.__exit__(None, None, None)
        self.target.release()

    def infer(self, frame):
        """Detect on an RGB frame. Returns a list of Detection in frame pixels."""
        letterboxed, scale, pad = letterbox(frame)

        start = time.time()
        self._bindings.input().set_buffer(np.ascontiguousarray(letterboxed))
        self._configured.run([self._bindings], 10000)
        self.last_inference_ms = (time.time() - start) * 1000.0

        outputs = {
            name: self._bindings.output(name).get_buffer()
            for name in self.output_shapes
        }
        return self._decode(outputs, frame.shape[:2], scale, pad)

    def _decode(self, outputs, frame_shape, scale, pad):
        boxes, scores, class_ids, coeffs = [], [], [], []

        for grid, names in self.levels.items():
            # The class head already comes out of the chip as probabilities (the
            # sigmoid is folded into the compiled model), so use it as-is. The
            # box head does not -- it is still raw DFL logits, see below.
            cls = outputs[names["cls"]].reshape(-1, NUM_CLASSES)
            best = cls.max(axis=1)
            hits = np.nonzero(best >= self.score_threshold)[0]
            if hits.size == 0:
                continue

            # Distribution Focal Loss: each side of the box is a 16-bin histogram
            # over distances (in grid cells) from the anchor centre. Expected
            # value of that histogram is the distance.
            raw = outputs[names["box"]].reshape(-1, 4 * DFL_BINS)[hits]
            dist = (_softmax(raw.reshape(-1, 4, DFL_BINS), axis=2) * self.dfl_bins).sum(axis=2)

            centers, stride = self.grids[grid]
            cx, cy = centers[hits, 0], centers[hits, 1]
            left, top, right, bottom = (dist[:, i] * stride for i in range(4))

            boxes.append(np.stack([cx - left, cy - top, cx + right, cy + bottom], axis=1))
            scores.append(best[hits])
            class_ids.append(cls[hits].argmax(axis=1))
            coeffs.append(outputs[names["coef"]].reshape(-1, NUM_PROTOS)[hits])

        if not boxes:
            return []

        boxes = np.concatenate(boxes).astype(np.float32)
        scores = np.concatenate(scores).astype(np.float32)
        class_ids = np.concatenate(class_ids).astype(np.int32)
        coeffs = np.concatenate(coeffs).astype(np.float32)

        # Offset each class into its own region of the plane so NMS never merges
        # two different classes that happen to overlap.
        offsets = class_ids[:, None].astype(np.float32) * (MODEL_INPUT_SIZE * 2)
        keep = _nms(boxes + offsets, scores, self.iou_threshold)[: self.max_detections]
        if not keep:
            return []

        boxes, scores = boxes[keep], scores[keep]
        class_ids, coeffs = class_ids[keep], coeffs[keep]

        masks = None
        if self.want_masks:
            wanted = np.arange(len(keep))
            if self.mask_classes is not None:
                wanted = np.nonzero(np.isin(class_ids, list(self.mask_classes)))[0]
            masks = [None] * len(keep)
            if wanted.size:
                built = self._masks(outputs[self.proto_name], coeffs[wanted], boxes[wanted],
                                    frame_shape, scale, pad)
                for i, mask in zip(wanted, built):
                    masks[i] = mask

        # Letterbox space -> original frame pixels.
        pad_x, pad_y = pad
        h, w = frame_shape
        boxes[:, [0, 2]] = np.clip((boxes[:, [0, 2]] - pad_x) / scale, 0, w - 1)
        boxes[:, [1, 3]] = np.clip((boxes[:, [1, 3]] - pad_y) / scale, 0, h - 1)

        return [
            Detection(
                int(class_ids[i]),
                self.labels[class_ids[i]] if class_ids[i] < len(self.labels) else str(class_ids[i]),
                float(scores[i]),
                tuple(float(v) for v in boxes[i]),
                None if masks is None else masks[i],
            )
            for i in range(len(keep))
        ]

    def _masks(self, proto, coeffs, boxes, frame_shape, scale, pad):
        """Prototype masks x coefficients -> one full-resolution mask per box."""
        h, w = frame_shape
        pad_x, pad_y = pad
        ratio = MODEL_INPUT_SIZE / PROTO_SIZE  # 4: proto pixels are 4x model pixels

        # (160*160, 32) @ (32, N) -> one soft mask per detection.
        soft = _sigmoid(proto.reshape(-1, NUM_PROTOS) @ coeffs.T)
        soft = soft.reshape(PROTO_SIZE, PROTO_SIZE, -1)

        # Drop the letterbox padding, then stretch back to the frame.
        x0 = int(round(pad_x / ratio))
        y0 = int(round(pad_y / ratio))
        x1 = PROTO_SIZE - x0
        y1 = PROTO_SIZE - y0
        soft = soft[max(y0, 0) : max(y1, y0 + 1), max(x0, 0) : max(x1, x0 + 1)]

        masks = []
        for i in range(len(boxes)):
            full = cv2.resize(soft[:, :, i], (w, h), interpolation=cv2.INTER_LINEAR)
            mask = full > 0.5
            # Crop to the box: prototypes are global, so a mask can otherwise
            # bleed onto other instances of the same class.
            bx1 = int(np.clip((boxes[i][0] - pad_x) / scale, 0, w))
            by1 = int(np.clip((boxes[i][1] - pad_y) / scale, 0, h))
            bx2 = int(np.clip(np.ceil((boxes[i][2] - pad_x) / scale), 0, w))
            by2 = int(np.clip(np.ceil((boxes[i][3] - pad_y) / scale), 0, h))
            cropped = np.zeros_like(mask)
            cropped[by1:by2, bx1:bx2] = mask[by1:by2, bx1:bx2]
            masks.append(cropped)
        return masks


def draw_detections(frame, detections, draw_boxes=True, draw_masks=True, mask_alpha=0.45):
    """Annotate an RGB frame in place-safe fashion (returns a new array)."""
    out = frame.copy()

    if draw_masks and detections:
        overlay = out.copy()
        painted = False
        for det in detections:
            if det.mask is None or not det.mask.any():
                continue
            overlay[det.mask] = class_color(det.class_id)
            painted = True
        if painted:
            cv2.addWeighted(overlay, mask_alpha, out, 1 - mask_alpha, 0, out)
        for det in detections:
            if det.mask is None:
                continue
            contours, _ = cv2.findContours(
                det.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(out, contours, -1, class_color(det.class_id), 1)

    for det in detections:
        x1, y1, x2, y2 = (int(round(v)) for v in det.box)
        color = class_color(det.class_id)
        if draw_boxes:
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        label = f"{det.class_name} {det.score:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(y1, th + 4)
        cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 4, ty), color, -1)
        cv2.putText(
            out, label, (x1 + 2, ty - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA
        )
    return out


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Run yolov8n-seg on a still image.")
    parser.add_argument("--image", required=True)
    parser.add_argument("--out", default="annotated.jpg")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--score", type=float, default=0.4)
    args = parser.parse_args()

    bgr = cv2.imread(args.image)
    if bgr is None:
        raise SystemExit(f"Could not read {args.image}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    model = HailoYoloSeg(model_path=args.model, score_threshold=args.score)
    try:
        detections = model.infer(rgb)  # first call includes warm-up
        detections = model.infer(rgb)
    finally:
        model.close()

    print(f"inference: {model.last_inference_ms:.1f} ms")
    for det in detections:
        print(" ", det, "mask px:", int(det.mask.sum()) if det.mask is not None else "-")

    annotated = draw_detections(rgb, detections)
    cv2.imwrite(args.out, cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
