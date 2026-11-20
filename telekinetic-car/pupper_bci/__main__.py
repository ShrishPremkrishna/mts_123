"""Replay, record, preview or drive Pupper from a Cyton headset."""

import argparse
import csv
import math
from pathlib import Path
import sys
import time
from typing import Iterator

from .control import INPUT_MAX_AGE, Controller, Motion, calibrate
from .limits import COMMAND_PERIOD
from .source import RECORDING_COLUMNS, SAMPLE_RATE, CytonSource, Sample, read_recording

CALIBRATION_SECONDS = 12.0


def demo_samples() -> list[Sample]:
    """Known rest, focus, left/right tilt and an unplugged headset."""
    samples = []
    for index in range(41 * SAMPLE_RATE):
        t = index / SAMPLE_RATE
        focused = 12 <= t < 24 or 30 <= t < 36 or t >= 39
        beta = (12 if focused else 2) * math.sin(2 * math.pi * 13 * t)
        other = 4 * math.sin(2 * math.pi * 7 * t)
        tilt = 0
        if 26 <= t < 28 or 33 <= t < 36:
            tilt = 20
        elif 28 <= t < 30:
            tilt = -20
        angle = math.radians(tilt)
        accel = (math.sin(angle), 0.0, math.cos(angle)) if index % 10 == 0 else None
        samples.append(Sample(t, index % 256, (beta + other, 0.9 * beta - other), accel))
    return samples


def replay(samples: list[Sample], stage_seconds: float, tilt_sign: int) -> Iterator[tuple[float, Motion]]:
    control_start = 2 * stage_seconds
    rest = [sample for sample in samples if sample.t < stage_seconds]
    focus = [sample for sample in samples if stage_seconds <= sample.t < control_start]
    motion_samples = [sample for sample in samples if sample.t >= control_start]
    if not motion_samples:
        raise ValueError("replay needs a recording after the rest and focus calibration stages")
    controller = Controller(calibrate(rest, focus), tilt_sign)
    index = tick = 0
    while control_start + tick * COMMAND_PERIOD <= motion_samples[-1].t + 2 * INPUT_MAX_AGE:
        elapsed = tick * COMMAND_PERIOD
        now = control_start + elapsed
        next_index = index
        while next_index < len(motion_samples) and motion_samples[next_index].t <= now:
            next_index += 1
        command = controller.step(motion_samples[index:next_index], now=now, enabled=elapsed >= 1.5)
        yield now, command
        index = next_index
        tick += 1


def _stream_for(source: CytonSource, seconds: float) -> Iterator[tuple[float, float, list[Sample]]]:
    start = time.monotonic()
    end = start + seconds
    while True:
        batch = source.read()
        now = time.monotonic()
        yield start, now, [sample for sample in batch if start <= sample.t < end and 0 <= now - sample.t <= INPUT_MAX_AGE]
        if now >= end:
            return
        time.sleep(min(COMMAND_PERIOD, end - now))


def _capture(args: argparse.Namespace, source: CytonSource) -> None:
    with args.output.open("x", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(RECORDING_COLUMNS)
        previous_stage = None
        count = 0
        for start, now, batch in _stream_for(source, 2 * args.stage_seconds + args.seconds):
            elapsed = now - start
            stage = 0 if elapsed < args.stage_seconds else 1 if elapsed < 2 * args.stage_seconds else 2
            if stage != previous_stage:
                print(("Relax with your head level.", "Focus with your head level.",
                       "Try relaxed left/right tilts, then focus and release.")[stage], flush=True)
                previous_stage = stage
            for sample in batch:
                writer.writerow((sample.t - start, sample.sequence, *sample.eeg_uv,
                                 *(sample.accel_g if sample.accel_g is not None else ("", "", ""))))
                count += 1
    if not count:
        raise RuntimeError("no Cyton samples captured")
    print(f"Saved {count} samples to {args.output}")


def _collect(source: CytonSource, instruction: str, seconds: float) -> list[Sample]:
    input(f"{instruction} Press Enter to begin the {seconds:g}-second recording. ")
    print("Recording…", flush=True)
    return [sample for _, _, batch in _stream_for(source, seconds) for sample in batch]


def _format(command: Motion) -> str:
    focus = "—" if command.focus is None else f"{command.focus:.2f}"
    tilt = "—" if command.tilt_deg is None else f"{command.tilt_deg:+.1f}°"
    return (f"{command.state:<20} forward={command.forward:.2f} m/s "
            f"yaw={command.yaw:+.2f} rad/s   focus={focus}   tilt={tilt}")


def _live(source: CytonSource, controller: Controller, robot=None) -> None:
    preview_enabled = False
    status = None
    reported_at = -math.inf
    while True:
        tick = time.monotonic()
        samples = source.read()
        enabled = robot.poll() if robot is not None else preview_enabled
        now = time.monotonic()
        command = controller.step(samples, now=now, enabled=enabled)
        if robot is not None:
            robot.publish(command.forward, command.yaw)
        else:
            preview_enabled = command.state in ("released", "enabled")
        waiting_reason = robot.waiting_reason if robot is not None else None
        next_status = command.state, waiting_reason
        if next_status != status or now - reported_at >= 1.0:
            suffix = f"   {waiting_reason}" if waiting_reason else ""
            print(_format(command) + suffix, flush=True)
            reported_at = now
            status = next_status
        time.sleep(max(0.0, tick + COMMAND_PERIOD - time.monotonic()))


def _stage_seconds(value: str) -> float:
    seconds = float(value)
    if not 9 <= seconds <= 60:
        raise argparse.ArgumentTypeError("calibration stages must be between 9 and 60 seconds")
    return seconds


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    offline = sub.add_parser("replay", help="run recorded or demo samples with no hardware")
    offline.add_argument("--recording", type=Path, help="optional CSV. Omit for a synthetic demo")
    offline.add_argument("--flip-tilt", action="store_true")
    offline.add_argument("--stage-seconds", type=_stage_seconds, default=CALIBRATION_SECONDS)
    for mode, help_text in (
        ("capture", "save a calibration and test recording with the headset only"),
        ("monitor", "preview commands from the headset with no ROS or robot"),
        ("drive", "use the headset in an already running stock-walk session"),
    ):
        action = sub.add_parser(mode, help=help_text)
        action.add_argument("--port", required=True, help="Cyton USB dongle serial port")
        action.add_argument("--channels", type=int, nargs=2, default=(1, 2), metavar=("FP1", "FP2"))
        action.add_argument("--stage-seconds", type=_stage_seconds, default=CALIBRATION_SECONDS)
        if mode == "capture":
            action.add_argument("--output", type=Path, required=True)
            action.add_argument("--seconds", type=float, default=15.0, help="test recording after calibration")
        else:
            action.add_argument("--flip-tilt", action="store_true")
        if mode == "drive":
            action.add_argument("--runtime-receipt", type=Path, required=True)
            action.add_argument("--enable-button", type=int, required=True, help="hold-to-run button index in /joy")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    tilt_sign = -1 if getattr(args, "flip_tilt", False) else 1
    try:
        if args.mode == "replay":
            if args.recording is None and args.stage_seconds != CALIBRATION_SECONDS:
                parser.error("the synthetic demo has 12-second calibration stages")
            samples = read_recording(args.recording) if args.recording else demo_samples()
            print("Offline replay. An enable press is simulated 1.5 seconds after calibration.")
            previous = None
            for t, command in replay(samples, args.stage_seconds, tilt_sign):
                signature = command.state, command.forward, round(command.yaw, 2)
                if signature != previous:
                    print(f"{t:6.2f}s  {_format(command)}")
                    previous = signature
            return 0
        if args.mode == "capture" and not 2 <= args.seconds <= 120:
            parser.error("capture test duration must be between 2 and 120 seconds")
        with CytonSource(args.port, tuple(args.channels)) as source:
            if args.mode == "capture":
                _capture(args, source)
                return 0
            rest = _collect(source, "Relax with your head level.", args.stage_seconds)
            focused = _collect(source, "Focus with your head level.", args.stage_seconds)
            calibration = calibrate(rest, focused)
            print(f"Calibrated at {calibration.neutral_tilt_deg:+.1f}° neutral, with focus thresholds "
                  f"{calibration.focus.enter:.2f} on / {calibration.focus.exit:.2f} off. Ctrl-C exits.", flush=True)
            controller = Controller(calibration, tilt_sign)
            if args.mode == "monitor":
                print("Preview only. Positive yaw turns left.", flush=True)
                _live(source, controller)
            else:
                from .robot import open_robot

                with open_robot(args.runtime_receipt, args.enable_button) as robot:
                    print("Release the enable button, wait for 'released', then hold it to drive.", flush=True)
                    _live(source, controller, robot)
    except KeyboardInterrupt:
        return 130
    except (csv.Error, EOFError, ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"Pupper BCI: {exc or type(exc).__name__}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
