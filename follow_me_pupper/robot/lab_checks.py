"""The tiny test harness behind lab/test.py.

Nothing here is part of the lab: it builds fake detections and a fake robot
clock, loads follow_me.py without ROS, runs one check at a time, and prints
what each check means when it fails. A check whose code is not written yet
(it raises NotImplementedError) is skipped, not failed, so you can run the
tests after every function you write.
"""

import importlib
import sys
import traceback
import types


class _Position:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _BBox:
    def __init__(self, cx, cy, w, h):
        self.center = type("C", (), {"position": _Position(cx, cy)})()
        self.size_x, self.size_y = w, h


class _Hypothesis:
    def __init__(self, class_id, score):
        self.class_id, self.score = str(class_id), score


class _Result:
    def __init__(self, class_id, score):
        self.hypothesis = _Hypothesis(class_id, score)


class FakeDetection:
    """Quacks like a vision_msgs Detection2D, built from box corners."""

    def __init__(self, x1, y1, x2, y2, class_id=56, score=0.9, color=None):
        self.bbox = _BBox((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)
        self.results = [_Result(class_id, score)]
        self.id = "" if color is None else ",".join(str(c) for c in color)


PASS = FAIL = SKIP = 0


def _unwritten(exc):
    """Name of the function that raised NotImplementedError."""
    if exc.args:
        return exc.args[0]
    frames = traceback.extract_tb(exc.__traceback__)
    return frames[-1].name if frames else "?"


def check(name, got, want=True, tol=None, passes=""):
    """One check. ``got`` is a zero-argument function, so a function you have
    not written yet skips this check instead of stopping the run.

    Passes when got() == want (or within tol); ``want`` may be a function
    too, when working it out needs your code. On a failure it prints what
    your code returned, what was wanted, and an example that passes.
    """
    global PASS, FAIL, SKIP
    try:
        value = got()
        if callable(want):
            want = want()
    except NotImplementedError as exc:
        SKIP += 1
        print(f"  --   {name}  (not written yet: {_unwritten(exc)})")
        return
    except Exception as exc:
        FAIL += 1
        where = traceback.extract_tb(exc.__traceback__)[-1]
        print(f"  FAIL {name}")
        print(f"         your code raised {type(exc).__name__}: {exc}  ({where.name}, line {where.lineno})")
        if passes:
            print(f"         passes, e.g.: {passes}")
        return
    if tol is not None and value is not None and want is not None:
        ok = abs(value - want) <= tol
    else:
        ok = value is want or value == want
    if ok:
        PASS += 1
        print(f"  ok   {name}")
        return
    FAIL += 1
    print(f"  FAIL {name}")
    print(f"         got {value!r}, wanted {want!r}" + (f" (within {tol})" if tol else ""))
    if passes:
        print(f"         passes, e.g.: {passes}")


def run(sections, script, optional=None):
    """Run the named sections (all required ones with no argument), then summarize.

    ``sections`` and ``optional`` map a section name to (title, function).
    Optional sections only run when named.
    """
    optional = optional or {}
    every = {**sections, **optional}
    wanted = sys.argv[1:] or list(sections)
    unknown = [w for w in wanted if w not in every]
    if unknown:
        print(f"Unknown section {unknown[0]!r}. Choose from: {', '.join(every)}")
        raise SystemExit(2)
    for key in wanted:
        title, fn = every[key]
        print(f"{title}  (python3 {script} {key})")
        fn()
        print()
    summary = f"{PASS} passed, {FAIL} failed, {SKIP} not written yet."
    if FAIL:
        print(summary)
        raise SystemExit(1)
    if SKIP:
        print(summary + " Keep going!")
        raise SystemExit(1)
    print(f"All {PASS} checks passed. Take it to the robot.")
    left = [k for k in optional if k not in wanted]
    if not sys.argv[1:] and left:
        print("Optional steps: " + ", ".join(f"python3 {script} {k}" for k in left))


class FakeTime:
    """Quacks like an rclpy Time: subtracting two gives .nanoseconds."""

    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return types.SimpleNamespace(nanoseconds=int(round((self.seconds - other.seconds) * 1e9)))


def fake_node(now=100.0, **fields):
    """Stands in for the StateMachineNode: just the fields your methods read,
    and a clock that says it is `now` seconds."""
    node = types.SimpleNamespace(**fields)
    node.clock_now = FakeTime(now)
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: node.clock_now)
    return node


def load_follow_me():
    """Import lab/follow_me.py, with stand-ins for ROS if it is not installed."""
    try:
        importlib.import_module("rclpy")
    except ImportError:
        for name, attrs in {
            "rclpy": {}, "rclpy.node": {"Node": object}, "rclpy.signals": {"SignalHandlerOptions": object},
            "geometry_msgs": {}, "geometry_msgs.msg": {"Twist": object},
            "std_msgs": {}, "std_msgs.msg": {"String": object},
            "vision_msgs": {}, "vision_msgs.msg": {"Detection2DArray": object},
        }.items():
            module = sys.modules.setdefault(name, types.ModuleType(name))
            for key, value in attrs.items():
                setattr(module, key, value)
    return importlib.import_module("follow_me")


def need(name, value):
    """A constant you have not set yet (still None) skips the check."""
    if value is None:
        raise NotImplementedError(f"{name} is still None")
    return value
