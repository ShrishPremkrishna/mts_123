"""Keep the published state inside one UDP datagram.

The state grew: the prediction, the plans and their explanations are useful on screen but a
datagram has a hard ceiling (macOS refuses anything over 9216 bytes by default, and large
datagrams fragment on Wi-Fi). The window in the perception process always gets the full state.
What goes over the network is this compact copy:

  - the core fields (positions, timestamps, calibration, relative geometry) are passed through
    exactly as they are: other code compares some of them exactly;
  - the newer blocks (prediction, intercepts, machines, diagnostics) have their numbers rounded
    to the millimetre;
  - if that is still too big, the explanations are dropped first (route steps, timelines,
    alternatives), then long lists are thinned, then routes, then the plans themselves.

`_wire` in the result says how much was trimmed, so a consumer can tell.
"""
from __future__ import annotations

import json

COMPACT = ("prediction", "intercepts", "machines", "diagnostics")   # blocks this module may round and trim
WORDS = ("steps", "considered", "timeline", "why", "warnings", "note", "unsure_why", "other_paw")
LIMIT = 8000   # bytes of JSON; the publisher adds a few hundred of its own


def _round(obj, places: int = 3):
    if isinstance(obj, float):
        return round(obj, places)
    if isinstance(obj, dict):
        return {k: _round(v, places) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_round(v, places) for v in obj]
    return obj


def _thin(rows, keep: int):
    """At most `keep` rows, always including the first and the last."""
    if not isinstance(rows, list) or len(rows) <= keep:
        return rows
    step = (len(rows) - 1) / (keep - 1)
    return [rows[round(k * step)] for k in range(keep)]


def _size(state: dict) -> int:
    return len(json.dumps(state, separators=(",", ":")))


def wire_state(state: dict, limit: int = LIMIT) -> dict:
    """A copy of `state` that fits in `limit` bytes of JSON. The input is not changed."""
    out = dict(state)
    for key in COMPACT:
        if out.get(key) is not None:
            out[key] = _round(out[key])
    level = 0
    stages = (_drop_words, _thin_lists, _drop_routes, _drop_plans, _drop_prediction_lists)
    while _size(out) > limit and level < len(stages):
        stages[level](out)
        level += 1
    out["_wire"] = {"trimmed": level}
    return out


def _plans(out: dict):
    for plans in (out.get("intercepts") or {}).values():
        for plan in plans.values():
            if isinstance(plan, dict):
                yield plan


def _drop_words(out: dict) -> None:
    out["intercepts"] = {name: {k: (dict(p) if isinstance(p, dict) else p) for k, p in plans.items()}
                         for name, plans in (out.get("intercepts") or {}).items()}
    for plan in _plans(out):
        for key in WORDS:
            plan.pop(key, None)


def _thin_lists(out: dict) -> None:
    pred = out.get("prediction")
    if pred:
        pred = out["prediction"] = dict(pred)
        pred["path"], pred["trail"] = _thin(pred.get("path"), 12), _thin(pred.get("trail"), 12)
        if pred.get("last_roll"):
            pred["last_roll"] = {**pred["last_roll"], "trail": _thin(pred["last_roll"].get("trail"), 12)}


def _drop_routes(out: dict) -> None:
    for plan in _plans(out):
        plan.pop("path", None)


def _drop_plans(out: dict) -> None:
    out["intercepts"] = {}


def _drop_prediction_lists(out: dict) -> None:
    pred = out.get("prediction")
    if pred:
        out["prediction"] = {**pred, "path": [], "trail": [], "last_roll": None}
