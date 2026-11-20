"""Run from repo root with .venv-api/bin/python -m final_project.pupper_fc."""
import argparse
import json
import math
from pathlib import Path

from .runtime import DT, Sandbox
from .simulation import SCENARIOS


def main():
    parser = argparse.ArgumentParser(description="Offline approach-and-shot practice; no robot connection")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--scenario", choices=[*SCENARIOS, "all"], default="center")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--headless", action="store_true", help="Run deterministic trials without a web server")
    shot = parser.add_mutually_exclusive_group()
    shot.add_argument("--kick-side", choices=("left", "right"),
                      help="Use a measured-pose foot approach and simulated kick receipts")
    shot.add_argument("--legacy-run-through", action="store_true",
                      help="Open the historical bumper-strike rehearsal")
    parser.add_argument("--seconds", type=float, default=40)
    parser.add_argument("--output", type=Path, help="Write headless summary JSON to a new file")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not math.isfinite(args.seconds) or not 0 < args.seconds <= 600:
        parser.error("port must be 1..65535 and seconds must be >0 and <=600")
    if not 0 <= args.seed < 2**32:
        parser.error("seed must be 0..4294967295")
    if args.headless:
        results = []
        for scenario in SCENARIOS if args.scenario == "all" else [args.scenario]:
            sandbox = Sandbox(scenario, args.seed, kick_side=args.kick_side)
            sandbox.control({"action": "start"})
            for _ in range(math.ceil(args.seconds / DT)):
                sandbox.advance()
                if sandbox.decision.phase in ("await_reset", "kick_complete"):
                    break
            results.append(dict(scenario=scenario, seed=args.seed, **sandbox.metrics,
                                phase=sandbox.decision.phase,
                                reason=sandbox.decision.reason))
        report = dict(scope="Approximate 2D simulation only; not physical performance",
                      kick_side=args.kick_side, trials=results)
        data = json.dumps(report, indent=2)
        if args.output:
            with args.output.open("x") as handle:
                handle.write(data + "\n")
        print(data)
    else:
        if args.scenario == "all":
            parser.error("--scenario all requires --headless")
        if args.output:
            parser.error("--output requires --headless")
        import uvicorn
        from .server import create_app
        print(f"Pupper FC offline practice: http://127.0.0.1:{args.port}", flush=True)
        side = None if args.legacy_run_through else (args.kick_side or "left")
        uvicorn.run(create_app(Sandbox(args.scenario, args.seed, kick_side=side)), host="127.0.0.1",
                    port=args.port, access_log=False)


if __name__ == "__main__":
    main()
