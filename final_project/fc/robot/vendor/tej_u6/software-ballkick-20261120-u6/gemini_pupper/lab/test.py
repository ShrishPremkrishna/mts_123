#!/usr/bin/env python3
"""Run the desk tests (a fake robot: no Pupper, no ROS, no Gemini needed).

    python3 lab/test.py            # everything
    python3 lab/test.py todo3      # one TODO (todo1 ... todo11)
    python3 lab/test.py part2      # one part of the lab (part1 ... part7; part6, the spider, is the notebook)
    python3 lab/test.py robot      # the provided robot (should always pass)

The tests themselves are in robot/test_tools.py.
"""
import runpy
from pathlib import Path

runpy.run_path(str(Path(__file__).resolve().parent.parent / "robot" / "test_tools.py"), run_name="__main__")
