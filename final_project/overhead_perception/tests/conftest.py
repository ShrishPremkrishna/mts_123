"""Make the tests importable from the package folder and from the repo root.

Some tests import `fieldcam...` (package folder on the path) and some import
`final_project.overhead_perception...` (repo root on the path). Putting both on
the path lets `pytest tests` here and `pytest final_project/overhead_perception`
from the repo root collect the same tests.
"""
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent
REPO_ROOT = PACKAGE.parents[1]
for path in (PACKAGE, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
