# pupper-mjlab

This is a fork of mjlab used to train and evaluate Pupper gait policies in simulation. Shared principles are in the [root AGENTS.md](../AGENTS.md).

- `src/mjlab/tasks/pupper_gait/`: Pupper gait task.
- `src/mjlab/tasks/tracking/config/pupper/`: Pupper tracking configuration.
- `tests/`: test suite.
- `notebooks/`: course workflows; `lab4_runs/`: team run definitions and records.

Dependencies are declared in `pyproject.toml` and pinned in `uv.lock`. The numerical environment is the team-root `.venv`, provisioned as described in [the foundation setup reference](../foundation/README.md). On macOS it runs on CPU. GPU training needs CUDA.

The upstream README's commands are scoped to this training root. Running them may provision or change dependencies, so check what a command does before running it against the shared environment. Simulation results are separate evidence from behavior on the physical robot.
