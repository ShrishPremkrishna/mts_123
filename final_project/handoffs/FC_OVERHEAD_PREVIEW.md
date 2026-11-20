# Overhead appearance preview

Use `--appearance-preview` with `final_project/overhead_perception/scripts/run_perception.py` to view anonymous robot centers. The existing live match stream still uses its established pose sources. The team-color experiment estimates identity from a colored body patch and signed heading from a separate nose marker.

The anonymous detector found all six labeled centers across eight archived views. No robot in those recordings wore the proposed color patches. The digital patch controls exercise the software. For a replay with the original images, both `scripts/benchmark_appearance.py` and `scripts/benchmark_team_colors.py` under `final_project/overhead_perception` take `--phone-dir` and `--pose-window`. Run `python -m pytest final_project/overhead_perception/tests` for the checked-in regression suite.

Before using color poses for match commands, record both marked teams crossing under the mounted camera. Label center, team and nose direction through occlusion and under field lighting. Measure identity swaps, heading reversals, observation gaps and capture-to-output age. Use the robot tags for reference during that comparison.
