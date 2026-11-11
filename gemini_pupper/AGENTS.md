# Gemini Pupper

Shared principles are in the [root AGENTS.md](../AGENTS.md). This package began as Peng Xu's course starter and is now implemented and extended. Classroom TODO and do-not-edit comments describe the original starter exercise.

- `lab/`: tool functions and the agent prompt.
- `robot/`: tool registry, HTTP API, job queue and native ROS bridge.
- `web/`: browser client.
- `tricks/`: trick assets.

`LAB_MODULES` in `robot/tool_registry.py` controls which modules have their `@tool` functions registered. The server lists them at `GET /tools` and runs them at `POST /tools/<name>`. In the current implementation, finite actions are queued and Stop bypasses the queue. The current voice client gives one acknowledgement per spoken action request after server acceptance, and reports refusals or later failures. Receipts describe acceptance and controller execution; physical outcomes come from observation.

Course units are m/s, seconds, degrees and degrees/s. Native ROS yaw rate is rad/s.

The README covers [Offline checks](README.md#offline-checks) and [Robot operation](README.md#robot-operation). Lab 6 write-up material goes through [the Lab 6 writer handoff](../foundation/submission/lab6/README.md).
