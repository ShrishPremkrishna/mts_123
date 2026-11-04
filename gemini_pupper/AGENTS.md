# Gemini Pupper agent notes

Use the [shared working agreement](../AGENTS.md) for repo-wide guidance.

## Coursework

- TODO 1–11 in `lab/` are lesson references already implemented on `main`. Keep student attribution and robot observations as recorded. Base submission claims on recorded results. Obtain missing personal reflections or unrecorded observations from the student.
- For harness work, edit the relevant behavior in `lab/`, server component in `robot/`, or browser component in `web/`.
- Keep Peng Xu's credit and the course design context.

## Tools and motion

- Add a `@tool` function to a module listed in `LAB_MODULES` in `robot/tool_registry.py`. The server lists and runs it through `GET /tools` and `POST /tools/<name>` with no new server or web routing.
- Finite actions run as queued jobs that check admission on request and robot state at execution.
- Status queries run immediately. Stop bypasses the queue, interrupts active work and requests zero velocity.
- Send one success acknowledgement per spoken action request, after the server accepts it. Use the server’s refusal or failure when a request fails.
- Report terminal job and controller state as authoritative, and report physical motion only as observed.
- Preserve receipts and stale-instance and command-epoch protection. Keep API keys external and out of logs.
- Course units are m/s, seconds, degrees and deg/s. Native ROS angular velocity is rad/s.

## Checks and hardware

Run the [offline checks](README.md#offline-checks). Follow [Robot operation](README.md#robot-operation) for physical guardrails and [the workstreams doc](../foundation/WORKSTREAMS.md) for lab handoff.
