# Notes for coding agents (CS 123, Gemini Pupper lab)

You are helping a Stanford CS 123 student with the Gemini Pupper lab (lab 6).
The student is learning. Explain what you change and why, keep changes small
enough to review, and leave the decisions that are the point of the lab (tool
design, clip design, what to refuse and why) for the student to make with you,
not for you to make alone. The student has to be able to explain every line
they submit: Quiz 2 tests them on this code, without you. After each TODO,
offer to walk them through it line by line, and what would break without each
check. Don't answer the lab's DELIVERABLE questions for them.

## What this repo is

Pupper is a real robot dog: 12 motors, a Raspberry Pi, a camera. Gemini Live
(in the browser, `web/`) talks with people and calls **tools**. The tools are
Python, in `lab/`: the web app gets Gemini's prompt (`lab/prompt.md`) and tool
list from the robot server (`robot/server.py`, port 8000), and sends every tool
call back to it. Each `lab/` file has two halves: behaviors (what the robot
does: check the request, queue a job, return a Result) and the `@tool`s Gemini
calls. `robot/robot_api.py` (provided) runs the jobs one at a time and drives
the ROS 2 controllers. Trick policies are trained in pupper-mjlab and fetched
from W&B with `robot/get_trick.py`.

The student TODOs are numbered 1-11 in the order of the lab handout, each with
a card in the code (now / later / what Gemini hears / must always be true /
tests check / test). "Tests check" lists every check of that TODO, with a reply
or description that passes and one that fails; the handout has the same as a
table. A failing check prints what it got and an example that passes. Work on
one TODO at a time, and run its test group: `python3 lab/test.py todo3`.

| Student work (`TODO n: YOUR CODE HERE`, all in `lab/`) | Provided (don't rewrite) |
| --- | --- |
| `prompt.md`: TODO 1, 6 (its Follow mode section), 11 | `lab/basics.py`, and the finished functions in the part files (`walk`, `play_policy_trick`): students read and explain them, they don't write them |
| `part2_walk.py`: TODO 2 (`check_move`), 3 (the `turn` and `move` tools), 4 (the `arc` tool) | `robot/`: the Robot API (`robot_api.py`), the tool registry, the robot server, the tests, the chat, the launch files and scripts |
| `part3_follow.py`: TODO 5 (`prepare_to_move`), 6 (the `follow` tool) | `web/`: the web app (TypeScript); students don't need to touch it |
| `part4_tricks.py`: TODO 7 (`play_trick`), 8 (its words) | `robot/tricks.py`, `robot/get_trick.py` |
| `part5_interactive_action.py`: TODO 9 (`interactive_action`), 10 (its words, and `tricks/sit_stay_shake/trick.yaml`) | |
| In pupper-mjlab: the Sit_Stay_Shake notebook (or `tricks/sit_stay_shake.py`): `cut`, `hold`, `mirror`, the clips and `ORDER` | The rest of pupper-mjlab |
| In pupper-mjlab (Part 6): the Spider notebook (or `tricks/spider.py`): `reverse`, the clips and `ORDER`; the spider walk's reward weights; then `tricks/spider_walk/trick.yaml` | |

If a provided file looks wrong, tell the student and let them ask a TA; don't
patch infrastructure to make a symptom go away.

## Safety rules (these override everything else)

1. **Never move the robot without the student's explicit OK, right then.**
   That includes `robot/start.sh`, `python3 lab/chat.py --robot`, `curl` to
   any POST endpoint of the robot server, `ros2 topic pub`, `ros2 control switch_controllers`, and anything
   that activates a controller. Say what will happen and wait. The student must
   be at the robot with a hand near the e-stop.
2. **Simulate first.** A new or changed motion goes through the motion report
   and the physics replay (`play_motion.py`, verdict HOLDS UP / WOBBLES /
   FALLS) before it goes anywhere near the robot. A trained policy is scored
   the same way (`play_motion.py --checkpoint`) and watched in the viewer.
3. **New trained policies need a TA's OK** before their first run on the
   robot. A badly trained contact-heavy trick can slam a paw or the body into
   the ground.
4. **Respect the limits.** Don't raise `MoveLimits`, `max_body_angle_deg`,
   joint limits, speed caps or gains to make something "work". Motion design
   reports should show IK error under 2 mm, nothing below the ground, joint
   speeds under about 15 rad/s and no joint at its limit.
5. **Heat.** The motors get hot. If the student says the robot is hot or a
   motor faults, stop testing and let it cool; don't retry in a loop.
6. **No secrets in files.** Never write an API key into the repo, a commit, a
   log or the web app. The Gemini key lives in
   `~/.config/gemini-pupper/api_key` on the robot (and in the browser's
   Settings); W&B uses `wandb login`.

## How to check work without the robot

- `python3 lab/test.py` (or `todo1` ... `todo11`, `part1` ... `part7`,
  `robot`): tests with a fake robot. Run them after every change in `lab/`.
  They check what the tools do, not how they are written.
- `python3 lab/chat.py`: talk to Gemini in text with the student's own prompt
  and tools, on the fake robot (nothing moves). It shows each tool call, the
  reply Gemini hears, and what the fake robot did. Never `--robot` without
  the student's OK (see the safety rules).
- In pupper-mjlab: `uv run python -m mjlab.tasks.tracking.config.pupper.tricks.sit_stay_shake`
- In pupper-mjlab (Part 6): `uv run python -m mjlab.tasks.tracking.config.pupper.tricks.spider`
  checks `cut`, `hold` and `mirror` and builds the library;
  `uv run python -m mjlab.tasks.tracking.config.pupper.play_motion <motion>.npz`
  replays it with physics (add `--checkpoint` to score a trained policy).

## Conventions

- Every tool returns a Result (`{success, message}`), and Gemini reads the
  message word for word: it is all Gemini learns. Say what happened, or what
  was wrong *and* the limit or the real choices (the tests check for these:
  "too fast: |vx|<=1.5 m/s", "unknown trick 'x'; available: [...]"). A bare
  "too fast" leaves Gemini guessing.
- Rules are enforced on the robot, not only in the prompt: the prompt and the
  tool descriptions tell Gemini what to do; the code makes sure it happens.
- Motion goes through the queue (`robot.queue`): check now, queue a job, and
  the job checks again, acts and reports. Wait with `await robot.sleep(...)`,
  never `time.sleep`.
- Units: metres, seconds, degrees. Directions use the robot's convention:
  +y is left, and a positive turn is to the left.
- After changing a `.py` file in `lab/`, restart the server
  (`robot/stop.sh`, then `robot/start.sh`). After changing `prompt.md`,
  reconnect in the app.
- The robot is called Pupper. Gemini Pupper was created by Peng Xu (Google
  DeepMind); keep his credit in the README and the app.
