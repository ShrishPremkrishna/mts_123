<div align="center">
<img width="1200" height="475" alt="GHBanner" src="https://github.com/user-attachments/assets/0aa67016-6eaf-458a-adb2-6e31a0763ed6" />
</div>

# Gemini Pupper

Talk to Pupper, and it listens, looks and does what you ask. Gemini Live is
the brain: a foundation model that hears you, sees through the camera and
talks back. Trained policies are the muscles. In between are the tools: the
only way Gemini can move the robot, and the only way it learns what really
happened. In CS 123, Lab 6 (*Bringing Your Pupper to Life*), you write them.

**Gemini Pupper was created by [Peng Xu](https://scholar.google.com/citations?user=460NWeQAAAAJ&hl=en)
(Google DeepMind)**, original repo [sippeyxp/gemini-pupper](https://github.com/sippeyxp/gemini-pupper).
This is the CS 123 version: tools in Python, trick policies, and safety checks.

## What's where

You only work in `lab/`. Everything else is provided.

```
gemini_pupper/
├── lab/                         ← you work here
│   ├── prompt.md                who Pupper is and how it behaves        TODO 1, 6, 11
│   ├── basics.py                read first: finished examples, nothing to write
│   ├── part2_walk.py            check_move, the turn and move tools, arc  TODO 2, 3, 4
│   ├── part3_follow.py          the follow rule, then the follow tool     TODO 5, 6
│   ├── part4_tricks.py          play_trick, then its words                TODO 7, 8
│   ├── part5_interactive_action.py  interactive_action, then its words              TODO 9, 10
│   ├── test.py                  python3 lab/test.py todo3   (tests with a fake robot)
│   └── chat.py                  python3 lab/chat.py         (talk to Gemini in text)
├── tricks/                      one folder per trick; sit_stay_shake lands here in Part 5
├── robot/                       provided: the Robot API, the robot server, launch files, scripts
├── web/                         provided: the web app (TypeScript); you never need it
└── docs/
```

Every file in `lab/` has the same two halves, in order. First **what the robot
does** (a behavior: is it allowed, do it, report what happened), then **what
Gemini reads** (an `@tool` with the tool's name, its description, and a short
body). The TODOs are numbered 1–11 in the order of the lab handout. Each one
has a short card in the code: what happens now, what happens later, what Gemini
hears, what must always be true, and which test checks it. The finished
functions (`stand_up`, `walk`, `play_policy_trick`, …) are there to read: the
lab asks you to explain them.

The Robot API you call (`robot.queue`, `robot.switch_to`, …) is listed at the
top of `robot/robot_api.py`.

## Setup (once, on the Pupper)

```bash
git clone https://github.com/<you>/gemini_pupper.git ~/gemini_pupper
cd ~/gemini_pupper
robot/setup.sh
```

`setup.sh` installs Node in your home folder, builds the web app, checks the
Python packages and asks for your Gemini API key (from
[Google AI Studio](https://aistudio.google.com/apikey)). The key is saved in
`~/.config/gemini-pupper/api_key`, never in this repo. Run `setup.sh` again
any time; it is safe.

The Pupper must also be logged into W&B (`wandb login`) with the account you
train tricks with.

## Running

On the Pupper, on the ground, with clear space around it:

```bash
robot/start.sh --kiosk    # robot + the app on Pupper's own screen (uses its mic and speaker)
robot/start.sh            # robot only; open the app from your laptop (below)
robot/stop.sh             # relax the motors and stop everything
```

From your laptop: `ssh -N -L 8000:localhost:8000 pi@pupper<N>.local`, then
open http://localhost:8000, open **Settings**, paste your key and press
**WAKE PUPPER**. The debug view shows every tool call and its result.

The e-stop is the gamepad's right stick (press it): it relaxes the motors,
whatever Gemini is doing.

## Talk to Gemini in text

`lab/chat.py` (based on Peng Xu's Gemini Live example) runs the same loop as
the web app, in your terminal: you type, Gemini calls tools, and you see every
call and every reply. It needs `python3 -m pip install --user google-genai pyyaml`
(`setup.sh` installs it on the Pupper) and your key in `GEMINI_API_KEY` or
`~/.config/gemini-pupper/api_key`.

```bash
python3 lab/chat.py            # your prompt and tools, on a fake robot: nothing moves
python3 lab/chat.py --robot    # your tools on the real Pupper (through the robot server): it moves!
```

## After you change something

| You changed | Then |
| --- | --- |
| `lab/prompt.md` | try it in text (`python3 lab/chat.py`), then reconnect in the app |
| a `.py` file in `lab/` | `python3 lab/test.py`, try it in text, then `robot/stop.sh && robot/start.sh` |
| a trick (`get_trick.py` or a `trick.yaml`) | `robot/stop.sh && robot/start.sh` |

## Tricks

Every trick is played by policies, so the robot is always in one simple state: locomotion,
one trick (trick mode), motors idle, or fallen. Each folder under `tricks/` with a `trick.yaml` is a
trick Gemini can play (`robot/tricks.py` lists the fields). Tricks are trained
in [pupper-mjlab](https://github.com/cs123-stanford/pupper-mjlab); training
puts the policy on W&B, and one command fetches it:

```bash
python3 robot/get_trick.py <entity>/mjlab/<run-id> --name my_trick
robot/stop.sh && robot/start.sh
```

A playback trick plays one reference motion from start to finish. An
interactive trick is one Pupper stays in and that takes requests: `actions` in
its `trick.yaml` lists the clips Gemini may ask for (with the `interactive_action`
tool), and `exit_clip` is the clip that stands back up.

## More

- [Peng Xu's write-up: Interactive Robot Dog with Gemini Live](docs/interactive-robot-dog.md)
- [System architecture](docs/architecture.md)
