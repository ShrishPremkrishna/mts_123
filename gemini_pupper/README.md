# Gemini Pupper

Pupper is the Stanford CS 123 robot dog. Gemini Live hears and sees through the browser app and moves the robot only through server-validated Python tools.

Gemini Pupper was created by [Peng Xu](https://scholar.google.com/citations?user=460NWeQAAAAJ&hl=en) (Google DeepMind). See the original repository [sippeyxp/gemini-pupper](https://github.com/sippeyxp/gemini-pupper) and the write-up [Interactive Robot Dog with Gemini Live](docs/interactive-robot-dog.md).

## Code map

Tools are `@tool` functions in `lab/part*.py`, imported by `robot/tool_registry.py` through `LAB_MODULES` and served at `GET /tools` and `POST /tools/<name>`. See [System architecture](docs/architecture.md).

## Course TODOs

TODO 1–11 are [Lab 6](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/lab-6.html) lesson references already implemented on `main`. TODO 1, 6 and 11 are in `lab/prompt.md`. The rest are in `lab/part2_walk.py` through `lab/part5_interactive_action.py`, plus `tricks/sit_stay_shake/trick.yaml`. Student attribution and robot observations stay as recorded. The sibling [pupper-mjlab](../pupper-mjlab/) package trains Sit_Stay_Shake and Spider, and final project work goes in `../final_project/`.

## Offline checks

Run these from the monorepo root. They use a fake robot.

```bash
.venv-api/bin/python gemini_pupper/lab/test.py todo3
.venv-api/bin/python -m pytest -q gemini_pupper/tests/test_ordered_follow.py
npm --prefix gemini_pupper/web test
```

`lab/chat.py` pairs a fake robot with the real Gemini provider. Run it only with authorization and an external API key kept out of logs.

## Robot operation

Follow [the hardware checklist](../foundation/HARDWARE_CHECKLIST.md) as the canonical procedure and read the [current session record](../foundation/results/interruption-recovery.json) before using the robot. Staging copies a release, and explicit activation makes it the runtime. Never assume source, staged and loaded versions match.

Prepare, status and app-only steps do not home the robot. `start-native` may home, so put the robot on the stand first. An operator stays present with R3 and power within reach. Check changed motions in physics replay and the viewer. Newly trained policies require course TA approval before their first hardware use. Keep limits and gains, and stop on hot motors or a fault.

Stop interrupts queued and in-flight work and requests zero velocity. Terminal job and controller state come from the robot, and the operator confirms observed motion. R3 latches a stop. Motor release and full driver shutdown are separate supervised steps.

For technical background, see the [foundation reference](docs/foundation.md). The [archived package instructions](https://github.com/ShrishPremkrishna/mts_123/blob/6a6edb0bab0c4d36e14a32c7757f98c3966c10cb/gemini_pupper/README.md) and [original course agent notes](https://github.com/ShrishPremkrishna/mts_123/blob/6a6edb0bab0c4d36e14a32c7757f98c3966c10cb/gemini_pupper/AGENTS.md) preserve the stand-alone setup, exercise map, and motion design thresholds.
