# CS123 Pupper

This is the team's shared repository, [ShrishPremkrishna/mts_123](https://github.com/ShrishPremkrishna/mts_123), with `main` as the integration branch. Shrish's checkout is `/Users/shrish/workspace/cs123`; other clones use their own repository root.

The final project is **Pupper FC**, two autonomous soccer-playing Puppers using overhead vision. Current work is [final_project/fc](final_project/fc/STATUS.md) on branch `fc-v1`. Start with [the project brief](final_project/README.md) and `python3 foundation/scripts/session_status.py`. The checkpoint records dated observations. Fresh robot access, service ownership and teammate progress require verification for the current session.

## Code map

| Component | Location and current role |
| --- | --- |
| Voice and tool harness, including Lab 6 | [gemini_pupper](gemini_pupper/README.md). `web/` supplies conversation and operator controls, `robot/` hosts the API and execution queue, `lab/` defines tools and prompts, and `tricks/` contains selected skills. |
| Person detection and following, including Lab 5 | [follow_me_pupper](follow_me_pupper/README.md), with camera calibration, Hailo inference, tracking and ROS velocity publication. |
| Gait and trick training | [pupper-mjlab](pupper-mjlab/README.md), with simulation tasks, reward configurations, policy export and evaluation. |
| Native policy deployment | [pupper_gait_deploy](pupper_gait_deploy/README.md), with ROS controllers, observation/action conventions and deployment helpers. |
| Early labs | [PD control](pd_control_lab/README.md), [forward kinematics](forward_kinematics_lab/README.md), [IK walking](ik_heuristic_walking_lab/README.md), and [implicit compliance](implicit_compliance/README.md). |
| Platform and supporting tools | [pupperv3-monorepo](pupperv3-monorepo/README.md) and [pupper_rebase](pupper_rebase/README.md) hold robot platform sources. [UniMate](UniMate/README.md) generates motion references; [pupper-gait-tuner](pupper-gait-tuner/README.md) supports gait authoring. |
| Final project (current) | [final_project/fc](final_project/fc/STATUS.md) on branch `fc-v1`: one laptop process (overhead camera → brain → signed UDP) drives the stock walker through a small Pi bridge. Status in [STATUS.md](final_project/fc/STATUS.md), plan in [DEMO.md](final_project/fc/DEMO.md), usage in [README.md](final_project/fc/README.md). |
| Final project (historical) | Pre-Nov 23 FC stacks were removed from the tree on 2026-11-24 and are preserved at tag `archive/pre-fc-v1-cleanup` (superseded agent branches at other `archive/*` tags). Still present: [overhead perception](final_project/overhead_perception/README.md) (camera pipeline used by `fc/eye.py`) and the [pupper_fc](final_project/pupper_fc/README.md) / [behaviors](final_project/behaviors/README.md) library remnant imported by `gemini_pupper`'s `fc_walk_kick` mode. |
| Evidence, releases and lab submission | [foundation](foundation/README.md), including [the Lab 6 writer packet](foundation/submission/lab6/README.md) and [hardware procedures](foundation/HARDWARE_CHECKLIST.md). |

The harness currently dispatches described tools through a server queue and ROS bridge. Native controllers execute policies on the robot. The person follower supplies velocity through its own ROS source. Training produces assets consumed by these controllers. Exact interfaces and units are in the relevant package sources and agent notes.

The platform folders are ordinary vendored directories in this Git repository. Shrish's legacy `/Users/shrish/workspace/cs123-foundation` worktree retains an older engineering branch and local evidence.

## Development and resources

Machine-specific paths and installed-tool observations below describe Shrish's Mac; local development checks last ran November 12, 2026. On another machine, verify available resources and use the setup recipes rather than assuming these paths or credentials exist.

| Resource | Access and scope |
| --- | --- |
| Mac API and browser | [Setup and fake-robot checks](gemini_pupper/README.md#offline-checks), `.venv-api/bin/python`, and npm in `gemini_pupper/web`. |
| Mac simulation and native checks | [CPU environment and offline checks](foundation/README.md). `.venv/bin/python` imports this checkout's `pupper-mjlab`. The offline ROS runner uses Docker Desktop; that Ubuntu image is test-only for the Debian Pi. Build deployment binaries against the target environment. GPU training uses a suitable CUDA host. |
| Claude Code and subagents | CLI 2.1.308 and exact `claude-fable-5-2` were verified November 17. Lean authenticated review invocation and measured startup are in the [diagnostic](artifacts/pupper-fc/2026-11-17/fable-latency/summary.json). Available agent/browser tools depend on the host application. |
| Gemini | Existing local credential location is `~/.config/gemini-pupper/api_key`. Onboard setup uses an explicit key file and a one-use browser bootstrap. Verify access without printing credentials. Confirm before any API experiment expected above $5. |
| Pupper | `ssh mts` uses Shrish's configured SSH alias and Tailscale. [The session checkpoint](foundation/results/interruption-recovery.json) holds access constraints and dated release observations. |
| Class library | Local Obsidian course folder is `/Users/shrish/Library/Mobile Documents/iCloud~md~obsidian/Documents/vaulted/Fall2026Classes/Cs123`. Start at `CS123.md` or `References/Course Sources.md`. It contains dated handouts, slides, notebooks, source captures and learning notes. [Live course site](https://cs123-stanford.readthedocs.io/en/latest/). |
| Experiment archives | Ignored `artifacts/` currently links into the legacy worktree. Additional archives are in `/Users/shrish/workspace/pupper-foundation-artifacts`. Retrieve the specific evidence named by a report. |

Cloud runtime availability and physical readiness are session-specific. Ordinary source work does not require the historical artifact archive or a robot connection.

For bounded Fable source reviews, use the exact model and the lean invocation
recorded in the diagnostic, with an explicit review system prompt and authorized
source on stdin. Safe
mode removed startup hooks while retaining existing sign-in; `--bare` disables
that authentication route. Retain bounded progress and final model/exit evidence
in files, not every streamed token in chat. A 95.6-second focused source/math
review succeeded; tiny connectivity probes do not predict review duration.

### Choose the check for the task

Run these from the repository root after the one-time setup linked above. Select the relevant lane; broad regression is available in the foundation reference when a change warrants it.

| Work | Focused offline check |
| --- | --- |
| Pupper FC (`final_project/fc`) | `cd final_project && ../.venv-fc/bin/python -m pytest -q fc/tests fc/robot/tests` (`fc/run.sh` creates `.venv-fc`) |
| API/tool behavior | `.venv-api/bin/python gemini_pupper/lab/test.py todo3` and `.venv-api/bin/python -m pytest -q gemini_pupper/tests/test_ordered_follow.py` |
| Browser | `npm --prefix gemini_pupper/web test` |
| CPU motion/model contracts | `FORCE_CPU=1 .venv/bin/python -m pytest -q pupper-mjlab/tests/test_pupper_foundation.py` |
| Native controller math/reference behavior | `bash pupper_gait_deploy/test/run_host_test.sh` — no ROS or robot |
| Linux ROS tests | [Offline ROS runner](foundation/README.md#local-numerical-setup), using the test-only Docker image; Pi side of fc: [fc/robot/README.md](final_project/fc/robot/README.md) |

These check different layers; passing them does not establish physical performance. Reuse the checkout-local environments. Reinstall or upgrade when a concrete dependency issue or chosen change requires it. For API/browser implementation, choose the specific tests nearest the changed behavior beyond the examples above.

## Fresh clone

```sh
GIT_LFS_SKIP_SMUDGE=1 git clone --branch main https://github.com/ShrishPremkrishna/mts_123.git
```

This leaves the vendored platform's large LFS assets as pointers. Materialize assets needed for the component being run. Selected Spider weights are tracked normally. The Pi also uses its provisioned native runtime, the [pinned Hailo model](follow_me_pupper/scripts/download_model.sh) and a built browser bundle. Source checkout, app staging and hardware activation are distinct steps.
