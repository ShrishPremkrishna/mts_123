# CS123 Pupper

This is the team's shared repository, [ShrishPremkrishna/mts_123](https://github.com/ShrishPremkrishna/mts_123), with `main` as the integration branch. The local checkout is `/Users/shrish/workspace/cs123`.

The final project is **Pupper FC**, two autonomous soccer-playing Puppers using overhead vision. Start with [the project brief](final_project/README.md) and `python3 foundation/scripts/session_status.py`. The checkpoint records dated observations. Fresh robot access, service ownership and teammate progress require verification for the current session.

## Code map

| Component | Location and current role |
| --- | --- |
| Voice and tool harness, including Lab 6 | [gemini_pupper](gemini_pupper/README.md). `web/` supplies conversation and operator controls, `robot/` hosts the API and execution queue, `lab/` defines tools and prompts, and `tricks/` contains selected skills. |
| Person detection and following, including Lab 5 | [follow_me_pupper](follow_me_pupper/README.md), with camera calibration, Hailo inference, tracking and ROS velocity publication. |
| Gait and trick training | [pupper-mjlab](pupper-mjlab/README.md), with simulation tasks, reward configurations, policy export and evaluation. |
| Native policy deployment | [pupper_gait_deploy](pupper_gait_deploy/README.md), with ROS controllers, observation/action conventions and deployment helpers. |
| Early labs | [PD control](pd_control_lab/README.md), [forward kinematics](forward_kinematics_lab/README.md), [IK walking](ik_heuristic_walking_lab/README.md), and [implicit compliance](implicit_compliance/README.md). |
| Platform and supporting tools | [pupperv3-monorepo](pupperv3-monorepo/README.md) and [pupper_rebase](pupper_rebase/README.md) hold robot platform sources. [UniMate](UniMate/README.md) generates motion references; [pupper-gait-tuner](pupper-gait-tuner/README.md) supports gait authoring. |
| Final project | [Pupper FC](final_project/README.md). Current outcomes, ownership, plan and teammate handoffs. As of November 12 this folder contains documentation; soccer gameplay and self-righting are planned capabilities. |
| Evidence, releases and lab submission | [foundation](foundation/README.md), including [the Lab 6 writer packet](foundation/submission/lab6/README.md) and [hardware procedures](foundation/HARDWARE_CHECKLIST.md). |

The harness currently dispatches described tools through a server queue and ROS bridge. Native controllers execute policies on the robot. The person follower supplies velocity through its own ROS source. Training produces assets consumed by these controllers. Exact interfaces and units are in the relevant package sources and agent notes. These are descriptions of the implemented system, open to revision as project requirements develop.

The platform folders are ordinary vendored directories in this Git repository. The legacy `/Users/shrish/workspace/cs123-foundation` worktree retains an older engineering branch and local evidence. Historical plans and evidence include source versions so they can be interpreted in context.

## Development and resources

| Resource | Access and scope |
| --- | --- |
| Mac API and browser | [Setup and fake-robot checks](gemini_pupper/README.md#offline-checks), `.venv-api/bin/python`, and npm in `gemini_pupper/web`. |
| Mac simulation and native checks | [CPU environment and offline checks](foundation/README.md). `.venv/bin/python` imports this checkout's `pupper-mjlab`. The offline ROS runner uses Docker Desktop. GPU training uses a suitable CUDA host. |
| Claude Code and subagents | Claude CLI is installed on this Mac. Prior authenticated source-only work is recorded; check availability in the current session. Available agent/browser tools depend on the host application. |
| Gemini | Existing local credential location is `~/.config/gemini-pupper/api_key`. Onboard setup uses an explicit key file and a one-use browser bootstrap. Verify access without printing credentials. Recorded spending preference requires confirmation before an API experiment expected above $5. |
| Pupper | `ssh mts` uses the configured SSH alias and Tailscale. The November 4 source checkout observation was `/home/pi/mts_123`; the selected R6 app was staged in a separate release directory. [Release identities and observation dates](foundation/results/interruption-recovery.json). |
| Class library | Local Obsidian course folder is `/Users/shrish/Library/Mobile Documents/iCloud~md~obsidian/Documents/vaulted/Fall2026Classes/Cs123`. Start at `CS123.md` or `References/Course Sources.md`. It contains dated handouts, slides, notebooks, source captures and learning notes. [Live course site](https://cs123-stanford.readthedocs.io/en/latest/). |
| Experiment archives | Ignored `artifacts/` currently links into the legacy worktree. Additional archives are in `/Users/shrish/workspace/pupper-foundation-artifacts`. Retrieve the specific evidence named by a report. |

The API, browser, CPU and offline ROS setup passed checks on November 4. The checkpoint records those results. Cloud runtime availability and physical readiness are session-specific.

## Fresh clone

```sh
GIT_LFS_SKIP_SMUDGE=1 git clone --branch main https://github.com/ShrishPremkrishna/mts_123.git
```

This leaves the vendored platform's large LFS assets as pointers. Materialize assets needed for the component being run. Selected Spider weights are tracked normally. The Pi also uses its provisioned native runtime, the [pinned Hailo model](follow_me_pupper/scripts/download_model.sh) and a built browser bundle. Source checkout, app staging and hardware activation are distinct steps.
