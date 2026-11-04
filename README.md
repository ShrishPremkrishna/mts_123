# CS123: Pupper

Use **main** for shared class and final-project work. Make changes on short-lived branches and merge ready work here. Lab 6 stays in its original package, so its imports, notebooks, and robot release paths keep working.

| Work | Start here |
| --- | --- |
| Final project: soccer workstreams, interfaces and first field tests | [Final project](final_project/README.md) |
| Lab 6: voice agent and robot tools | [Lab 6 package](gemini_pupper/README.md) · [assignment code](gemini_pupper/lab/) · [server and tool registry](gemini_pupper/robot/) · [voice client](gemini_pupper/web/) |
| Lab 5: camera, object detector and person follower | [follow_me_pupper](follow_me_pupper/) |
| Lab 4: gait training and deployment | [pupper-mjlab](pupper-mjlab/) · [team training handoff](pupper-mjlab/lab4_runs/shrish_stablegait/) · [native gait runtime](pupper_gait_deploy/) |
| Labs 1–3 | [PD control](pd_control_lab/) · [forward kinematics](forward_kinematics_lab/) · [IK walking](ik_heuristic_walking_lab/) |
| Reliable robot sessions, release tools and Lab 6 submission status | [foundation](foundation/README.md) · [robot checklist](foundation/HARDWARE_CHECKLIST.md) · [submission handoff](foundation/WORKSTREAMS.md) |

The top-level repository is the team monorepo. [pupperv3-monorepo](pupperv3-monorepo/) and [pupper_rebase](pupper_rebase/) contain robot platform code; [pupper-gait-tuner](pupper-gait-tuner/) and [UniMate](UniMate/) are supporting tools. Put new soccer perception, motion adapters, and game behaviors under [final_project](final_project/), importing shared code from its existing package. Do not fork a second copy of the Lab 6 app.

**Getting oriented:** run `python3 foundation/scripts/session_status.py` from the repository root. It does not contact the robot. A clone contains source and the [selected Spider policies](gemini_pupper/tricks/spider_walk/README.md). The Pi also needs its provisioned native runtime, the [pinned detector model](follow_me_pupper/scripts/download_model.sh), and the built web app. Installing source and starting a hardware session are separate operations. Keys, environments, recordings and experiment archives stay outside Git.
