# Facilitator Guide — Live Gait Tuning (20 min, 2 robots)

**Goal:** students discover the **speed ↔ stability tradeoff** by tuning a real robot's
walk, and see that a walk is "just a few numbers."

**Setup before students arrive**
- Each robot: ros2_control stack up, `python3 main.py` running, browser on
  `http://<pi-ip>:8080` shown on a screen/projector per team.
- Mark a **start/finish line ~2 m apart** on the floor per team.
- Robot on a stand or held until a team is ready to run.
- Sanity-check defaults walk on each robot once.

**Two teams of ~10, one robot each.** Rotating sub-roles keep hands busy:
*driver* (sliders), *scribe* (records params + run times), *spotter* (catches the robot).

## Run of show

| Time | What |
|------|------|
| 0–3 min | Hook: "A robot's walk is just a few numbers." Drag **step length** live on the projector — watch the foot paths grow and the robot speed up, then over-crank it until it stumbles. Pose the question: *bigger steps or faster steps?* |
| 3–5 min | Split into 2 teams. Explain the course (start→finish, fastest **stable** crossing wins) and the roles. "Send to robot" goes on once the spotter is ready. |
| 5–16 min | **Timed rounds.** Each round: tweak → run the course → scribe records time + the params. Encourage changing *one* slider at a time so they can reason about cause/effect. Nudge teams toward the tradeoff: who can go fastest *without* falling? |
| 16–19 min | **Head-to-head race** with each team's best stable gait. |
| 19–20 min | Debrief: what was the fastest stable combo, and *why*? (Long stride + high cadence = fast but tippy; some duty/step-height keeps it planted.) |

## Guiding questions
- Which single slider made the biggest difference to speed? To stability?
- Why does the robot stumble when step length gets too big?
- What does **duty** (time on the ground) do — and which gait pattern feels most stable?
- If you could only change two numbers to win the race, which two?

## Differentiation (mixed group)
- *Newer students*: stick to step length + frequency; read body speed off the panel.
- *Advanced*: switch **gait pattern** (trot/walk/pace/bound), tune **duty**, or open
  `gait.py` and look at how a foot trajectory is built / how IK finds the angles.

## Troubleshooting
- **Robot tips over** → "Stand (reset pose)", lower step length / raise duty / widen stance.
- **Page won't load** → confirm `<pi-ip>` and that `main.py` is running; same Wi-Fi.
- **Motors don't move** → "Send to robot" ticked? Is the controller stack up
  (`ros2 topic list` shows `/forward_command_controller/commands`)?
- **Jerky on enable** → expected ramp is ~1.2 s; don't toggle send rapidly.
