# Pupper project context

[README.md](README.md) maps the code, setup and available resources. [The project brief](final_project/README.md) separates agreed outcomes from implementation status. Run `python3 foundation/scripts/session_status.py` for the dated task and access checkpoint. The user's current request sets the task. Earlier plans remain available as historical references.

## Working approach

- Infer a useful role from the current request, source evidence and team context. Team ownership guides coordination, not the scope of help you can offer; handoffs are optional working notes.
- State consequential assumptions and define an observable result. Resolve reversible details with judgment and ask when the answer changes the goal or required access.
- Prefer understandable solutions sized to the task. Existing architecture is a starting point and may change when evidence or the goal warrants it.
- **Speed.** Shorten time to a verified outcome. Fix recurring tooling friction, parallelize independent work and use Claude Code or focused subagents where they help. Give concurrent editors clear file ownership.
- **Altitude.** Reassess the outcome behind the immediate task. Challenge premises constructively and propose a better direction when evidence or the user's goal warrants it.
- **Depth.** Examine source data and underlying mechanisms. Use appropriate instruments and independent checks to challenge assumptions and verify consequential claims.
- **Range.** Draw on relevant disciplines and compare promising alternatives through concrete experiments. Keep exploratory ideas distinguishable from implemented behavior.
- Update the smallest useful checkpoint when facts change. Include dates, source identities, evidence and unresolved questions. Load detailed historical records when relevant.

These principles adapt the [Karpathy-inspired coding guide](https://github.com/multica-ai/andrej-karpathy-skills/blob/main/CLAUDE.md) and [Rieseberg's ambition framework](https://felixrieseberg.com/how-to-be-more-ambitious-a-tutorial-for-humans-with-models/).

## Access and physical effects

`main` is the shared integration branch. Package-specific agent files explain local interfaces and commands. Their sibling `CLAUDE.md` files import the same guidance.

The Mac environments are `.venv-api` for the API and `.venv` for CPU simulation. Setup and verification commands are linked from the README. Keep credentials external to source, logs and shared artifacts.

Robot sessions have one operator coordinating camera, motion and services. Check current ownership and readiness before hardware operations. The maintained `start-native` step can home the legs. The [hardware procedure](foundation/HARDWARE_CHECKLIST.md) records command effects, shutdown and release selection. Simulate changed motion before supervised use and obtain applicable course approval for new policies. Preserve joint/current limits unless an explicitly reviewed change warrants different values.

Distinguish source checks, staged files, controller receipts and observed physical outcomes. Identify the actual running release when recording hardware evidence.
