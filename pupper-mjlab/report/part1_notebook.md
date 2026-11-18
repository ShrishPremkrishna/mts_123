<!-- RECOVERY GAP: lines 1-88 of the b916f17 version are not in any W&B diff. -->
The backward reference is correct (checked from the reference table). With a 0.1
<!-- RECOVERY NOTE: the line above is only the truncated hunk-header context of line 88; its remainder is unknown. -->

### The pick

Pick by sim-to-real readouts first, then tracking and slip ([criteria](part1_gallop_design.md#design-choices)). `fast_gallop_a` is out: it can't run backward. b and c track equally and saturate torque equally. **c** lands softer (5.9 vs 6.7 body weights), drifts less going forward (1.5 vs 7.8 deg per 10 s) and always keeps two feet down. b has fewer stop falls (9 vs 15 of 96) and less backward drift (-0.6 vs -3.5); c has fewer reversal falls (15 vs 19 of 160). c's weak point is hind splay and body roll going backward (-0.14 rad, 5.1 deg). Part 2 round 1 references were captured from c (commit `ba6a5f9`).

## Superseded design: fast trot

<!-- RECOVERY GAP: everything after line 95 of the b916f17 version is not in any W&B diff. -->
