# Soccer project guide

Start with the [project overview](README.md). Perception work continues in the [perception handoff](PERCEPTION_HANDOFF.md). Motion work continues in the [gait handoff](GAIT_HANDOFF.md). Harness work continues in the [Lab 6 handoff](../foundation/WORKSTREAMS.md).

Reuse the existing class packages. Add soccer-specific code here only when it is needed. When implementing a play, use the harness for dispatch and cancellation and a local controller for fresh observations and velocity updates. Request zero travel when pursuit loses its target. Verify cancellation and stale-track handling in replay. Report field outcomes from observed trials.
