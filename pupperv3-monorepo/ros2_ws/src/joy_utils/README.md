# Deliberate software Start

`/estop_controller/start_motion` (`joy_utils/srv/StartMotion`) performs the ordinary
Options release and remembered-controller activation as one transaction. It does
not initialize hardware, home motors, clear a fault, retry, or make ground startup
safe. Start only after the existing native initialization and physical readiness
procedure. Options and the real Stop button keep their existing behavior.
The course gamepad must be connected and publishing valid feedback. This service
uses the existing 250 ms Joy freshness bound at admission and before dispatch.

A caller echoes `instance_id`, `epoch`, `sequence` and `selected_controller` from
one `/estop_controller/stop_status` message. The gateway accepts that challenge
once, for 500 ms measured by its own steady clock, including time waiting for
inventory before dispatch. Gateway-observed Stop, changed selection, reconnect, outstanding work,
fault, or expiry prevents activation. The latch stays unchanged until dispatch;
an undispatched expiry never deactivates an already-running controller. A face
edge while stopped cancels queued Start without authorizing another controller.
Options remains a separate deliberate release and can replace queued Start.
A dispatched operation retains the existing bounded acknowledgement and
late-Stop cleanup; a service success acknowledges controller state, not physical
stability. Do not retry a failed/uncertain Start automatically.

Software callers must send Stop to `/estop_controller/request_stop`
(`std_msgs/msg/Empty`, reliable), as well as requesting immediate native damping
on `/emergency_stop`. The latter alone does **not** latch this gateway and cannot
cancel a queued Start. The separate request topic avoids a loop through the
gateway's own native emergency publications. Each received request advances the
epoch, cancels older queued Starts and retains corrective deactivation if an
already-dispatched activation succeeds late.

Status advertises `software_stop_topic` with that exact request-topic value.
The shared caller requires this capability before software Start; an older
gateway remains Options-only. On Stop, the caller latches locally and waits at
most one second for fresh status from the same gateway with a newer epoch and
`latched=true`. Missing acknowledgement is reported as uncertain. A latch
acknowledgement is not completed backend deactivation, policy-zero feedback or
physical stillness. The gateway continues to own delayed backend cleanup after
the caller closes.

Fresh controller inventory makes an already-active selected controller a no-op
when no competing motion controller is active. Known competing controllers are
deactivated. Inventory is requested after this Start's admission. A missing or
wrong-type selected controller or unknown active owner causes an inert refusal
before any switch; actual failed/ambiguous backend operations still fault. Generic
SwitchController STRICT requests remain unchanged. Do not send a second STRICT
activation after Options or a successful Start.

The maintained resident caller is
[`foundation/hardware/software_start.py`](../../../../foundation/hardware/software_start.py),
used by the [persistent movement client](../../../../final_project/behaviors/ROBOT_RUNBOOK.md#keeping-the-trial-client-ready).
The gateway publishes a new status challenge every 50 ms, including while
stopped. The caller needs a status observed within 200 ms; the gateway still
owns its stricter authoritative challenge checks at dispatch.

After native initialization and current operator readiness, an explicit
`{"command":"start"}` freezes the already-observed challenge at input receipt.
It submits once or refuses, without waiting for reconnect or acquiring a newer
epoch on behalf of an earlier press. A successful reply acknowledges the
selected controller; it does not issue a trial or prove present physical
readiness. The ordinary stock/zero-feedback checks still gate movement.

The first ordinary Stop latch is startable. A later Stop epoch or gateway
instance change ends that client session. An uncertain acknowledgement requests
Stop and exits; inspect the resulting gateway state before deliberately opening
a new client. No automatic retry or second STRICT activation follows Start.
The caller can also deliberately ensure the selected controller is active when
already released; the gateway uses fresh inventory to avoid redundant activation.

Source, generated `StartMotion` interface, gateway binary and caller must be
selected together. Older releases without this interface and Stop capability retain Options and
ordinary movement; they report software Start unavailable. This remains an
offline candidate until staged and physically accepted.

Offline test (no hardware): after configuring/building this package with
`BUILD_TESTING=ON`, run `ctest --test-dir <joy-build> --output-on-failure -R
'^(start_motion_|stop_arbiter_)'`. The DDS tests use a fake manager and the real
gateway, including duplicate STRICT failure, stale challenges, inventory delay,
Stop races and unknown active controllers.


## FC browser standing and Stop

On the FC walk/kick profile, the read-only startup setting
`fc_operator_control:=true` enables `/estop_controller/fc_start_motion`
(`joy_utils/srv/FcStartMotion`) and `/estop_controller/fc_operator_heartbeat`
(`std_msgs/msg/String`, best-effort, volatile, depth 1). The default is false.
The gateway checks for the configured `gemini_walk` and `trick_leg_kick`
controller list and one mapped face button before exposing these interfaces.

A visible, connected browser echoes a challenge from the existing gateway status.
Its Pi relay publishes the echoed `instance_id epoch sequence owner`, where
`owner` is a session-specific 32-character lowercase hexadecimal ID. Status
reports this ID in `fc_operator_owner` after a valid first heartbeat. While that
owner is current, other owners cannot supply heartbeats or stand. Each renewal
must cite an advancing sequence issued in the current epoch within 500 ms.
The 750 ms lease runs from issuance on the gateway's steady clock.

One deliberate browser Stand echoes a recent challenge from before the click
and identifies the same owner in its service request. Admission requires the
latched, idle gateway, released physical Stop, and selected `gemini_walk`.
The gateway marks `fc_operator_armed` as soon as it admits the request and
requires fresh inventory for dispatch. Throughout FC mode a connected gamepad
can issue physical Stop. Options, face buttons, and the ordinary Start service
cannot activate a controller. In the default mode their behavior is unchanged. Browser heartbeat loss
enters the ordinary Stop path on the 50 ms timer, publishes native damping,
cancels queued activation and follows an in-flight activation with corrective
deactivation. A software or physical Stop advances the epoch. Standing again
requires a new challenge and operator action after cleanup. A first heartbeat
alone never requests a switch. No part of this transaction calls Home.

The `fc_operator_watchdog` CTest runs the native executable and both real DDS
interfaces against an in-memory controller manager. It checks disconnection,
delayed/replayed messages, competing owners, cancellation before and after
dispatch, and physical Stop. For an installed robot, verify both controllers'
states and the hardware command claims on the supported robot.
