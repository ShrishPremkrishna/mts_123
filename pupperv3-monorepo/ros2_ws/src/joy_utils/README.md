# Deliberate software Start

`/estop_controller/start_motion` (`joy_utils/srv/StartMotion`) performs the ordinary
Options release and remembered-controller activation as one transaction. It does
not initialize hardware, home motors, clear a fault, retry, or make ground startup
safe. Start only after the existing native initialization and physical readiness
procedure. Options and the real Stop button keep their existing behavior.
The course gamepad must be connected and publishing valid feedback. Start uses
the existing 250 ms Joy freshness bound at admission and before dispatch; this
is not joystickless support or a new continuous stopping watchdog.

A caller echoes `instance_id`, `epoch`, `sequence` and `selected_controller` from
one `/estop_controller/stop_status` message. The gateway accepts that challenge
once, for 500 ms measured by its own steady clock, including time waiting for
inventory before dispatch. Stop, changed selection, reconnect, outstanding work,
fault, or expiry prevents activation. The latch stays unchanged until dispatch;
an undispatched expiry never deactivates an already-running controller. A face
edge while stopped cancels queued Start without authorizing another controller.
Options remains a separate deliberate release and can replace queued Start.
A dispatched operation retains the existing bounded acknowledgement and
late-Stop cleanup; a service success acknowledges controller state, not physical
stability. Do not retry a failed/uncertain Start automatically.

Fresh controller inventory makes an already-active selected controller a no-op
when no competing motion controller is active. Known competing controllers are
deactivated. Inventory is requested after this Start's admission. A missing or
wrong-type selected controller or unknown active owner causes an inert refusal
before any switch; actual failed/ambiguous backend operations still fault. Generic
SwitchController STRICT requests remain unchanged. Do not send a second STRICT
activation after Options or a successful Start.

Use one resident ROS caller, not separate `ros2 topic echo` and `ros2 service call`
processes. After sourcing the workspace that contains this service, this explicit
one-shot invocation observes status and sends at most one Start. It never homes or
restarts a service, and exits on refusal or uncertainty:

The example's authorization snapshot is captured after discovery, not at shell
launch. Stop events before that snapshot are outside its fence. A resident Start
button must complete discovery first, freeze its already-observed token at the
button press, and submit once or refuse; it must not wait for reconnect and sample
a newer epoch on behalf of an earlier press.

```bash
python3 - <<'PY'
import json, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from joy_utils.srv import StartMotion

rclpy.init()
node = rclpy.create_node('deliberate_start_once')
client = node.create_client(StartMotion, '/estop_controller/start_motion')
attempted, future, error = False, None, None

def status(message):
    global attempted, future, error
    if attempted:
        return
    attempted = True
    try:
        state = json.loads(message.data)
        blocked = ('fault', 'stop_required', 'in_flight',
                   'unknown_remote_submission', 'shutting_down')
        if state['protocol'] != 'pupper_stop_v1' or any(state[k] for k in blocked):
            raise ValueError('gateway is not ready for a deliberate Start')
        request = StartMotion.Request()
        for key in ('instance_id', 'epoch', 'sequence', 'selected_controller'):
            setattr(request, key, state[key])
        future = client.call_async(request)
    except Exception as exc:
        error = str(exc)

try:
    if not client.wait_for_service(timeout_sec=2.0):
        raise RuntimeError('Start service unavailable; no action submitted')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(String, '/estop_controller/stop_status', status, qos)
    deadline = time.monotonic() + 3.0
    while not error and (future is None or not future.done()) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if error:
        raise RuntimeError(error)
    if future is None or not future.done():
        raise RuntimeError('No confirmed result; inspect Stop status, do not retry automatically')
    result = future.result()
    print(result.message)
    raise SystemExit(0 if result.success else 1)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

Offline test (no hardware): after configuring/building this package with
`BUILD_TESTING=ON`, run `ctest --test-dir <joy-build> --output-on-failure -R
'^(start_motion_|stop_arbiter_)'`. The DDS tests use a fake manager and the real
gateway, including duplicate STRICT failure, stale challenges, inventory delay,
Stop races and unknown active controllers.
