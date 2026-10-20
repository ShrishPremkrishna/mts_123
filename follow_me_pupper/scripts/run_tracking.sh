#!/bin/bash
# One-command demo: robot stack + state machine + interactive tracking prompt.
#
# For lab work you usually want three terminals instead, so you can restart
# your state machine without restarting the robot:
#
#   1) ros2 launch robot/follow_me.launch.py
#   2) python3 lab/follow_me.py
#   3) python3 robot/test_tracking.py

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
cd "$PROJECT_ROOT"

cleanup() {
    echo ""
    echo "Shutting down..."
    [ -n "$STATE_PID" ] && kill "$STATE_PID" 2>/dev/null
    [ -n "$ROS_PID" ] && kill "$ROS_PID" 2>/dev/null
    sleep 1
    bash "$SCRIPT_DIR/cleanup.sh"
    exit 0
}
trap cleanup INT TERM

echo "Cleaning up any previous run..."
bash "$SCRIPT_DIR/cleanup.sh" > /dev/null 2>&1
sleep 1

# Control + camera + viser viewer. detect:=true (the default) makes viser_camera.py
# run YOLOv8n-seg on the Hailo and publish /detections for follow_me.py.
echo "1/3  Robot stack + camera + viser viewer..."
ros2 launch "$PROJECT_ROOT/robot/follow_me.launch.py" > /tmp/follow_me_stack.log 2>&1 &
ROS_PID=$!
sleep 12   # camera + Hailo model load

echo "2/3  Tracking state machine..."
python3 "$PROJECT_ROOT/lab/follow_me.py" > /tmp/follow_me_state.log 2>&1 &
STATE_PID=$!
sleep 2

echo "3/3  Interactive prompt."
echo ""
echo "  Viewer: http://$(hostname -I | awk '{print $1}'):8080"
echo "  Logs:   /tmp/follow_me_stack.log  /tmp/follow_me_state.log"
echo ""

python3 "$PROJECT_ROOT/robot/test_tracking.py"
cleanup
