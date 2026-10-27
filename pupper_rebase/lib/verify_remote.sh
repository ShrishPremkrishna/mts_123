#!/bin/bash
# Runs ON the Pupper after rebase.sh; prints one PASS/FAIL line per check.
MONO=/home/pi/pupperv3-monorepo
TARGET=0e50afbdc11ae066606b0290c23969f46a265626
ok=0; bad=0
check() { if eval "$2" >/dev/null 2>&1; then echo "PASS  $1"; ok=$((ok+1)); else echo "FAIL  $1"; bad=$((bad+1)); fi; }
check "~/.config owned by pi"               "[ \$(stat -c %U /home/pi/.config) = pi ]"
check "monorepo at cs123-build"            "[ \$(git -C $MONO rev-parse HEAD) = $TARGET ]"
check "13 trick recordings installed"      "[ \$(ls $MONO/ros2_ws/install/animation_controller_py/share/animation_controller_py/launch/animations/*.csv | wc -l) -eq 13 ]"
check "detector imports pyzbar"            "python3 -c 'import pyzbar.pyzbar'"
check "Gemini lab server deps"             "python3 -c 'import fastapi, uvicorn, multipart'"
check "viser + yourdfpy (FK lab viewer)"   "python3 -c 'import viser, yourdfpy'"
check "pupper-gui running"                 "systemctl is-active pupper-gui"
check "comitup enabled"                    "systemctl is-enabled comitup"
check "tailscale installed"                "command -v tailscale"
check "tailscale not logged in"            "! tailscale status >/dev/null 2>&1"
check "claude code installed"              "/home/pi/.local/bin/claude --version"
check "old voice agent not enabled"        "! systemctl is-enabled llm-agent"
check "no API keys in home"                "! grep -rIqE --exclude-dir=.git --exclude-dir=node_modules --exclude-dir=build --exclude-dir=install --exclude-dir=log --exclude-dir=target 'AIza[0-9A-Za-z_-]{35}|sk-[A-Za-z0-9_-]{20,}' /home/pi"
echo "$ok passed, $bad failed"
[ $bad = 0 ]
