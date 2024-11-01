#!/usr/bin/env bash
source /ws/install/setup.bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
python3 ${SCRIPT_DIR}/client.py
#ros2 action send_goal simple action_simple/action/ActionSimple "{input: 5}"
