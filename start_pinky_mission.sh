#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  WORKSPACE_SETUP="$SCRIPT_DIR/install/setup.bash"
elif [[ -f "$SCRIPT_DIR/../../install/setup.bash" ]]; then
  WORKSPACE_SETUP="$SCRIPT_DIR/../../install/setup.bash"
else
  echo "找不到工作空间 install/setup.bash，请先执行 colcon build。" >&2
  exit 1
fi
source "$WORKSPACE_SETUP"
set -u
MISSION_LOCK="${XDG_RUNTIME_DIR:-/tmp}/pinky_mission_${UID}.lock"
exec 9>"$MISSION_LOCK"
if ! flock -n 9; then
  echo "Pinky Mission 已经在运行。请关闭现有 RViz/启动终端后再试。" >&2
  exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec ros2 launch pinky_mission mission.launch.py "$@"
