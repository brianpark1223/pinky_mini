#!/usr/bin/env bash
set -uo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

echo "ROS_DISTRO=${ROS_DISTRO:-missing} DOMAIN=$ROS_DOMAIN_ID DISCOVERY=$ROS_AUTOMATIC_DISCOVERY_RANGE"
echo "--- nodes"
ros2 node list
echo "--- required topic endpoints"
for topic in /map /pinky_1/amcl_pose /pinky_2/amcl_pose /pinky_1/battery_state /pinky_2/battery_state; do
  counts="$(ros2 topic info "$topic" 2>/dev/null | tr '\n' ' ')"
  echo "$topic  ${counts:-missing}"
done
echo "--- required action servers"
for action in /pinky_1/navigate_to_pose /pinky_2/navigate_to_pose \
              /pinky_1/compute_path_to_pose /pinky_2/compute_path_to_pose; do
  servers="$(ros2 action info "$action" 2>/dev/null | sed -n 's/^Action servers: //p')"
  echo "$action servers=${servers:-0}"
done
echo "--- TF"
for frame in pinky_1/base_footprint pinky_2/base_footprint; do
  if timeout 2s ros2 run tf2_ros tf2_echo map "$frame" 2>&1 | grep -q '^At time'; then
    echo "map -> $frame OK"
  else
    echo "map -> $frame MISSING"
  fi
done
PINKY1_HOST="${PINKY1_HOST:-192.168.14.91}"
PINKY2_HOST="${PINKY2_HOST:-192.168.14.90}"
echo "--- Pinky robot network reachability (user-configured addresses)"
for entry in "Pinky 1:$PINKY1_HOST" "Pinky 2:$PINKY2_HOST"; do
  name="${entry%%:*}"
  host="${entry#*:}"
  if ping -c 1 -W 1 "$host" >/dev/null 2>&1; then
    echo "$name ($host) reachable by ping; robot identity/ROS health not verified"
  else
    echo "$name ($host) unreachable by ping"
  fi
done
