#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd -- "$REPO/../.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Limit discovery to this repository. The current workspace also contains old
# top-level copies with identical package names; --base-paths avoids deleting or
# modifying those user files while preventing colcon's duplicate-name error.
cd "$WORKSPACE"
colcon build --base-paths src/pinky_mini \
  --packages-select pinky_mission pinky_mission_panel --symlink-install
