#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MAP="$ROOT/event_control/maps/sesac_gangnam_2.yaml"
POINTS=""
HOME_SLOTS=""
TRAFFIC=""
REAL=false
GUI=true
RVIZ=true

while (($#)); do
  case "$1" in
    --real) REAL=true ;;
    --no-gui) GUI=false ;;
    --no-rviz) RVIZ=false ;;
    --map) MAP="$2"; shift ;;
    --points) POINTS="$2"; shift ;;
    --home-slots) HOME_SLOTS="$2"; shift ;;
    --traffic) TRAFFIC="$2"; shift ;;
    *) echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
  esac
  shift
done

if $REAL && [[ -z "$POINTS" || -z "$HOME_SLOTS" || -z "$TRAFFIC" ]]; then
  echo "실차 모드에는 --points, --home-slots, --traffic 실측 설정 파일이 필요합니다." >&2
  exit 2
fi
for path in "$MAP" ${POINTS:+"$POINTS"} ${HOME_SLOTS:+"$HOME_SLOTS"} ${TRAFFIC:+"$TRAFFIC"}; do
  [[ -f "$path" ]] || { echo "파일을 찾을 수 없습니다: $path" >&2; exit 2; }
done

# A previously sourced/broken workspace can make the Jazzy setup try to load a
# non-existent local setup.sh. Start the main server from a clean ROS overlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

if ros2 node list 2>/dev/null | grep -Eq '^/(map_server|lifecycle_manager|fms_controller|delivery_gui)$'; then
  echo "중앙 서버 노드가 이미 실행 중입니다. 중복 발행과 배정을 막기 위해 기존 프로세스를 먼저 종료하세요." >&2
  ros2 node list | grep -E '^/(map_server|lifecycle_manager|fms_controller|delivery_gui)$' >&2 || true
  exit 3
fi

LOG_DIR="${XDG_RUNTIME_DIR:-/tmp}/pinky_main_server_${UID}"
mkdir -p "$LOG_DIR"
pids=()
stop_all() {
  trap - INT TERM EXIT
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for _ in {1..50}; do
    alive=false
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
    $alive || break
    sleep 0.1
  done
  for pid in "${pids[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap stop_all INT TERM EXIT

start() {
  local name="$1"; shift
  echo "$name 시작, 로그: $LOG_DIR/$name.log"
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  pids+=("$!")
}

start map_server ros2 run nav2_map_server map_server --ros-args -p "yaml_filename:=$MAP"
start lifecycle ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p 'node_names:=[map_server]'

ready=false
for _ in {1..50}; do
  if ros2 lifecycle get /map_server 2>/dev/null | grep -q active; then ready=true; break; fi
  sleep 0.2
done
$ready || { echo "지도 서버가 활성 상태가 아닙니다. $LOG_DIR/map_server.log를 확인하세요." >&2; exit 4; }

fms=(python3 "$ROOT/event_control/fms_controller.py" --ros-args -p docking:=false)
if $REAL; then
  fms+=( -p dry_run:=false -p "points_file:=$POINTS" -p "home_slots_file:=$HOME_SLOTS"
         -p "traffic_file:=$TRAFFIC" -p traffic_enabled:=true )
else
  fms+=( -p dry_run:=true -p traffic_enabled:=false )
fi
start fms "${fms[@]}"

if $GUI; then
  start gui python3 "$ROOT/event_control/delivery_gui.py" --ros-args \
    -p robots:=pinky_1,pinky_2 -p "map_yaml:=$MAP" -p map_topic:=/map
fi
if $RVIZ; then
  start rviz rviz2 -d "$ROOT/pinky_mission/rviz/pinky_mission.rviz"
fi

echo "Pinky 중앙 서버를 시작했습니다. 모드: $($REAL && echo '실차(FMS 비활성)' || echo '모의 실행')"
echo "상태 확인: ros2 topic echo /delivery/status"
echo "모든 모듈 중지: Ctrl+C"
wait -n "${pids[@]}"
echo "모듈 하나가 종료되어 중앙 서버의 나머지 프로세스도 함께 종료합니다." >&2
exit 5
