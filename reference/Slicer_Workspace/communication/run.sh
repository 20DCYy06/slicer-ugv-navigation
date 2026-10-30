#!/usr/bin/env bash
# Communication only. Never launches navigation, a chassis driver or a goal.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
source "$HERE/config/network.env"
mode="${1:-status}"
common=(--init --network host --restart unless-stopped
  -e "ROS_MASTER_URI=http://$UGV_ROS_IP:11311" -e "ROS_IP=$UGV_ROS_IP"
  -e PYTHONNOUSERSITE=1 -e PYTHONUNBUFFERED=1
  -e HOME=/tmp/ugv-communication -e ROS_LOG_DIR=/tmp/ugv-communication-log
  -v "$ROOT:/slicer:ro")

start_one() {
  local name="$1"; shift
  if docker inspect "$name" >/dev/null 2>&1; then
    if [[ "$(docker inspect "$name" --format '{{.State.Running}}')" == true ]]; then
      echo "$name is already running; leaving it unchanged."
    else
      docker start "$name"
    fi
  else
    docker run -d --name "$name" "${common[@]}" "$@"
  fi
}

start_relay() {
  start_one ugv-state-relay "$FKIE_IMAGE" bash -lc \
    'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; export ROS_NAMESPACE=/ugv/communication; exec python3 -B /slicer/communication/relay.py --config /slicer/communication/config/relay.yaml'
}

case "$mode" in
  start-relay) start_relay ;;
  start)
    # Adopt existing containers without recreating the shared local ROS master.
    start_one ugv-roscore "$FKIE_IMAGE" bash -lc \
      'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; exec roscore'
    start_one ugv-discovery -e "ROBOT_HOSTS=['$UAV_ROS_IP']" "$FKIE_IMAGE" bash -lc \
      'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; until rosparam get /rosdistro >/dev/null 2>&1; do sleep 2; done; exec rosrun fkie_master_discovery master_discovery "_robot_hosts:=$ROBOT_HOSTS"'
    start_one ugv-sync --user "$(id -u):$(id -g)" \
      -v "$ROOT/catkin_ws/src/slicer_navigation/config:/slicer/catkin_ws/src/slicer_navigation/config:rw" \
      "$FKIE_IMAGE" bash -lc \
      'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; source /slicer/catkin_ws/devel_navigation/setup.bash; until rosservice list 2>/dev/null | grep -qx /master_discovery/list_masters; do sleep 2; done; exec roslaunch /slicer/config/fkie_ugv_sync.launch'
    start_relay ;;
  reload-sync) docker restart ugv-sync ;;
  status) docker ps -a --filter name=ugv- --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}' ;;
  check)
    docker run --rm --network none --user "$(id -u):$(id -g)" \
      -e HOME=/tmp -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
      -e PYTHONNOUSERSITE=1 -v "$ROOT:/slicer:ro" "$FKIE_IMAGE" bash -lc \
      'source /opt/ros/noetic/setup.bash; exec python3 -B /slicer/communication/verify_relay.py' ;;
  build)
    docker build --build-arg "BASE_IMAGE=$FKIE_BASE_IMAGE" \
      -f "$HERE/Dockerfile.fkie" -t "$FKIE_IMAGE" "$ROOT" ;;
  *) echo 'Usage: communication/run.sh {start|start-relay|reload-sync|status|check|build}'; exit 2 ;;
esac
