#!/usr/bin/env bash
# WSL observation only: Docker/WSLg environment plus one RViz preset.
set -euo pipefail

case "${1:-perception}" in
  perception) ugv_preset=small_obstacle.rviz ;;
  navigation) ugv_preset=odom_navigation.rviz ;;
  -h|--help)
    printf 'Usage: %s [perception|navigation]\n' "$0"
    printf '  perception (default): point clouds, scan, costmaps and footprint\n'
    printf '  navigation: goals, global/local plans, odom and costmaps\n'
    printf 'Uses ROS_MASTER_URI / ROS_IP when set; otherwise selects ROS_IP from the route to the master.\n'
    exit 0 ;;
  *) printf 'Choose perception or navigation. See --help.\n' >&2; exit 2 ;;
esac
if [[ $# -gt 1 ]]; then
  printf 'Use one preset argument only. See --help.\n' >&2
  exit 2
fi

ugv_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ugv_image="${UGV_NOETIC_IMAGE:-industrial-cup/nav-slicer:20261023-dwa}"
ugv_master="${ROS_MASTER_URI:-http://192.168.10.33:11311}"
ugv_ip="${ROS_IP:-}"
if [[ -z "$ugv_ip" ]]; then
  ugv_master_host="${ugv_master#*://}"
  ugv_master_host="${ugv_master_host%%:*}"
  ugv_master_host="${ugv_master_host%%/*}"
  ugv_ip="$(ip -4 route get "$ugv_master_host" | awk '{for(i=1;i<NF;i++) if($i=="src") {print $(i+1); exit}}')"
  if [[ -z "$ugv_ip" ]]; then
    printf 'Cannot select observer IP. Set ROS_IP to the WSL address reachable from the car.\n' >&2
    exit 2
  fi
fi

printf 'RViz: %s | master=%s | observer=%s\n' "$ugv_preset" "$ugv_master" "$ugv_ip"
exec docker run --rm --network host \
  --cap-drop ALL --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" \
  -e DISPLAY -e QT_QPA_PLATFORM=xcb -e QT_XCB_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e ROS_HOME=/tmp/ugv-rviz-ros \
  -e "ROS_MASTER_URI=$ugv_master" -e "ROS_IP=$ugv_ip" -e ROS_HOSTNAME= \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
  --mount "type=bind,src=$ugv_root/src/ugv_bringup/rviz,dst=/rviz,readonly" \
  --entrypoint /bin/bash "$ugv_image" \
  -c 'set -e; unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; exec rviz -d "$1" __name:=ugv_rviz_observe' \
  bash "/rviz/$ugv_preset"
