#!/usr/bin/env bash
# Environment shell only: no sessions, registration, robot processes or devices.
set -euo pipefail
ugv_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ugv_image="${UGV_NOETIC_IMAGE:-industrial-cup/nav-slicer:20261023-dwa}"
# Optional source-only reference for launch/include resolution. Never source
# the collected ARM devel/install setup files in an x86 test container.
ugv_slicer_root="${UGV_SLICER_ROOT:-$(dirname -- "$ugv_root")/Slicer_Workspace}"
ugv_reference_args=()
if [[ -d "$ugv_slicer_root/catkin_ws/src" ]]; then
  ugv_reference_args=(--mount "type=bind,src=${ugv_slicer_root},dst=/slicer,readonly")
fi
ugv_tty=()
if [[ -t 0 && -t 1 ]]; then ugv_tty=(-it); fi
if [[ $# -eq 0 ]]; then set -- bash; fi
exec docker run --rm "${ugv_tty[@]}" --network none --cap-drop ALL \
  --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
  "${ugv_reference_args[@]}" \
  --mount "type=bind,src=${ugv_root},dst=/workspace" --workdir /workspace \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
  -e ROS_HOME=/workspace/.ros -e PYTHONDONTWRITEBYTECODE=1 \
  --entrypoint /bin/bash "$ugv_image" -c \
  'set -e; ugv_command=("$@"); set --; source /opt/ros/noetic/setup.bash; if [[ -f /workspace/devel/setup.bash ]]; then source /workspace/devel/setup.bash; fi; if [[ -d /slicer/catkin_ws/src ]]; then export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/slicer/catkin_ws/src"; fi; exec "${ugv_command[@]}"' bash "$@"
