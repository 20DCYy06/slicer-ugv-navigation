#!/usr/bin/env bash
# Runtime environment only. Does not start master, sensors, drivers or release.
set -euo pipefail
ugv_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ugv_slicer_root="${UGV_SLICER_ROOT:-/home/potato/Slicer_Workspace}"
ugv_image="${UGV_NOETIC_IMAGE:-industrial-cup/nav-slicer:20261023-dwa}"
ugv_master="${ROS_MASTER_URI:-http://192.168.10.33:11311}"
ugv_ip="${ROS_IP:-192.168.10.33}"
ugv_serial="${UGV_SERIAL_DEVICE:-}"
ugv_device_args=()
if [[ -n "$ugv_serial" ]]; then
  if [[ "$ugv_serial" != /dev/* || "$ugv_serial" == */../* || ! -c "$ugv_serial" ||
        "$(readlink -f -- "$ugv_serial")" != /dev/* ]]; then
    printf 'UGV_SERIAL_DEVICE must name an existing character device under /dev: %s\n' "$ugv_serial" >&2
    exit 2
  fi
  # Keep the real tty name inside Docker so the driver's auto scan finds it.
  ugv_serial="$(readlink -f -- "$ugv_serial")"
  ugv_device_args=(--device "$ugv_serial" --group-add "$(stat -Lc %g -- "$ugv_serial")")
fi
ugv_gnss="${UGV_GNSS_DEVICE:-}"
if [[ -n "$ugv_gnss" ]]; then
  if [[ "$ugv_gnss" != /dev/* || "$ugv_gnss" == */../* || ! -c "$ugv_gnss" ||
        "$(readlink -f -- "$ugv_gnss")" != /dev/* ]]; then
    printf 'UGV_GNSS_DEVICE must name an existing character device under /dev: %s\n' "$ugv_gnss" >&2
    exit 2
  fi
  ugv_gnss="$(readlink -f -- "$ugv_gnss")"
  if [[ "$ugv_gnss" == "$ugv_serial" ]]; then
    printf 'The same device cannot be mapped as both chassis and GNSS.\n' >&2
    exit 2
  fi
  ugv_device_args+=(--device "$ugv_gnss:/dev/gnss" --group-add "$(stat -Lc %g -- "$ugv_gnss")")
fi
ugv_secret_args=()
if [[ -n "${UGV_NTRIP_CREDENTIALS:-}" ]]; then
  if [[ ! -f "$UGV_NTRIP_CREDENTIALS" || ! -r "$UGV_NTRIP_CREDENTIALS" ]]; then
    printf 'UGV_NTRIP_CREDENTIALS must name a readable file.\n' >&2
    exit 2
  fi
  ugv_secret="$(readlink -f -- "$UGV_NTRIP_CREDENTIALS")"
  ugv_secret_args=(--mount "type=bind,src=${ugv_secret},dst=/run/secrets/slicer_ntrip.json,readonly")
fi
ugv_tty=()
if [[ -t 0 && -t 1 ]]; then ugv_tty=(-it); fi
if [[ $# -eq 0 ]]; then set -- bash; fi
exec docker run --rm "${ugv_tty[@]}" --network host --cap-drop ALL \
  --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
  "${ugv_device_args[@]}" \
  "${ugv_secret_args[@]}" \
  --mount "type=bind,src=${ugv_root},dst=/workspace" \
  --mount "type=bind,src=${ugv_slicer_root},dst=/slicer,readonly" --workdir /workspace \
  -e "ROS_MASTER_URI=${ugv_master}" -e "ROS_IP=${ugv_ip}" -e ROS_HOSTNAME= \
  -e ROS_HOME=/workspace/.ros -e PYTHONDONTWRITEBYTECODE=1 \
  --entrypoint /bin/bash "$ugv_image" -c '
    set -e
    ugv_command=("$@"); set --
    unset ROS_HOSTNAME
    source /opt/ros/noetic/setup.bash
    if [[ -f /slicer/catkin_ws/devel_navigation/setup.bash ]]; then
      source /slicer/catkin_ws/devel_navigation/setup.bash
    fi
    source /workspace/install/setup.bash --extend
    export ROS_PACKAGE_PATH="/slicer/catkin_ws/src/Slicer_wasd_control:${ROS_PACKAGE_PATH}"
    exec "${ugv_command[@]}"
  ' bash "$@"
