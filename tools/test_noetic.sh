#!/usr/bin/env bash
set -euo pipefail
ugv_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS-only output directory; every run gets a distinct file, including failures.
mkdir -p "$ugv_root/.test-results"
ugv_run="$(mktemp -d "$ugv_root/.test-results/run-XXXXXXXX")"
printf 'TEST_ONLY: no network, no devices. Evidence: %s\n' "$ugv_run"
set +e
"$ugv_root/tools/noetic.sh" bash -c '
  set -eo pipefail
  catkin_make -j2
  source devel/setup.bash
  # Sourcing Catkin replaces ROS_PACKAGE_PATH; restore the optional source-only
  # reference added by noetic.sh without sourcing its collected ARM setup files.
  if [[ -d /slicer/catkin_ws/src ]]; then
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/slicer/catkin_ws/src"
  fi
  python3 -m unittest discover -s tools -p "test_*.py" -v
  catkin_make run_tests -j2
  catkin_test_results --all build/test_results
  catkin_make install -j2
  # Start with no inherited devel/source overlay for the installed checks.
  env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    HOME="${HOME:-/tmp}" ROS_HOME=/workspace/.ros \
    ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 \
    PYTHONDONTWRITEBYTECODE=1 bash --noprofile --norc -c "
      set -e
      source /opt/ros/noetic/setup.bash
      source /workspace/install/setup.bash
      if [[ -d /slicer/catkin_ws/src ]]; then
        export ROS_PACKAGE_PATH=\"\${ROS_PACKAGE_PATH}:/slicer/catkin_ws/src\"
      fi
      cd /tmp
      exec python3 /workspace/tools/check_noetic_install.py --prefix /workspace/install
    "
' 2>&1 | tee "$ugv_run/output.log"
ugv_status=${PIPESTATUS[0]}
set -e
if [[ -d "$ugv_root/build/test_results" ]]; then
  cp -r "$ugv_root/build/test_results" "$ugv_run/junit"
fi
printf '%s\n' "$ugv_status" > "$ugv_run/exit-code"
exit "$ugv_status"
