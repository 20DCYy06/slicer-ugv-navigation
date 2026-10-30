# Time and shared-map prerequisite (read-only snapshot, 2026-10-22)

## UGV observed state

- Jetson `.10.33`: systemd-timesyncd active; chrony inactive.
- `NTPSynchronized=yes`; upstream `202.118.1.81` via `1.pool.ntp.org`.
- Reported offset about +7.497 ms and jitter 4.396 ms at inspection time.
  This is an upstream NTP snapshot, NOT a measurement of UAV-to-UGV skew.
- No time service, system timezone or clock value was changed. Clock steps
  during active ROS/LiDAR navigation can invalidate TF timestamps: plan any
  synchronization migration during a stopped maintenance window.
- UGV system timezone is America/New_York; different display timezones alone
  do not imply ROS epoch timestamp skew. Compare UTC/epoch time, not wall time.

UAV maintainers should run these read-only commands on `.10.32` and compare
with `.10.33` results. No SSH access to UAV was used or requested again.

```bash
date -u
timedatectl show -p NTP -p NTPSynchronized
systemctl is-active chrony.service systemd-timesyncd.service
chronyc tracking      # if chrony is installed/running
chronyc sources -v    # identify selected common source and measured offset
timedatectl timesync-status  # if systemd-timesyncd is used
```

Before coordinated following, agree a common stable clock source (or designate
a chrony server on the radio LAN) and measure relative offset from BOTH hosts.
Do not run competing NTP clients; do not blindly run chronyc makestep while
navigation, flight control or LiDAR processing is active. This change does not
install or migrate chrony without a coordinated maintenance plan.

## Frame contract

- `/ugv/odom`: complete `nav_msgs/Odometry`, unchanged local frame `odom`,
  child `base_link`, original time and velocities; maximum 5 Hz.
- `/ugv/pose`: original `geometry_msgs/PoseWithCovarianceStamped` from AMCL,
  normally `map`; maximum 1 Hz. Actual source header is preserved.
- No TF export, no global ROS_NAMESPACE, no manual timestamp rewriting.
- Continuous odom is NOT automatically the common global map frame. Do not
  directly mix its x/y into a UAV map unless the frame relation is established.

UGV current map: `catkin_ws/src/slicer_navigation/maps/my_room.yaml`, resolution
0.05 m/cell, YAML image origin `[-20.0, -20.0, 0.0]`, image `my_room.pgm`.
The YAML image origin is not by itself an observed physical-world alignment.

Confirm identical map assets and a physical origin/yaw convention with UAV:

```bash
cd /home/potato/Slicer_Workspace
sha256sum catkin_ws/src/slicer_navigation/maps/my_room.yaml \
          catkin_ws/src/slicer_navigation/maps/my_room.pgm
```

If UAV uses a different SLAM/ENU/GPS world frame, its maintainer must explicitly
define the map-frame relationship. This exporter performs no guessed transform.
`current_goal` and periodic heartbeat remain deferred until basic transport,
time synchronization and the shared-frame contract are accepted by both teams.
