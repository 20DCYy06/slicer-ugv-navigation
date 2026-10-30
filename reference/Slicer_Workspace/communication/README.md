# UGV state-only communication

UGV Jetson: `192.168.10.33`; UAV Jetson: `192.168.10.32`.
Radio addresses `.10.13` / `.10.12` are transport devices, not ROS_IP values.
SSH/Wi-Fi addresses may differ. No global namespace is applied to navigation.

| Internal source | External topic | Message | Export policy |
|---|---|---|---|
| `/odom` | `/ugv/odom` | nav_msgs/Odometry | max 5 Hz, complete pose/twist/covariance |
| `/amcl_pose` | `/ugv/pose` | geometry_msgs/PoseWithCovarianceStamped | max 1 Hz |
| `/slicer_c30d_driver/ready` | `/ugv/chassis_ready` | std_msgs/Bool | on change, latched |
| `/slicer_multi_goal_patrol/state` | `/ugv/mission/state` | std_msgs/String | on change, latched |

`relay.py` forwards serialized messages without changing Header.seq, stamp,
frame_id or values. Only external publication rates are reduced. Four typed publishers are advertised even
when a source is absent; this does NOT fabricate mission/localization data.
There is no control input, TF relay, coordinate conversion or motion limiter.
The rate cap drops excess incoming samples, never queues a backlog or replays
old poses on a timer. No source message means no new exported pose. State
changes are immediate; identical Bool/String payloads are not resent. Late
subscribers receive the latest latched state without periodic broadcasts.
Latched state is the last observed value, not proof that the source is still
online; a separate heartbeat remains deferred as requested.
The internal `/odom`, AMCL, EKF and driver rates are unchanged. There is no
precision reduction, field removal or message-type change. From a 20 Hz odom
source the 5 Hz cap reduces odom payload traffic by about 75%, before TCP/FKIE
overhead. Set `max_hz` in `config/relay.yaml` to 10 if following needs 10 Hz.
No `/ugv/test` is exported; remove it from the UAV receiving whitelist too.
Only the new relay node is namespaced: `/ugv/communication/state_relay`.
All four outward topic prefixes are lowercase `/ugv/`. No uppercase aliases
are retained. UAV reception requires the same lowercase names in its whitelist.

## Files and compatibility

- `config/network.env`: network/image settings for communication only.
- `config/relay.yaml`: the four source/target pairs.
- `config/sync_topics.yaml`: UGV FKIE import whitelist, retaining `/uav/test`.
- `../config/fkie_ugv_sync.launch`: PRESERVED public entrypoint, loads that YAML.
- `launch/fkie_ugv_sync.launch`: optional wrapper for the same entrypoint.
- `Dockerfile.fkie`: archived build recipe; existing root Dockerfile is retained.
- `UAV_HANDOFF.md`: manual UAV adaptation, no SSH access needed from UGV.

FKIE master_sync imports registrations from other masters. Adding exports to
the UGV list alone does NOT make the UAV receive them: its receiving whitelist
must also be updated and its sync process reloaded. No automatic UAV changes.

## Commands on UGV

```bash
cd /home/potato/Slicer_Workspace
./communication/run.sh check       # private master, network=none, no devices
./communication/run.sh start       # adopts existing master/discovery/sync
./communication/run.sh reload-sync # restart ugv-sync only after YAML changes
./communication/run.sh status
docker logs --tail 30 ugv-state-relay
```

No Compose installation is required. `start` never recreates a running master
or starts navigation, a chassis driver or a goal. Existing ugv containers keep
their original settings. The relay uses `slicer-fkie:local`, network host, ROS_IP
`.10.33`, and restart unless-stopped. Root Docker files remain compatible.

The navigation CLI is unchanged except that `check` now uses a private loopback
master inside `--network none`; mapping/nav still use the original `.10.33`.

```bash
NAV=./catkin_ws/src/slicer_navigation/scripts/run_navigation.sh
"$NAV" check
# Only when stationary, Motor OFF and no goals, for read-only integration:
"$NAV" nav --background my_room.yaml start_control:=true gui:=true
docker exec slicer-navigation bash -lc \
 'source /opt/ros/noetic/setup.bash; rostopic info /odom; rosnode info /move_base; rostopic list | grep "^/ugv/"'
docker exec ugv-state-relay bash -lc \
 'source /opt/ros/noetic/setup.bash; rostopic echo -n 1 /ugv/odom'
```

`verify_relay.py` publishes synthetic state ONLY in the isolated Docker check,
compares accepted input/output serialized bytes including headers, verifies
5/1 Hz caps, changed-only states and late-subscriber latching on simulated 20 Hz
streams. It never runs against the real robot master. There is no intentional
waiting for accepted samples, but excess samples are deliberately dropped. Actual
cross-radio latency and reception require UAV-side acceptance tests.

## Time and frame contract

`/ugv/odom` stays in `odom -> base_link` (continuous local odometry), while
`/ugv/pose` retains AMCL's map-frame global pose. A common `frame_id=map` name
alone does not establish a shared map: both teams must agree the map asset,
origin, resolution and axis convention. No automatic frame transform is added.
See `TIME_AND_FRAMES.md` for the read-only UGV snapshot and UAV checklist.

Telemetry-change backup: `/home/potato/Backups/Slicer/2026-10-22/Slicer_telemetry_rate_backup_20261022_leWFDk/`.
To undo just this change, restore that backup's communication files and restart
only `ugv-state-relay`. Navigation and the shared ROS master need no restart.

## UAV waypoints through the FKIE interface (save only)

`../config/fkie_ugv_sync.launch` starts both master_sync and
`/ugv_waypoint_receiver`. The UGV receiving whitelist includes
`/uav/mission/waypoints` (`std_msgs/String` containing a JSON waypoint batch).
The receiver replaces `slicer_navigation/config/waypoints.yaml`, logs each
name/coordinate, and does NOT start patrol or publish control/goal messages.
See `FKIE_WAYPOINTS.md`. The old UGV self-publishing demo tools were removed
from the workspace and backed up outside it.

The sync container sources the existing navigation overlay; the workspace is
read-only except for the navigation config directory, mounted writable for
the waypoint file/lock/backups. It runs as the host user, not root. The shared
ROS master, discovery, navigation algorithms and output telemetry stay unchanged.

## Backup / rollback

Pre-change complete backup (outside the workspace):
`/home/potato/Backups/Slicer/2026-10-22/Slicer_communication_backup_20261022_GvA5W8/`.
It contains verified workspace tar.gz, file checksums and original Docker
inspection results. To roll back communication, stop `ugv-state-relay`, restore
the original `config/fkie_ugv_sync.launch` from that backup and restart ONLY
`ugv-sync`. Restore the NAV launcher from the same backup if reverting the
offline-check fix. Do not restart shared roscore or delete core assets.
