# UAV manual handoff (communication only)

No UAV files/processes were modified by the UGV deployment. Run these steps on
the UAV Jetson `192.168.10.32`, not on a radio device. Do not change navigation,
flight-control namespaces, topic message contents, or TF.

## Existing deployment: preferred edit

The earlier UAV read-only snapshot had `[/ugv/test, /odom]`. The current
handoff below follows the user's revised lowercase list, without unprefixed
`/odom`. The host/container path of its launch file and its Docker
container name cannot be determined without UAV login. Do NOT assume it is the
UGV launch path or guess the container name.

In the existing UAV master_sync launch/YAML, replace its sync_topics value with
the following (all exported UGV state names are lowercase):

```yaml
sync_topics:
  - /ugv/odom
  - /ugv/pose
  - /ugv/chassis_ready
  - /ugv/mission/state
```

For inline roslaunch configuration inside the existing master_sync `<node>`:

```xml
<rosparam param="sync_topics">
  - /ugv/odom
  - /ugv/pose
  - /ugv/chassis_ready
  - /ugv/mission/state
</rosparam>
```

Do not leave two sync_topics definitions. Keep all other FKIE parameters.
The new interface uses `/ugv/odom`, not an unprefixed `/odom` import. This
document changes no UAV files automatically; the UAV maintainer applies and
accepts this updated list manually.
The full message types remain compatible: odom is capped at 5 Hz, pose at 1 Hz;
ready/mission state publish on change and are latched. `/ugv/test` is no longer
in the receiving list. No new target/heartbeat interface is added yet.

Find the existing launch mount and container on UAV:

```bash
docker ps --no-trunc --format 'table {{.Names}}\t{{.Image}}\t{{.Command}}'
# Set this to the actual dedicated FKIE sync container shown above:
UAV_SYNC_CONTAINER='REPLACE_WITH_ACTUAL_UAV_SYNC_CONTAINER'
docker inspect "$UAV_SYNC_CONTAINER" --format '{{json .Mounts}}'
docker inspect "$UAV_SYNC_CONTAINER" --format '{{json .Config.Cmd}}'
```

Edit that mounted configuration file, backing it up first. If the container
also runs flight control or the ROS master, do NOT restart the container: have
its maintainer reload only the FKIE sync launch using that deployment's manager.
For a dedicated sync-only container the complete reload command is:

```bash
docker restart "$UAV_SYNC_CONTAINER"
docker logs --tail 50 "$UAV_SYNC_CONTAINER"
docker exec "$UAV_SYNC_CONTAINER" bash -lc \
 'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://192.168.10.32:11311 ROS_IP=192.168.10.32; rosparam get /master_sync/sync_topics'
```

`rosparam set` alone does NOT reload existing FKIE SyncThread filters. Do not
restart the UAV ROS master, discovery, navigation or flight controller.

## Optional canonical file for an existing external launch

If reorganizing the UAV communication files manually, save the YAML above to
`$HOME/communication/config/fkie_uav_sync_topics.yaml`, and inside its existing
sync node load it with an absolute path:

```xml
<rosparam command="load" file="/home/ACTUAL_USER/communication/config/fkie_uav_sync_topics.yaml"/>
```

This file must be visible at that path inside the container (use its existing
bind mount). Merely creating a host YAML that the launch never loads has no
effect. Existing `sync_nodes`/`ignore_nodes`/`ignore_topics` overrides, if any,
must not exclude `/ugv/communication/state_relay` or the four new topics.

## Acceptance on UAV

Run in the UAV Noetic communication container after reloading:

```bash
docker exec -it "$UAV_SYNC_CONTAINER" bash
unset ROS_HOSTNAME
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.10.32:11311 ROS_IP=192.168.10.32
rosparam get /master_sync/sync_topics
rostopic list | grep '^/ugv/'
rostopic info /ugv/odom
rostopic echo -n 1 /ugv/odom
rostopic echo -n 1 /ugv/pose
rostopic echo -n 1 /ugv/chassis_ready
# Only while a real patrol node exists / has published state:
rostopic echo -n 1 /ugv/mission/state
rosservice call /master_sync/get_sync_info
```

The remote publisher URI must contain `192.168.10.33`, not `.10.13`. If the UAV
can list a topic but receives no messages, check bidirectional TCPROS reachability
and advertised addresses. Topic presence alone is not a successful data test.
AMCL may not publish pose before localization. Mission state is not fabricated
to make a test pass. No `/cmd_vel` or `/tf` is newly exported by this interface.
