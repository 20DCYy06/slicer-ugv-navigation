# 通过既有 FKIE 接口接收飞机坐标

入口仍是 `/home/potato/Slicer_Workspace/config/fkie_ugv_sync.launch`。
它同时启动 FKIE `master_sync` 和只写文件的 `/ugv_waypoint_receiver`：

```text
UAV ROS Master (.10.32) 注册 /uav/mission/waypoints 发布者
    -> UGV master_sync 白名单同步发布者登记
    -> UGV /ugv_waypoint_receiver 通过 TCPROS 收到消息
    -> slicer_navigation/config/waypoints.yaml
    -> 人工执行 NAV patrol（接收节点不会代为执行）
```

注意 FKIE 同步的是 ROS 话题注册信息，不是把 launch 文件当作可写的数据端口。
数据由 ROS 发布者传给订阅者。实际发布者在哪台机器运行，决定数据连接走哪条网络路径。

## 输入协议

- 话题：`/uav/mission/waypoints`，已加入 **UGV 接收**白名单。
- 消息：`std_msgs/String`，`data` 字段是以下 JSON。
- 一条消息是一批完整任务，替换而不是追加现有点；旧文件先备份到 config/.backups。
- 坐标和 yaw 原样保存，不做坐标转换；`map` 必须是双方约定的同一地图坐标系。

```json
{"frame_id":"map","waypoints":[{"name":"point_001","x":2.5857,"y":0.0571,"z":0.0,"yaw":0.0,"wait_s":2.0},{"name":"point_002","x":0.5143,"y":0.1286,"z":0.0,"yaw":0.0,"wait_s":2.0}]}
```

## UGV 正常运行

既有 `ugv-sync` 容器会随重启自动运行这个 launch，无需手动启动另一个收点程序。

```bash
cd /home/potato/Slicer_Workspace
./communication/run.sh status
docker logs --tail 30 ugv-sync
cat catkin_ws/src/slicer_navigation/config/waypoints.yaml
```

`[UAV SAVED]` 表示收到并成功保存，日志包含发布者名称、点数、坐标和文件路径。
文件保存不触发导航；已有巡航进程也不会自动重载任务文件。准备好以后人工执行：

```bash
NAV=./catkin_ws/src/slicer_navigation/scripts/run_navigation.sh
# 若导航栈还没运行，先人工启动并完成定位：
"$NAV" nav --background my_room.yaml start_control:=true gui:=true
# 确认现场安全，决定开始移动后才执行：
"$NAV" patrol
```

## 真正在 UAV 上发送一次（队友手工执行）

在 UAV 的 Noetic 环境运行，ROS Master 是 `.10.32`，公布地址也必须是 `.10.32`：

```bash
unset ROS_HOSTNAME
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.10.32:11311
export ROS_IP=192.168.10.32
python3 - <<'PY'
import json, time
import rospy
from std_msgs.msg import String
rospy.init_node('uav_send_waypoints_once', anonymous=True)
pub = rospy.Publisher('/uav/mission/waypoints', String, queue_size=1, latch=True)
data = {'frame_id':'map','waypoints':[
    {'name':'point_001','x':2.5857,'y':0.0571,'z':0.0,'yaw':0.0,'wait_s':2.0},
    {'name':'point_002','x':0.5143,'y':0.1286,'z':0.0,'yaw':0.0,'wait_s':2.0}]}
pub.publish(json.dumps(data))
print('One batch published; waiting for the UGV subscriber...')
until = time.monotonic()+60
while pub.get_num_connections()==0 and not rospy.is_shutdown():
    if time.monotonic()>until: raise RuntimeError('No UGV connection; check FKIE sync')
    time.sleep(.1)
rospy.sleep(2.)
print('Delivered to a subscriber; confirm [UAV SAVED] on UGV.')
PY
```

这里锁存只是让 FKIE 建连稍晚时仍能拿到这一次消息，不循环发送任务。
UAV 不需要订阅这个输入话题，不需要将它加进 UAV 的接收白名单。

## 本次临时模拟与边界

经用户允许，测试发布进程可以运行在 UGV，但连接 UAV 的 `.10.32` ROS Master。
这不同于此前向 UGV 本机 Master 自发自收：测试必须通过 FKIE 把发布者注册同步回
UGV，并从 master_sync/get_sync_info 证明来源为 `.10.32`。

该临时发布者实际 URI 仍为 `.10.33`。FKIE 默认仅同步对端本机节点，因此这种
模拟可能需要临时开启 UGV `master_sync/sync_remote_nodes`；验收完恢复原参数并
重载 ugv-sync。真实 UAV 本机发布者不需要该选项。此测试不等同于飞机进程发送
TCPROS 数据跨电台的物理链路验收；真实测试应使用上面的 UAV 手工命令。

未登录 UAV、未修改其文件/配置/飞控，不同步 `/cmd_vel` 或 `/tf`。
旧回环程序及修订前配置备份：
`/home/potato/Backups/Slicer/2026-10-22/Slicer_fkie_waypoints_fix_20261022_NYf6Gr/`。

本次验收：catkin 编译、隔离 launch/文件测试及 `$NAV check` 通过。临时发布者
注册到 `.10.32` 后，UGV `master_sync/get_sync_info` 确认该话题来源 Master 为
`http://192.168.10.32:11311/`；接收节点输出两条 `[UAV POINT]` 和一条
`[UAV SAVED]`，实际 waypoint 文件的修改时间更新且内容与指定两点一致。
临时发布者已退出，两机上该测试发布登记均清除；UGV 测试参数已恢复为原先未设置
（默认关闭）状态，接收节点正常等待。未启动导航或巡航。原始来源证明和接收日志
保存在上述备份目录的 `fkie_import_provenance.txt`、`receiver_receipt.txt`。
