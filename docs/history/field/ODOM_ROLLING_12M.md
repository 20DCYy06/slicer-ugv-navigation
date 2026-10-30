# 12×12m滚动全局Costmap：现场短单

已于2026-10-25部署到Jetson独立工程 `/home/potato/slicer_ugv_ros1`，完成ARM64构建、安装及参数解析核对；未启动ROS节点或接触串口。以下命令在已加载本工程的ROS shell中执行，统一使用 `/ugv`。定位和唯一driver沿用现场现有流程；导航launch不启动driver、不release、不发目标。

后续转向接口切换见 [0x16车体IMU操作单](BODY_SPEED_0X16.md)：仅底盘终端显式选择body_speed，下面导航终端的Guard/Mux配置仍可沿用。切换模式不等于实际转向已验收。

## 1. 启动与观察

定位输入 `/odom`、`/scan`、TF已正常后，只启动一个导航实例：

```bash
roslaunch ugv_bringup slicer_odom.launch local_planner:=dwa connect_control:=true control_config:=$(rospack find ugv_bringup)/config/slicer_driver_wheel_speed_reference.yaml global_costmap_config:=$(rospack find ugv_bringup)/config/rolling_global_12m.yaml
```

`control_config`显式指向我方配置，避免队友的 `body_speed` 默认配置被我方Mux读取。只看规划、不接控制时改为 `connect_control:=false`，raw输出隔离为 `/ugv/cmd_vel/nav_raw`，不执行release；实际接线仍为raw→Guard→Mux→唯一driver→Watchdog。

另开有图形显示的ROS shell（桌面或已配置的WSLg）显示RViz；无显示转发的车端容器终端不能直接打开窗口：

Windows Docker Desktop/WSLg实时观察需要单独核对网络和显示转发，按[WSL RViz网络说明](RVIZ_WSL_NETWORK.md)执行；仅设置ROS_MASTER_URI或使用现有slicer_shell并不足够。

```bash
rosrun rviz rviz -d $(rospack find ugv_bringup)/rviz/odom_navigation.rviz
```

Fixed Frame为`odom`；显示scan、融合odom、全局/局部代价地图、GlobalPlanner路径、DWA局部轨迹和已接受目标。该配置没有发目标工具，目标统一用下面的Action客户端。

## 2. 发目标与查看规划

以下是**固定odom坐标**，单位米；`yaw`为终点朝向，单位弧度，0沿odom的+X，正值逆时针。坐标示例假定机器人在odom原点附近；12m窗口随车移动，目标须位于当时窗口内（约车周围各轴±6m，边缘还需余量）。扩大窗口不会扩大雷达视距。

先求路径，不发Action、不动车：

```bash
rosrun ugv_navigation goal.py plan --frame odom --x 1 --y 1 --yaw 0.7854
rosrun ugv_navigation goal.py plan --frame odom --x 3 --y 4 --yaw 0.9273
```

这两个yaw示例是从原点朝向相应坐标的角度，不是自动跟踪车头；可按任务修改。若要表示“相对当前车头前方/左方的偏移”，已有客户端支持`--relative`，此时x/y/yaw一起按当前base姿态变换一次后固定到odom，不能与绝对坐标混用。

现场接通控制、求得路径且路径周围适合测试后，**一次只发一个目标**：

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation goal.py send --frame odom --x 1 --y 1 --yaw 0.7854 --timeout 120
```

第一目标结束并停稳后，再执行：

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation goal.py send --frame odom --x 3 --y 4 --yaw 0.9273 --timeout 120
```

只观察模式省去release即可调用send检查软件输出，实际车辆不会由隔离raw驱动。

查看状态与规划更新：

```bash
rostopic echo /ugv/move_base/status
rostopic echo -n 1 /ugv/move_base/GlobalPlanner/plan/header
```

正常：目标保持同一goal_id，地图随scan更新，GlobalPlanner在活动目标下约1Hz更新，DWA根据局部地图生成轨迹。空路径先检查目标是否在窗口内、观测/障碍/未知区域和输入有效性；ABORTED保留结果，不自动重发。原0.125m/s、0.25rad/s与15cm到点容差均未改，软件检查不替代实车转向或停车验收。

## 3. 在线障碍的判据

这份profile仅把全局窗口改为12m，并让GlobalPlanner允许规划经过unknown；unknown仍保留未知标记，局部DWA和Guard检查照旧。全局路线经过未观测区不代表当地已经可安全行驶。

两层地图继续使用同一scan源：`marking=true`、`clearing=true`、3m障碍标记、最多4m射线清除、0.05m分辨率；实际覆盖还受scan range_max/视场影响。`observation_persistence=0`不表示旧障碍自动过期：移走后要有真实新射线经过才会清除，遮挡后的格子不会被穿透清空。全局/局部更新4/5Hz，活动目标重规划1Hz；没有自动恢复或失败重发。

现场观察障碍放入/移走后的栅格标记、射线清除及路径改变。未标记、移走后仍阻塞、路径/局部轨迹无法通过或输入停更时先hold并保留记录；不靠清图或放宽Guard掩盖问题。

## 4. 停止与采集

Ctrl-C结束正在等待的send会请求取消该目标；另一个ROS shell可用标准Action取消，再保持控制：

```bash
rosrun ugv_navigation goal.py cancel
rosrun ugv_slicer_control control.py hold
```

以现场车体停稳为准，异常时用已有物理急停/Motor OFF；停止roslaunch本身不等于停车确认。

**发第一个目标前**，在采集终端执行下面一整行，等 `RECORDING STARTED` 再发目标。600秒是上限，整组完成并保留停车尾段后Ctrl-C结束采集，不必等满；避免准备时间用完短录制窗口。

```bash
python3 tools/collect_odom_test.py --duration 600 --output evidence/odom12-$(date +%Y%m%d-%H%M%S) --extra-topic /ugv/move_base/current_goal --extra-topic /ugv/move_base/cancel --extra-topic /ugv/move_base/GlobalPlanner/plan --extra-topic /ugv/move_base/DWAPlannerROS/local_plan --extra-topic /ugv/move_base/global_costmap/costmap --extra-topic /ugv/move_base/local_costmap/costmap
```

复用既有只读采集器：已有goal/status/result/feedback、odom、scan、TF、速度链和诊断，额外仅加已接受目标、cancel、两条plan及两张完整costmap；不录点云或相机。`current_goal`保留目标帧/坐标，弥补短命Action发布者可能导致的首个goal消息缺失。整组交付`motion.bag`、`manifest.json`、`rosbag.log`及人工记录的目标、实际位移/偏转、障碍出现/移走、取消/干预和停车情况；未知量留空。

## 离线重现与边界

WSL中只运行现有无设备、断网容器：

```bash
./tools/noetic.sh catkin_make install -j2
./tools/noetic.sh python3 tools/check_odom12_ros.py /workspace/.test-results/odom12-check-$(date +%Y%m%d-%H%M%S)
```

验证使用真实GlobalPlanner/DWA/costmaps和理想传感器模型，私有master，无driver/Guard接通。它验证配置、GetPlan、栅格标记/清除、周期重规划及取消输出，不声称真实融合、实车绕障或长距离导航通过。30×40m扩展差异另见 [对比清单](ODOM_COSTMAP_30X40_NOTES.md)。

2026-10-25离线结果：Catkin构建/安装通过；5项ROS检查通过，参数仅全局宽、高及`allow_unknown`三项变化，34项局部参数一致；安装后的RViz成功订阅8个显示话题；上述采集命令的6个附加话题及odom/result均录入bag，取消结果匹配，解码无错误。记录位于工作区`.test-results/odom12-preparation-20261025/`，其中模型缺少真实轮速/driver的提示保留，不计作实车通过。

车端部署记录为`.test-results/odom12-deploy-20261025/`：含覆盖前3个文件的恢复包、交付文件清单、ARM64构建日志、默认/12m参数及安装哈希核对结果。车端再次确认仅上述3项参数变化，局部34项一致。固件、driver、Guard/Mux逻辑未修改，默认启动仍为8m；使用本页的`global_costmap_config`才启用12m。
