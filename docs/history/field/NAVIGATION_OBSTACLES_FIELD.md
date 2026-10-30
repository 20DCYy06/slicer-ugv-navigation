# 小障碍与 DWA / TEB：现场操作单

**部署与 ARM64 安装状态见[部署记录](NAVIGATION_OBSTACLES_DEPLOYMENT.md)。** 以下命令在新文件已安装的车端执行；旧安装不含新增 RViz / 采集选项。终端名称按职责区分。速度、footprint、Guard 规则保持原样；默认 DWA。TEB 二维跟踪的严格禁倒车兼容检查尚未通过，见[结果](TEB_ODOM_NOTES.md)，不能当成已验收替换。

## 1. 启动，先静止看障碍

车端 `potato` 宿主机的**定位终端**：

```bash
cd /home/potato/slicer_ugv_ros1
./tools/slicer_shell.sh roslaunch slicer_bringup sensors.launch root:=/slicer
```

车端**导航终端**进入 ROS shell 后启动；只需改 `local_planner:=dwa` 为 `teb` 即可对照。`control_config` 是 Guard/Mux 参数，底盘控制模式由底盘终端单独决定：

```bash
cd /home/potato/slicer_ugv_ros1
./tools/slicer_shell.sh
ugv_cfg="$(rospack find ugv_bringup)/config"
roslaunch ugv_bringup slicer_odom.launch local_planner:=dwa connect_control:=true control_config:="$ugv_cfg/slicer_driver_wheel_speed_reference.yaml" global_costmap_config:="$ugv_cfg/rolling_global_12m.yaml"
```

此时不 release，不必启动底盘。正常：`/odom`、`/scan`、`odom→base_link` 更新，两张地图可见；重启导航后 Mux 为 startup held。Windows电脑的 **WSL观察终端**（`rescue@…`，已配置好网络和Docker Desktop）执行：

```bash
cd /home/rescue/slicer_ugv_ros1
./tools/rviz_wsl.sh perception
```

静态 preset 的 Fixed Frame=`base_link`，便于看实际高度；逐个打开/关闭 LIO body cloud、filtered cloud、scan_raw、scan、local costmap，别把 LIO 输出点云当作原始 Livox 数据。障碍分别放在车前 **0.5 / 1 / 2m**，记录距离基准、尺寸和方向；每处静置至少5秒、移走再等5秒。用水瓶和一个不透明物体对照，不推动小车试碰。

在另一车端 ROS shell（宿主机 `cd /home/potato/slicer_ugv_ros1 && ./tools/slicer_shell.sh`）作为**采集终端**，每个位置独立短采，等 `RECORDING STARTED` 再摆放：

```bash
python3 tools/collect_odom_test.py --navigation --perception --duration 20 --output evidence/perception-$(date +%Y%m%d-%H%M%S)
```

同时留存本次启动所用 `/slicer/config/slicer_config.yaml` 的 `sensors/frames/chassis/navigation` 相关节与 `sha256sum /slicer/config/slicer_config.yaml` 输出；这些文件内的掩码/安装值并非全部发布为 ROS 参数，manifest 中只有对应文件路径，不能独立还原。

判据：点云→scan_raw→scan→地图逐层均有该障碍，移走且新射线通过后能清除。哪层首次消失就查那层，见[最短定位表](SMALL_OBSTACLE_FINDINGS.md)。静止水瓶时断时续也算未通过；全 inf 只代表本次无有效投影返回，不代表物理上无障碍。若漏标或 TF 报错，停在静态检查，不做绕障；不靠清图继续。Inflation 彩色区域是代价，不等于车身；橙色 footprint 才是碰撞多边形。

## 2. 感知稳定后，空旷区对照运动

先确认车轮周围无线束/电池碰擦。车端唯一**底盘终端**（不与 WASD / 队友串口程序并开）：

```bash
cd /home/potato/slicer_ugv_ros1
UGV_SERIAL_DEVICE=/dev/c30d ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_driver.launch config_file:=/workspace/install/share/ugv_bringup/config/slicer_driver_body_speed.yaml control_mode:=body_speed publish_tf:=false
```

正常显示 `body_speed` / `0x16` / `0x93`，不应退回 wheel_speed 或 PWM。无新 fault、定位新鲜、Mux held 后开始。遇旧锁存故障才按当前固件的 Motor OFF + 重启 driver 流程清除；不能反复发目标试解锁。

采集终端先启动，保留结束后至少5秒；600秒是自动结束上限，显示 `CAPTURE COMPLETE` 后不再录制：

```bash
python3 tools/collect_odom_test.py --navigation --duration 600 --output evidence/nav-dwa-$(date +%Y%m%d-%H%M%S)
```

运动观察可关闭静态窗口，在同一WSL终端用 `./tools/rviz_wsl.sh navigation` 打开导航配置；两种模式均有 DWA/TEB local plan。**车端操作终端**先执行 `rosrun ugv_slicer_control control.py state`；下面每次只选一项，停稳并记录后再做下一项。正 x 是当前车头前方、正 y 左方，yaw 为弧度；`--relative` 在发出时变换一次并固定到 odom。

当前位置原地掉头，判断是否出现大平移：

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation goal.py send --relative --x 0 --y 0 --yaw 3.1416 --timeout 30
```

后方目标，观察能否转向后前进，而非倒退：

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation goal.py send --relative --x -1 --y 0 --yaw 3.1416 --timeout 90
```

二维目标，plan 只求路径，不运动：

```bash
rosrun ugv_navigation goal.py plan --relative --x 1 --y 1 --yaw 0.7854
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation goal.py send --relative --x 1 --y 1 --yaw 0.7854 --timeout 90
```

**每次结束先记录5秒，再 `control.py hold`；需要停止时不等待。** `SUCCEEDED` 的当前容差为15cm / 14.3°，不代表精确到点。对照 TEB 时先 hold，再 Ctrl-C 停导航 launch，改 `local_planner:=teb` 重开、另录 `evidence/nav-teb-…`，其余参数不变；重启后重新 release。先做原地目标；若出现 `negative body vx disallowed`，本次不兼容，保留失败，切回 DWA，不反复释放或改 Guard。

## 3. 简单绕障、巡航与停车

只用第1步已稳定可见的障碍，摆在已观察的直线路径中并留足两侧通道；不要先用漏检的瓶子验证运动。复用上面的 plan/send，将目标设为 `--relative --x 2 --y 0 --yaw 0`。看 GlobalPlanner 路径、所选 local plan 和实际车头；障碍无标记、贴近碰撞、轮卡住、fault 或持续抽搐时立即停止。

另在无障碍直线试相同2m目标：标记同一车体固定点、录像、量净位移和停止余程；巡航速度看持续匀速段，别用含起停/转弯的全程平均冒充。当前上限 **0.125m/s、0.25rad/s**，本轮不提高。三段 cmd 与 MCU 接受目标/实测分层判读见[速度表](NAV_SPEED_LIMITS.md)。

## 4. 停止与收包

```bash
rosrun ugv_navigation goal.py cancel
rosrun ugv_slicer_control control.py hold
rostopic echo -n 1 /cmd_vel
```

确认 Mux held、最终 Twist 零、实车停稳；软件零值不能代替目视确认。异常直接使用物理急停 / Motor OFF。采集终端 Ctrl-C 收包，带走整个新 evidence 目录（bag、manifest、rosbag.log），附 planner、目标、实测位移/转角/停车余程、障碍出现/移除时刻、是否干预；未测留空。回退仅需 hold 后将导航改回 `local_planner:=dwa`，无需动固件或定位；关闭诊断投影不会影响原 `/scan`。
