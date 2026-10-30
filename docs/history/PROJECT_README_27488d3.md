# Slicer UGV · ROS1 Noetic

## 2026-10-30 离线候选

新增独立的在线建图与 D455 深度避障入口，保留现有定位和底盘控制链。
请先读 [本轮使用说明与验证范围](OFFLINE_ONLINE_NAV_20261030.md)。
新入口为 `roslaunch ugv_bringup slicer_online.launch`；默认不启动传感器、
相机或底盘驱动，不连接真实控制入口，消费已有传感器/录包输入。
本轮未部署。旧 README 下方的现场版本描述为采集时保留的历史记录，
不能替代本轮说明和用户 2026-10-29 的固件审查结论。

## 既有 odom 导航说明（采集时保留）

独立 Catkin 工作区，当前范围为 **odom 局部导航与最小实车接线**。已有 Slicer 定位提供 `/odom`、`odom→base_link` 和 `/scan`；本工程启动标准move_base和GlobalPlanner。Slicer入口默认DWA，通用odom入口默认TEB；可用 `local_planner:=dwa|teb` 明确选择，不需要GNSS、AMCL或静态地图。

2026-10-26 的[小障碍与 DWA/TEB 现场操作单](field/NAVIGATION_OBSTACLES_FIELD.md)集中说明启动、分层观察、对照与收包；本次车端同步和 ARM64 安装状态见[部署记录](field/NAVIGATION_OBSTACLES_DEPLOYMENT.md)。[离线验证](field/NAVIGATION_OBSTACLES_VALIDATION.md)、[感知发现](field/SMALL_OBSTACLE_FINDINGS.md)、[TEB 限制](field/TEB_ODOM_NOTES.md)、[速度瓶颈](field/NAV_SPEED_LIMITS.md)区分软件验证和实车待验。TEB 已复用队友路径偏好，但本次严格 Guard 兼容检查未通过，默认保持 DWA，不将其作为已验收替换。

新增[12×12m滚动全局Costmap测试与现场短单](field/ODOM_ROLLING_12M.md)，已于2026-10-25部署到Jetson独立工程并完成ARM64构建安装。通过现有`slicer_odom.launch`的可选`global_costmap_config`加载，默认8m配置不变，局部4m配置不变；该测试profile允许全局规划经过unknown，局部与Guard检查保持原样。[30×40m固定/滚动差异](field/ODOM_COSTMAP_30X40_NOTES.md)单列，未提前重构。

当前提供显式选择的[0x16车体IMU模式](field/BODY_SPEED_0X16.md)，复用队友已有固件接口、现有串口owner与Guard/Mux；原PWM/0x10模式保留，默认不切换。软件接口验证与实车转向验收分别记录。

**实车入口：[Slicer odom 现场短单](field/SLICER_ODOM.md)。** 修复允许同步到Jetson独立新工程；驱动启动、release/hold与短目标均由用户现场执行，不重复已完成的WASD底盘测试。当前记录见[继续修复验证](field/COMPLETE_FIX_VALIDATION.md)：已取得两次直行自行停车记录，有效转向及实物障碍响应待验。

最新轮速接口改动仅在本地验证，未部署或启动真实driver。原、新现场driver均使用开环PWM；本地现提供显式可选的MCU轮速协议，默认不切换。实际固件已由用户确认并经本轮全片双读匹配21日基线；方向及速度闭环仍待实测。三处时基修复与烧写记录见[C30D固件记录](firmware/C30D_TIMEBASE_2026-10-24.md)，既有接口分析见[底盘速度接口核查](field/BASE_CONTROL_FINDINGS.md)。

```text
现有 Slicer LIO + wheel → local EKF → /odom + TF ─┐
现有 /scan ─────────────────────────────────────┼→ /ugv/move_base
                                               └→ raw Twist
控制接线：raw → 选定适配版 Guard → Mux → 独立 C30D driver → MCU
早期导航软件模型：raw → TEST_ONLY 理想运动学模型 → odom / scan / TF
```

- `src/ugv_bringup`：一处管理 launch 和参数。
- `src/ugv_navigation`：标准导航启动、直接 Action 客户端和软件测试。
- `src/ugv_slicer_control`：既有Guard/Mux的选定适配版、一次性交接客户端，以及新工程受控C30D driver副本。driver默认连续PWM、可显式选择已有MCU轮速协议，通过标准诊断话题记录命令与遥测；导航launch不自动启动它。

不创建无实现的 localization/interfaces 包：局部定位直接消费既有接口。RTK和自动恢复暂不展开。没有新任务状态机、Session、Contract 或场次登记。

## 当前 WSL：一条命令测试

```bash
cd /home/rescue/slicer_ugv_ros1
./tools/test_noetic.sh
```

宿主机未装 ROS1，该命令使用已有 `industrial-cup/nav-slicer:20261023-dwa` **环境镜像**（包含Noetic DWA 1.17.3），只挂本工作区，以当前用户运行、`--network none`、无设备，独立 ROS master。它构建、运行测试、检查 JUnit 并验证 install；输出与退出码在 `.test-results/run-*`。不调用旧项目脚本，不扫描或修改旧项目。

也可以构建本工程的环境镜像（需要网络）：

```bash
docker build -t ugv-noetic:local .
UGV_NOETIC_IMAGE=ugv-noetic:local ./tools/test_noetic.sh
```

## ROS 环境中的最少命令

在已安装 Noetic 的终端操作；WSL 可先 `./tools/noetic.sh` 进入环境 shell。**每次 noetic.sh 是独立容器/master 网络，不会连接另一个 shell 的模型。** 自动闭环请用上述测试命令；原生 Noetic 多终端则使用同一 ROS master。

当前 WSL 想手动发目标时，在同一个环境 shell 内运行（只用于隔离软件模型；演示文件仅在源码工作区提供，不安装到 install）：

```bash
./tools/noetic.sh
# 以下在该 shell 内；首次先 catkin_make -j2，再 source devel/setup.bash
roslaunch ugv_navigation software_demo.launch > /tmp/ugv-demo.log 2>&1 &
ugv_launch_pid=$!
rosrun ugv_navigation goal.py send --x 0.8 --y 0 --yaw 0
rostopic echo -n 1 /ugv/move_base/status
rosrun ugv_navigation goal.py cancel
kill -INT "$ugv_launch_pid"
wait "$ugv_launch_pid"
```

该演示模型只积分raw Twist，不经过Guard/Mux/driver，也不包含PWM、电机、电桥或地面响应。标准ROS组件的Action成功只证明该软件模型到点。控制链与假串口测试范围另见[验证记录](VALIDATION.md)，均不替代实车结果。

```bash
source /opt/ros/noetic/setup.bash
cd /home/rescue/slicer_ugv_ros1  # 容器内为 /workspace
catkin_make -j2
source devel/setup.bash

# 已有 odom / scan / TF 时：仅启动导航，不启动传感器或底盘
roslaunch ugv_bringup odom.launch
```

以下在同一 ROS master 的另一个已 source 终端执行；坐标是 `odom` 中的米和弧度，需自行选择已观测、窗口内的短目标：

```bash
# 一个目标；Ctrl-C 或超时只取消本命令的目标
rosrun ugv_navigation goal.py send --frame odom --x 0.8 --y 0 --yaw 0

# 状态、反馈；最终结果在 /ugv/move_base/result
rostopic echo /ugv/move_base/status
rostopic echo /ugv/move_base/feedback

# 明确取消该 Action server 的全部目标；在 status/result 确认 ACK
rosrun ugv_navigation goal.py cancel

# 单独观察定位和规划，不发运动目标
rostopic hz /odom
rostopic hz /scan
rosrun tf tf_echo odom base_link
rostopic echo /ugv/move_base/GlobalPlanner/plan
```

停止：先取消目标并观察终止状态，再在 roslaunch 终端 `Ctrl-C`。实车按现场短单先hold并确认停车，保留Guard/Mux/Watchdog/急停全链；退出导航进程不是制动验收。

## 参数与物理接线边界

参数集中在 [ugv_bringup/config](../../src/ugv_bringup/config)。`.125 m/s`、`.25 rad/s`、包络、8m/4m滚动窗口及故障等待预算均为未标定调试值；不能据此承诺实车速度、停车距离或安全失联时间。扫描未覆盖处为 unknown，规划器拒绝；窗口外目标应拆分为可观测短目标。

`odom.launch` 默认 raw 输出 `/ugv/cmd_vel/nav_raw`，用于软件或独立规划观察。`slicer_odom.launch connect_control:=true` 接 `/cmd_vel/nav_raw` 与选定适配版Guard，正确设置planner namespace和costmap；Mux启动held，通过参数明确授权新 `/ugv/control`，由一次性 `control.py release/hold` 交接。**不能改成 `/cmd_vel` 或 Guard 的输出 `/cmd_vel/nav`。** 导航入口不启动driver、不自动release；独立入口为 `roslaunch ugv_bringup slicer_driver.launch`，仅由用户在原driver停止后启动，详见[现场短单](field/SLICER_ODOM.md)。

禁倒车时Guard整条拒绝负vx请求并给出hold_reason，原始纯转仍允许；这不保证TEB能找到前向侧方轨迹。连续PWM映射也不保证`.25rad/s`约71‰纯转能起转或消除历史超调。本轮不改GlobalPlanner组合、角速上限、到点容差或固件；MCU ack表示接受命令，回报PWM是timer ticks，均不是车体已执行或停稳的证明。

独立driver的`observe_motor_registers:=true`可启用已有`0x15/0x92`寄存器观察，默认关闭。它仅在真实串口的正常非零命令发送后最多每秒请求5次，不等待回复；实际回复经现有`~diagnostics`的`motor_registers`事件记录，缺回复不改变命令刷新。底层仍保留0.2秒串口写超时，每个69字节回复约占6毫秒UART发送时间。`0x92`没有MCU时间戳或原请求序号，只能按主机接收时间和MCU帧序号观察；CCR、enable及GPIO寄存器不能证明电流、轮子运动或停稳。

旧项目 `/home/rescue/industrial_cup_ros1` 作为只读回退基线。新源码仅放到车端独立 `/home/potato/slicer_ugv_ros1` 做原生构建和无目标验证，队友树只读。实际运动由用户执行。

[迁移来源与取舍](MIGRATION.md) · [ROS 团队接口](INTERFACES.md) · [本次修复验证与未解决项](field/REVIEW_FIX_VALIDATION.md) · [历史软件验证](VALIDATION.md)
