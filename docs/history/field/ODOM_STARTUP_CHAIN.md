# odom 现场启动链与有效配置

车端独立新工程的ROS控制版本为 `f4bd9ee`，在Jetson aarch64/arm64镜像中完成无设备、断网安装，包含已批准的独立LIO新鲜度检查；采集器TF兼容修复 `598c899` 已同步。换电后助手恢复原定位和导航Launch，分别为后台容器 `ugv-sensors`、`ugv-navigation`，D由用户启动，通信节点保持。真实输入、TF、wheel_speed配置/零速遥测与GetPlan-only已再次核对通过；助手未启动底盘driver、未release或发送目标/非零速度。合并固件 `956eca3` 的非零验收仍待现场执行。

## 入口与责任

| 入口 | 实际启动内容 | 重要参数、输入和输出 |
|---|---|---|
| `tools/slicer_shell.sh` | 只打开已有 Docker/Noetic 环境 | 默认镜像 `industrial-cup/nav-slicer:20261023-dwa`，master/IP 为电台地址 `192.168.10.33`；先加载 `/slicer/catkin_ws/devel_navigation`，再加载 `/workspace/install`；队友树只读挂到 `/slicer` |
| `slicer_bringup sensors.launch root:=/slicer` | 原定位与雷达链，内含 EKF；没有底盘 driver | Livox → FAST-LIO `/Odometry` → `slicer_lio_projector` `/lio/odom`；`slicer_ekf` 同时消费 `/wheel/odom`，输出 `/odom` 与 `odom→base_link`；滤波点云 → `/scan_raw` → `/scan` |
| `ugv_bringup slicer_driver.launch` | **我方** `ugv_slicer_control/c30d_driver_node.py`，节点 `/slicer_c30d_driver` | 默认 `config_file=$(find ugv_bringup)/config/slicer_driver.yaml`、`control_mode=""`、`publish_tf=false`、`observe_motor_registers=false`；空模式覆盖使用配置文件中的 `pwm_reference` |
| `ugv_bringup slicer_odom.launch` | 默认只启动 `/ugv/move_base` | 默认 DWA、`connect_control=false`；输入 `/odom`、`/scan`，两级 rolling costmap 都在 `odom`；输出隔离在 `/ugv/cmd_vel/nav_raw` |
| 同入口 `connect_control:=true` | 上述规划器，加 `/slicer_nav_arc_guard` 与 `/slicer_velocity_mux` | `/ugv/move_base → /cmd_vel/nav_raw → Guard → /cmd_vel/nav → Mux → /cmd_vel → driver`；默认 `start_mux=true`，启动保持，不自动 release |

`odom.launch` 这个通用入口自身默认 TEB，`slicer_odom.launch` 显式覆盖为 DWA；不要混用两个入口的默认值。导航 launch 不会启动传感器、EKF、driver 或 TF 广播器。

### Slicer 定位来源与依赖

下述Slicer定位源码说明最初来自本地 **2026-10-23 审查快照**；本轮没有修改队友树：

`C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/review_20261023/slicer_ugv_review_2026-10-23/teammate_workspace_reference/catkin_ws/src/slicer_bringup/`

实际读取了 `launch/sensors.launch`、`scripts/lio_projector.py`、`scripts/sensor_geometry.py` 及该快照的统一配置。快照中未包含 `.generated/fusion.yaml`，故不把历史文件当作现场有效融合参数。

本次车端只读观察到 `/lio/odom`、`/odom`、`/scan` 约10Hz持续更新；driver尚未启动，故 `/wheel/odom` 当前无消息，尚不能确认实际wheel_speed握手、轮速比例或非零输出。定位已运行，本次只需由用户重开A/D，不重开传感器或另起fusion。

- `sensors.launch` 加载 `/slicer/.generated/sensor_launch.yaml`、`lio.yaml`、`fusion.yaml`、`scan.yaml` 和 `MID360.json`；这些文件必须已经存在，本轮不在队友只读树重新生成。
- 依赖 `slicer_bringup`、`livox_ros_driver2`、`fast_lio`、`robot_localization`、`slicer_navigation`、`pointcloud_to_laserscan`；我方规划依赖 `move_base`、`global_planner`、`dwa_local_planner`，显式选 TEB 才依赖 `teb_local_planner`。
- 用户已完成的融合静态审查基线是 **LIO x/y/yaw + wheel vx**。`slicer_ekf` 读取生成参数并 remap `odometry/filtered→/odom`，不另起第二个 fusion 节点；现场采集 `/slicer_ekf` 参数确认 `odom0/odom1/config` 与这一基线相符。
- `lio_projector` 保留 `/Odometry` 的消息时间戳，以首次底盘位姿建立局部原点，只发布 `/lio/odom`，不发 TF。轮式积分有独立原点，不能用两者绝对 x/y 相减判断尺度。
- `slicer_geometry` 发静态 `base_link→laser_link`、`base_link→body` 等几何变换；动态 `odom→base_link` 由 EKF 唯一负责，driver 必须保持 `publish_tf=false`。不加恒等 TF 掩盖坐标问题。

## wheel_speed 的明确启用路径

由用户在Jetson宿主机启动D，配置使用容器内绝对路径：

```bash
cd /home/potato/slicer_ugv_ros1 && UGV_SERIAL_DEVICE=/dev/c30d ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_driver.launch config_file:=/workspace/install/share/ugv_bringup/config/slicer_driver_wheel_speed_reference.yaml control_mode:=wheel_speed publish_tf:=false
```

这两个选择都明确给出，避免裸 launch 沿用 PWM 默认。新模式路径为：`load_config` → `Chassis` 唯一串口 owner → `configure(encoder_signs)` 发 `0x12` 独立 PWM/编码器符号 → 匹配 nonce ACK 与新 configured 遥测 → 循环将 Twist 换成 ABCD int16 mm/s，仅发 `0x10`。故障或配置丢失退出，没有非零 PWM 自动降级。MCU ACK 证明接受，不证明车轮运动。

不能用节点名证明正在运行我方 driver：队友旧 driver 也使用 `/slicer_c30d_driver`。A/D启动后，以下只读核对由助手继续执行，用户无需再逐条搬命令：

```bash
rospack find ugv_bringup
rospack find ugv_slicer_control
roslaunch --files ugv_bringup slicer_driver.launch
rosparam get /slicer_c30d_driver/provenance
```

前三条说明**当前 shell 将选择什么**；已运行节点仍须看 provenance 的 `control_mode`、`config_file`、`effective_config`、`module_paths_sha256` 和 `mcu_configuration`，并在本次 diagnostics 中确认 `serial_tx kind=16`、新鲜遥测 tick/ACK。`control_mode` ROS 参数可能只是覆盖值，单看参数不等于有效模式。无 provenance/诊断，或路径仍为 `slicer_wasd_control`，不得把本地新代码能力算到旧进程头上。

## 默认值与现场有效值

| 内容 | 源码/选定配置默认 | 现场必须确认 |
|---|---|---|
| driver 模式 | `slicer_driver.yaml: pwm_reference` | 显式参考配置 + `wheel_speed`，有效模式一致；新鲜 CONFIG ACK，0x10 发送与接受 |
| USB 身份 | `1a86:55d4 / 5CF0016377`，115200 | 安装 `tools/udev/99-c30d.rules` 后使用 `/dev/c30d`；Shell解析为真实tty再映射，driver仍按USB身份复核；唯一owner，别同时启动旧 `start_ros.sh` |
| 轮速配置 | 60000 counts/rev、85 mm、轮距0.15m；ABCD=右后/左后/右前/左前；PWM `[-1,1,1,1]`、编码器 `[-1,1,-1,1]` | 有效配置逐项一致；几何与比例仍需实测，不能以软件换算自证标定 |
| 动态 TF | EKF `odom→base_link`，driver false | 唯一发布者，消息 frame 与 TF 一致、时间戳推进 |
| Guard | 命令0.5s、grid1.5s、TF0.5s、独立LIO0.5s，20Hz；无turn boost、无倒车 | `lio_topic=/lio/odom`，同时检查接收新鲜度和源header有效/推进；实际planner/costmap ns与限制取交集，0.5s不是物理停车时长 |
| Mux | 配置读取队友 `teleop_params.yaml`；20Hz、命令0.25s，`manual_release_owner=/ugv/control` | `manual_hold=true` 启动保持；teleop不活跃，最终 `/cmd_vel` 持续零；held 本身不屏蔽人工 teleop |
| driver/MCU 停止 | host命令0.25s，遥测超时0.5s；MCU Watchdog500ms | 当前运行配置和实际遥测；本轮不另开串口读取，不把 ready 锁存值当持续健康 |
| DWA | x上限0.125m/s，w上限0.25rad/s，xy容差0.15m、yaw容差0.25rad | 未标定限幅/容差；不能把0.5m Action成功当实际走足0.5m或±5cm停车通过 |

`/slicer_c30d_driver/diagnostics` 内可用 MCU 反馈含目标、raw counts、名义轮速、软件 duty、接受序号、tick、电压、fault。软件 duty 不是电桥 CCR 或转矩；`observe_motor_registers:=true` 会增加串口查询，本轮只读采集工具不替用户打开它，也不新开串口。

`f4bd9ee` 的Guard已独立订阅 `/lio/odom`：即使EKF odom/TF、grid和raw仍更新，LIO停更、重复源header或旧header也不能维持非零出口。该行为已做隔离软件验证；现场仍需核对当前运行参数与LIO抖动，出口归零不等于车轮已物理停稳。所有运动、取消和hold仍由用户现场执行。

## 安装与入口验证记录

**当前车端：** `f4bd9ee` 已在Jetson ARM64隔离容器中安装完成；新driver/Guard及helper实际安装内容与本次源码哈希一致，显式wheel_speed Launch解析得到四项正确参数。用户启动A/D后的实际CONFIG nonce ACK匹配、configured/ready有效，轮速/遥测tick推进，采样全为0x10四零；LIO/scan约10Hz、融合odom/轮odom约19Hz，TF唯一owner和前方0.5m的22点GetPlan路径均已确认。助手只读观察driver已有反馈，没有打开串口。[源码更新文档](ODOM_SOURCE_UPDATE.md)仅留作重新安装参考，无需重复执行。

**此前WSL验证：**

在已有 `industrial-cup/noetic-dev:local` 镜像中，以 `--pull never --network none`、无设备映射完成临时 Catkin install，未触碰主工程 build/devel/install。四组实际 `roslaunch --dump-params` 与 XMLLoader 解析确认了默认导航、接控制链导航、默认 PWM driver、显式 wheel_speed driver 的安装路径、参数与 remap；安装后的 Python 模块可导入，当前主工程 `test_driver.py` **50/50 通过**。未启动节点或打开串口。证据在 .test-results/odom-preflight/install-check（原归档未随仓库收录：`../../.test-results/odom-preflight/install-check/`）。

上述WSL临时安装只证明当时源码的安装与launch解析：该开发镜像缺少move_base/DWA、robot_localization、pointcloud_to_laserscan、tf等运行包，未据此声称规划插件加载或传感器运行通过。当前车端安装结果见本节首段，节点运行状态仍由A/D启动后的只读检查确认。

另对本地默认运行镜像 `industrial-cup/nav-slicer:20261023-dwa` 做了无节点包路径查询：move_base、global_planner、DWA、TEB、robot_localization、pointcloud_to_laserscan、tf/tf2_ros 均存在；Slicer/Livox/FAST-LIO 包需由 `/slicer/catkin_ws/devel_navigation` overlay 提供，裸镜像没有。证据为 `.test-results/odom-preflight/runtime-image-dependencies.json`。这是本地 amd64 镜像检查，不能代替 Jetson 的镜像、overlay、生成参数与运行进程核对。
