# 迁移边界与来源

本文首先记录第一次软件交付。后续最小实车接线已选择性保留既有Guard/Mux适配版，新增真实身份 `/ugv/control` 的显式交接；独立审查后又在新工程加入受控C30D driver副本及诊断，并修改Guard的禁倒车出口。当前用法见[现场短单](field/SLICER_ODOM.md)，证据范围见[接线验证](field/INTEGRATION_VALIDATION.md)。以下“本阶段不部署/未迁移归还”的描述保留第一次交付语境，不代表当前代码仍缺这些接口。

本轮修复仅在本地进行，没有部署新driver、连接硬件或修改队友树。新driver独立启动，导航launch不包含它；现场仍运行旧driver时，不能把本包已有连续函数当作已接入修复。新副本的实际导入链、串口编解码和诊断须以本轮隔离假传输测试及安装路径核验为依据，不能沿用此前软件模型的通过结论。

本工程是独立 ROS1 Noetic Catkin workspace，根目录运行 `catkin_make`。旧工程 `/home/rescue/industrial_cup_ros1` 只读；源码参考 HEAD 为 `65308aa`，已知生产执行版本为 `3edbcd7`，两者不能视为同一次运行。本阶段不部署 Jetson、不修改队友目录、不启动真实驱动、不发送实车目标。

## 阶段 1：已有 odom 上的标准导航

只消费既有 Slicer 的 `/odom`、`odom → base_link` TF 和 `/scan`（`base_link` 中的 360° 激光）。既有定位链为 LIO 原始 `/Odometry`（`camera_init → body`），经 `/lio/odom` 投影；`/slicer_ekf` 融合 LIO 的 x/y/yaw 与 wheel vx，输出 `/odom` 并发布 `odom → base_link`。这些节点和生产 TF 的所有权仍在原系统。本工程不重复启动它们，不发布 `map → odom`，不创建静态图或 AMCL。

新工程启动标准move_base和GlobalPlanner，global/local costmap均在odom中滚动。通用odom入口默认TEB；2026-10-24修复增加标准DWA并作为Slicer入口默认，`local_planner:=teb`保留原对照。任务接口仍为标准 `move_base_msgs/MoveBaseAction`，没有旧Operations、Session、Contract、目标自动重发者。临时阻塞由move_base在patience内对原目标重规划；ABORTED后不自行恢复任务。DWA参数来自实际安装Noetic1.17.3默认及同一低速限制，角轨迹分辨率.025由单参失败/成功对照确定，详见[本轮记录](field/COMPLETE_FIX_VALIDATION.md)。

默认输出为 `/ugv/cmd_vel/nav_raw`，用于隔离软件调试，不能据此声称已接上物理链。原系统链为：

```text
/cmd_vel/nav_raw → Slicer Guard → /cmd_vel/nav → Mux → /cmd_vel → C30D
```

本工程的速度只能对接 Guard raw 入口，禁止直接连接 `/cmd_vel`。后续实际外部对接必须同时满足：

- launch 显式设置 `cmd_vel_topic:=/cmd_vel/nav_raw`。
- 外部 Guard 设置 `planner_namespace=/ugv/move_base`、`costmap_topic=/ugv/move_base/local_costmap/costmap`；更换 namespace 时二者同步修改。
- 外部 Guard 使用正确的 `config_file`、`control_config_file`，且 `turn_compensation_enabled=false`、`allow_patrol_spin=false`；仅修改本工程的 TEB 参数不能代替这些设置。

源 Mux 启动时 `manual_hold=true`。旧 `/cup_nav/manual_intent` 和 state 接口使用 `std_msgs/String` JSON；release 又绑定 `/cup_nav_executor` 以及 Mux 实例和代次。本阶段不复制这套协议，不提供独立实车“归还控制权”操作，也不声称 Guard/Mux/C30D 已联通。第三阶段再做必要的最小解耦。

## 逐文件来源与取舍

“当前 Slicer 源码”特指已选定 `/home/rescue/industrial_cup_ros1/validation/slicer-build-inputs/681de31eaeabf5ab2d96e9822d453de563ce8319c4c6227e64294d910688e07f/workspace` 源码；不代表扫描历史输入。旧 bridge 下列路径相对于 `/home/rescue/industrial_cup_ros1/catkin_ws/src/cup_nav_bridge`。navigation/TEB 已从选定源码提取，不复制整个工作区或生成的 navigation。

| 新文件 | 旧来源或新编写内容 | 保留理由与变化 |
| --- | --- | --- |
| `src/ugv_bringup/package.xml`、`CMakeLists.txt` | 新建 Catkin 包元数据 | 只承载 launch/config，不复用旧运维依赖。 |
| `src/ugv_navigation/package.xml`、`CMakeLists.txt` | 新建 Catkin 包元数据 | 声明标准 move_base/actionlib 等依赖，不依赖旧 bridge 内部 Python 包。 |
| `src/ugv_bringup/launch/odom.launch` | 参考旧 `launch/initialization_local.launch`、`launch/slicer_planner.launch` 的导航接线，新写入口 | 只包含导航；namespace、topic、frame、配置目录显式可配。剥离驱动、定位启动和硬件生命周期。 |
| `src/ugv_navigation/launch/move_base.launch` | 参考旧 `launch/slicer_planner.launch`，新写参数加载与 remap | 将 odom/scan/raw 速度接入标准节点；不发布生产 TF。 |
| `src/ugv_bringup/config/move_base.yaml` | 选定 `slicer_navigation/config/navigation_params.yaml`；旧 `config/slicer_planner_overrides.yaml`、RTK costmap 策略 | 保留 GlobalPlanner/TEB。选择 GradientPath：`use_grid_path=false`、`old_navfn_behavior=false`，这是来源选择，不是对现场提取失败的修复证明。缺少失败时原始cost字节等条件，本轮不改四组合或unknown策略。禁用旋转及清图 recovery、禁止 make_plan 隐式清图。 |
| `src/ugv_bringup/config/teb.yaml` | 当前 `slicer_navigation/config/teb_local_planner_params.yaml`；旧 `config/slicer_planner_overrides.yaml`、`launch/initialization_local.launch` 的限速 | 选择性保留 TEB 算法参数；低速和加速度为未标定调试上限，不宣称适配实车动力学。 |
| `src/ugv_bringup/config/costmap_common.yaml` | 当前 `config/slicer_config.yaml`、`scripts/prepare_workspace.py`；旧 `config/rtk_costmap_global.yaml`、`rtk_costmap_local.yaml` | 只保留激光 obstacle/inflation 层和 Slicer 传感器约定；无 StaticLayer。保留 unknown，GlobalPlanner `allow_unknown=false`。 |
| `src/ugv_bringup/config/costmaps.yaml` | 旧 `config/rtk_costmap_global.yaml`、`rtk_costmap_local.yaml` 的滚动窗口思路 | 选择 global 8×8 m、local 4×4 m 软件窗口；没有迁移 RTK 坐标和地图管理。 |
| `src/ugv_bringup/config/footprint.yaml` | 旧 `config/vehicles/slicer.json` 中较大的 Slicer 调试包络 | costmap footprint 与 TEB vertices 使用同一多边形；有意保留更大的保守包络，理由见下。 |
| `src/ugv_navigation/scripts/goal.py` | 新编写标准 actionlib 客户端 | 直接发送一个 MoveBaseGoal；超时/Ctrl-C 取消自己的目标；不复用旧 CLI、Operations 或 Session。 |
| `src/ugv_slicer_control/scripts/c30d_driver_node.py`、`c30d_link.py`、`chassis_core.py`、`serial_transport.py` | 独立审查所指明的队友driver及必要SLC2/轮速依赖；来源hash见`ugv_slicer_control/SOURCES.json`的`controlled_driver_reference` | 受控副本实际调用本包连续映射，增加标准诊断；串口唯一所有权、限幅、零、Watchdog和轮速保留。独立配置/launch位于`ugv_bringup`，不覆盖或自动启动队友driver。 |
| `src/ugv_navigation/test/*`、`tools/test_noetic.sh` | 新写隔离验证设施 | 只为软件模型提供输入和观察；不迁移旧运行日志、bag 或硬件模拟结果。结果以 `VALIDATION.md` 为准。 |
| `README.md`、`docs/INTERFACES.md`、本文件、`docs/VALIDATION.md` | 新工程说明、来源与本轮验证记录 | 明确接口和证据边界，不把旧执行结果当成本工程通过结果。 |

第一次软件交付没有复制旧运行日志、生成navigation、GNSS凭据或协议、驱动、backup、历史封存输入或缓存。阶段2/3的文件清单只是索引，不表示已迁移。本轮driver副本仅从明确选定源码迁移必要的driver/SLC2编解码/轮速依赖，来源见控制包来源清单，不扫描历史输入；队友树仍只读。

独立审查后的最小控制变更：新driver调用`control_core.wheel_pwm → slicer_manual.wheel_pwm`连续函数；由单一串口所有者发布标准`DiagnosticArray`，保留原限幅、立即零、反向过零、命令/遥测保护和轮速接口。`slicer_driver.launch`与导航入口分开。Guard禁倒车时整条拒绝负vx请求，不再改成纯转；该变更使策略明确，不保证TEB侧方目标的前向进展。连续函数也不保证约71‰纯转起转或消除历史超调。

### 未标定参数与 footprint 差异

第一次交付选定源的总配置hash前缀为`4694a1d1…`，由其`slicer_config.yaml + prepare_workspace.py`生成的footprint是x ∈ [−0.145, +0.25] m、y ∈ [−0.12, +0.12] m。旧bridge的`vehicles/slicer.json`声明hash前缀`b787a974…`，对应更大包络x ∈ [−0.165, +0.27] m、y ∈ [−0.14, +0.14] m。这些是早先选定源，不代表10月23日后续队友快照或当次运行参数；不能用旧声明证明当前生成值。

新工程有意保留后者作为保守调试包络，未实测、未标定，也不等于真实车体及载荷的完整扫掠范围。`footprint_padding=0`，同一文件同时加载到两个 costmap 和 TEB，避免三处形状分叉。

以下均是软件调试值：前进 0.125 m/s、角速度 0.25 rad/s、线加速度 0.2 m/s²、角加速度 1.0 rad/s²，以及 global 8 m/local 4 m 窗口。TEB 的正反向界限 `max_vel_x_backwards=0.02` 用于优化器，不能解释为允许实车倒车；外部 Guard 仍须禁止倒车。到点容差、传感器时限、patience 和 inflation 等也没有硬件验收意义。

禁止通行 unknown、禁用 move_base recovery、没有盲目清图。TEB 内部的 `shrink_horizon_backup`/`oscillation_recovery` 是优化器行为，不代表引入旧任务恢复或允许旋转清图。

## 后续索引：尚未迁移

阶段 2 才考虑 RTK/ENU、对齐、robot_localization 及定位有效性，当前不建立空壳 `ugv_localization`。候选源文件：

- `cup_gnss/enu.py`
- `cup_nav_bridge/rtk_coordinates.py`、`rtk_alignment.py`、`slicer_lio_pose.py`
- `cup_nav_bridge/rtk_inputs.py`、`rtk_estimation.py`、`nodes/rtk_candidate.py`
- `cup_nav_bridge/config/rl_global_candidate/active.yaml`

阶段 3 再选择迁移 `cup_nav_bridge/recovery.py`、`executor_core.py`、`health.py` 及相应纯逻辑测试，先剥离 RTK、地图管理和旧会话/运维框架耦合；只增加标准导航确有必要的有界恢复语义及控制权交接。

软件模型只能在无设备、`--network none`、独立 ROS master 的容器运行。即使 move_base/TEB 到点，也不能证明定位精度、实车制动、Guard/Mux/C30D 安全链或配送停稳。构建与本轮真实软件验证的具体通过/失败项见 [VALIDATION.md](VALIDATION.md)，本文件不预先宣称通过。

早期模型直接积分raw Twist，未覆盖Guard/Mux/driver；后续控制链模型及真实driver假传输测试须分别列明。MCU ack只表示接受命令，回报PWM为timer ticks；参考固件`00/coast`意图不能证明实际Flash或电桥状态。本轮不改角速、到点容差、GlobalPlanner或固件，不展开新任务状态机、RTK或自动恢复。

## 新增机制为什么存在

没有新增任务状态管理器、定位转发层或自定义 ROS 协议。`goal.py` 只是可替换的 actionlib 客户端：有限数值/正超时检查阻止 NaN、无穷及无效等待预算进入目标；取消使用 actionlib 自有 goal handle，避免 Ctrl-C 误取消队友目标。`cancel` 子命令明确是用户要求的全部取消。

costmap 的 unknown 和 scan 新鲜度、TF 时效检查使用成熟 ROS 组件自身实现；没有它们，规划器可能使用未观测区域或失联前的观测继续运动。`tools/noetic.sh` 只隔离运行环境，防止测试误接真实 master；不参与任务提交或运行许可。源码 hash 仅用于本次迁移审查，不是启动或运动门禁。完整来源清单见 [MIGRATION_SOURCES.json](MIGRATION_SOURCES.json)。

标准行为核对来源：[move_base Noetic 实现](https://github.com/ros-planning/navigation/blob/noetic-devel/move_base/src/move_base.cpp)、[GlobalPlanner](https://github.com/ros-planning/navigation/blob/noetic-devel/global_planner/src/planner_core.cpp)、[ObstacleLayer](https://github.com/ros-planning/navigation/blob/noetic-devel/costmap_2d/plugins/obstacle_layer.cpp)。实际验证仍以新工程日志为准。
