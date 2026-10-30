# Odom 导航的 TEB 对照与禁倒车边界（2026-10-26）

本轮仅在本地 WSL 的无设备、`--network none` 容器中检查现有 TEB。没有连接车端、SSH、部署、串口、启动实车驱动、release 或实车目标。默认入口 `slicer_odom.launch` 继续使用 DWA；TEB 是显式选择的对照项。下文中的动作与传感器均是容器内的软件试验，不能作为实车结论。

## 配置来源与保留值

参考队友本次快照 `deliverables/slicer_odom_review_20261026/external_source/Slicer_Workspace/` 中：

- `catkin_ws/src/slicer_navigation/config/teb_local_planner_params.yaml`：局部 band、前向初始化、朝向覆盖、3×3 优化迭代、单 band 与障碍代价策略。
- `config/slicer_config.yaml` 的 `navigation`：`min_turning_radius=0`、`teb_forward_drive_weight=900`、`max_reverse=0`、`min_obstacle_dist=0.144`。
- `catkin_ws/src/slicer_navigation/tests/check_navigation.py`：队友生成配置应为 `max_vel_x_backwards=0.02`，并由出口约束车体前进。该文件读取 `.generated/navigation.yaml`；本次交付快照没有提供可直接审查的配置生成器实现，因此不把这些静态断言当作生成器已运行的证据。

我方 `teb.yaml` 已具备所需四轮差速模型：`max_vel_y=0`、`acc_lim_y=0`、`min_turning_radius=0`、`cmd_angle_instead_rotvel=false`。允许两侧轮反向进行原地旋转，不表示允许车体 `linear.x<0`。继续保留我方前进 `0.125 m/s`、角速度 `0.25 rad/s`、线加速度 `0.2 m/s²`、角加速度 `1 rad/s²`；不导入队友更高速度、起转 boost 或固件/轮速模型。

TEB、全局和局部 costmap 仍共用 `footprint.yaml` 的同一多边形 `[-0.165, 0.27] × [-0.14, 0.14] m`。`slicer_planner.yaml` 在 Slicer 入口补上原有 `min_obstacle_dist=0.144 m`，未放宽间距。12 m 全局窗口继续通过已有 `global_costmap_config` 覆盖启用；局部窗口仍为 4 m，没有复制队友 5 m 设置。`allow_unknown=true` 只影响全局求路，不把未知格改成已观测空闲，也不替代局部碰撞检查。

本轮在仅由 Slicer TEB 入口加载的 `slicer_planner.yaml` 中复用队友 `weight_optimaltime=1.5`、`weight_shortest_path=0.2`，改变时间与路径偏好；不改变 DWA。其来源是队友当前 launch 最后加载的 `teb_local_planner_params.yaml`，不是根据少量数据猜值。通用 `teb.yaml` 中旧 `1.0/0.0` 留作原有基线；Slicer 入口用这两项覆盖。`allow_init_with_backwards_motion=false`、`global_plan_overwrite_orientation=true`、forward weight `900`、速度/加速度和 footprint 不变。

## 标准 TEB 不能仅靠 YAML 硬禁负 vx

本机实际安装插件为 `ros-noetic-teb-local-planner 0.9.1-1focal.20250521.005922`；DWA 为 `1.17.3`。本次版本与头文件证据保存在 `.test-results/navigation-obstacles-20261026/plugin-package-inspection.log` 和 `plugin-headers.log`。

安装的 `teb_local_planner_ros.h` 在 `saturateVelocity` 注释中明确要求不要把 `max_vel_x_backwards` 设成零或负数，而应使用优化权重惩罚倒车。对应上游 [TEB 0.9.1 输出速度限制实现](https://github.com/rst-tu-dortmund/teb_local_planner/blob/0.9.1/src/teb_local_planner_ros.cpp) 及 [差速运动学误差项](https://github.com/rst-tu-dortmund/teb_local_planner/blob/0.9.1/include/teb_local_planner/g2o_types/edge_kinematics.h) 可用于核对。

因此保留正值 `max_vel_x_backwards=0.02`，它只是插件允许的反向速度幅度，不是任务授权倒车。`allow_init_with_backwards_motion=false` 控制初始轨迹方向，`weight_kinematics_forward_drive=900` 是有限权重的软优化项；二者均不能保证提取出的每条 raw Twist 都有 `vx>=0`，包括量级很小的负数。把 backwards bound 设成零、继续堆高权重或把微负值描述成零，都不能建立硬保证。

本轮不改 Guard/Mux。生产控制中，raw 始终只接 Guard 入口；任何负 vx 继续由 Guard 拒绝整条命令。代价是某些转向、后方目标或到点调整可能反复被拒绝而停滞。Action 在允许积分负 vx 的理想模型中成功，也不能证明它能通过严格 Guard 完成。标准配置不能解决的部分必须保留为未通过；默认 DWA 仍是本项目已有硬 `min_vel_x=0` 约束的选择。

## 本次离线检查

新工具 `tools/check_teb_ros.py` 只运行真实 `move_base`、TEB、GlobalPlanner、costmaps 和标准 `MoveBaseAction`，也支持 `--planner dwa` 以同一模型和病例对照 DWA。传感器/里程计/TF 与运动使用已有 `TestOnlyDifferentialPlant`，直接积分 raw Twist，不夹紧负速度、不伪造 Action 结果、不模拟或启动 Guard/Mux。每个病例重新启动导航和理想模型，使用私有 `12427` master，并在启动前强制检查只有 loopback 网络。

```bash
./tools/noetic.sh python3 tools/check_teb_ros.py /workspace/.test-results/teb-odom-check
```

需要单独观察初始化时可重复 `--case`，例如 `--case rotate --case behind`；`--timeout 60` 是每个 Action 的软件观察时限。现提供六项病例：前进、当前位置 90° 转向、当前位置 180° 掉头（`turnaround`）、侧向目标、后方目标、有限障碍绕行。后方目标为 `(-0.5, 0, π)`，初始朝向 0；记录初始全局路径的前八个姿态，便于判断初始化是否促成先转向再前进。

每例记录真实参数、Action state、终点误差、raw 速度界限、差速 Twist、动作后的实际零输出、全部 raw 命令和下采样姿态。`negative_raw_count` 严格统计 `linear.x<0`，没有“忽略微负值”的 epsilon；`guard_forward_compatible` 和功能完成分开报告。只有两者均通过才将病例标为 `passed`。此工具检查的停止是软件输出停止，不是车辆制动验收。障碍场景记录雷达射线产生的真实 costmap 标记和导航结果，但理想模型没有碰撞响应，结果不能作为实车绕障验收。

首次工具运行在发送目标前发现 costmap 动态配置将 footprint 参数写回字符串、TEB 仍保留数组；测试工具直接比较类型误报。已仅在工具中解码字符串并验证三份几何相同，未修改 footprint 或生产参数；首次运行不计为 TEB 功能结果。

以下先保留适配前原有 `1.0/0.0` 基线。真实插件结果在 `.test-results/navigation-obstacles-20261026/teb-baseline-v2/`，五例均实际返回 `SUCCEEDED`，但完成过程中的 raw 负值严格计入兼容失败：

| 病例 | Action/输出检查 | 负 vx 条数 | 最小 raw vx（m/s） | 严格 Guard 兼容 |
| --- | --- | ---: | ---: | --- |
| 前进 `(0.7, 0, 0)` | 通过 | 43 | -0.000166165 | 未通过 |
| 当前位置旋转 90° | 通过 | 0 | 0 | 本次病例通过 |
| 侧向 `(0, 0.5, π/2)` | 通过 | 40 | -0.016140348 | 未通过 |
| 后方 `(-0.5, 0, π)` | 通过 | 100 | -0.008879585 | 未通过 |
| 有限障碍绕行 | 通过 | 15 | -0.011512173 | 未通过 |

这里的 Action/输出检查仅指真实动作终态、终点误差、Twist 上限与横移零、停止输出；没有把障碍几何间距列入该字段。绕障动作即使完成，也不能据此宣称轨迹对整车 footprint 的碰撞检查通过。

上述前进和转向结果已经反证“现有标准 TEB 配置可保证 raw 不含倒车命令”。后方目标能在允许 raw 负值积分的模型中转到目标，不等于经过严格 Guard 后还能完成同一动作。原地旋转病例仅可保留为后续现场候选，尚无实车验证；二维跟踪和绕障仍为实验对照。没有增加转换层或把负值截成零，也没有为了让有限病例变绿而调高 forward 权重。

`teb-baseline-v2/trajectory-metrics.json` 由保存的 raw/pose 数据离线重放得到：后方目标在车头首次累计转过 90° 时已经净平移 `0.0341 m`、走过 `0.0367 m`；整个动作 raw 反向速度的时间积分约 `0.0527 m`。它不是“先完全原地掉头，再只向前走”。原地旋转病例路径长度与最大平移均为零，最终约转过 `1.35 rad`，在既有 `0.25 rad` yaw 容差内结束。前进病例虽然到点，但总路径约 `1.030 m`、累计绝对转角约 `6.108 rad`，不能把它描述成平顺直线跟踪。

v2 工具在每个已 `SUCCEEDED` 的 Action 后仍多发一次 cancel，导致客户端记录 `PREEMPTING when DONE`。现已改为仅取消未结束目标，清理时停止追踪；没有修改生产节点。v2 保留原始失败证据，修正工具后的复测另行记录，不能把工具状态错误归因于 TEB 求解失败。

修正后的 TEB 复测在 `.test-results/navigation-obstacles-20261026/teb-final/`，客户端日志没有 ROS `ERROR`：前进约 `24.67 s` 完成，但仍有 `35` 条负 vx，最小 `-0.000275218 m/s`；原地旋转约 `5.60 s` 完成且 `0` 条负 vx；后方目标约 `13.50 s` 完成，但有 `100` 条负 vx，最小 `-0.009240044 m/s`。前进总路径 `1.031 m`、累计绝对转角 `6.080 rad`；后方目标在首次累计转过 90° 时净平移 `0.0350 m`，全程 raw 反向积分 `0.0531 m`。消除工具多余取消后，严格兼容失败的结论不变。

修正后的同场景 DWA 对照在 `.test-results/navigation-obstacles-20261026/dwa-comparison/`：前进约 `5.00 s`、原地旋转约 `5.60 s` 返回 `SUCCEEDED`，两者均没有负 vx。后方目标虽然也没有负 vx，却在约 `20.07 s` 因振荡返回 `ABORTED`，位置误差约 `0.525 m`、yaw 误差约 `3.074 rad`；不能宣称 DWA 已解决后方目标。保留 DWA 默认不等于所有二维路径或后方目标已验收。

### 最终交付：队友路径偏好适配版

`teb-slicer-adapted/` 是最终 `slicer_planner.yaml` 的真实插件检查，六份 `parameters.json` 均确认 `weight_optimaltime=1.5`、`weight_shortest_path=0.2`，速度/footprint 仍相同。没有把适配前的成功沿用给新配置：

| 病例 | 实际 Action | 负 vx 条数 | 最小 raw vx（m/s） | 严格兼容 |
| --- | --- | ---: | ---: | --- |
| 前进 | SUCCEEDED，24.70s | 125 | -0.000028655 | 未通过 |
| 原地90° | SUCCEEDED，5.49s | 0 | 0 | 本次通过 |
| 原地180° | SUCCEEDED，11.80s | 0 | 0 | 本次通过 |
| 侧向目标 | SUCCEEDED，7.50s | 39 | -0.020000 | 未通过 |
| 后方目标 | SUCCEEDED，13.30s | 98 | -0.010937 | 未通过 |
| 有限障碍场景 | SUCCEEDED，18.06s | 16 | -0.020000 | 未通过 |

六例速度上限/横移零/动作后零输出检查均通过，严格兼容仅2/6通过，整项工具如实以退出码1结束，日志无 ROS ERROR。新权重没有解决禁止负 vx 的具体矛盾，也没有证明优于 DWA；保持显式实验选项和 DWA 默认，不加过滤层或放宽 Guard。用户要求的“TEB不生成Guard禁止指令”尚未满足，不能作为生产替代验收。

## 尚未验证

为核对现场操作单的确切命令，另在 `teb-turnaround/`、`dwa-turnaround/` 检查当前位置 `yaw=π` 目标：两者均约11.90秒返回 `SUCCEEDED`、无负 vx、无平移；软件实际转角分别约167.5°、166.5°，均在现有14.3°容差内提前结束。这只支持有限原地掉头的软件入口，不解决位置在后方的完整路径跟踪。工具的 `--planner dwa` 使用相同理想模型和目标，不以不同实验条件比较快慢。

本次没有实车转向、摩擦/滑移、编码器/IMU/EKF 融合、底盘起转、Guard/Mux 全链动作完成、实际停车距离或 Jetson 算力验证。即使全部有限软件病例通过，也不能从有限采样推出 TEB 永远不产生负 vx。若出现负值或后方目标停滞，应保留失败证据与 DWA 默认，不通过改 Guard、反向夹紧、修改轮距或增加速度上限绕过。
