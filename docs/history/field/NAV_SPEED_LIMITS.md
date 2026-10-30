# 导航速度边界与下一次采集

本轮只做本地离线准备，不调整速度、加速度、PI、footprint 或避障参数。当前 `/ugv` 运行快照选择 DWA，规划器和 Guard 的前进上限都是 **0.125m/s**，角速上限都是 **0.25rad/s**。这些是当前软件运行边界，不是实测地面最高速度，也不能用队友工程的 0.8m/s 参数替代。

## 各层实际含义

| 层 | 当前设置或已采证据 | 约束含义 |
| --- | --- | --- |
| DWA | `max_vel_x=max_vel_trans=0.125`，`min_vel_x=0`；`max_vel_theta=0.25`；`acc_lim_x=acc_lim_trans=0.2m/s²`，`acc_lim_theta=1rad/s²` | 软件候选速度边界；采样、障碍代价、路径、目标末段和当前 odom 会使输出更低。`min_vel_trans=0.01`、`min_vel_theta=0.05` 不是能克服机械阻力的保证。 |
| TEB 比较配置 | 本地 `teb.yaml` 为 `max_vel_x=0.125`、`max_vel_theta=0.25`，加速度 0.2/1.0；后退参数为 0.02 | 只有实际选择 TEB 才生效。当前快照中的 TEB 子树只有 footprint，不能当成正在运行的 TEB。TEB 后退候选仍受 Guard 的整条负 vx 拒绝。 |
| Slicer Guard | 当前运行日志明确为 `forward=0.125 reverse=0.000 angular=0.250`；源 `navigation` 与所选 planner 的上限、加速度逐项取较小值 | 出口执行确定性的限幅/圆弧限加速度；负 vx 整条指令归零。LIO、TF、costmap 过期或碰撞包络失败也会归零。当前 `turn_compensation_enabled=false`，不存在起转抬升。 |
| Mux | 20Hz，输入保鲜 0.25s，startup/manual hold/急停及 mission/nav/teleop 来源选择 | 不进行数值速度缩放。未释放时仅允许新鲜 teleop，正常释放后 mission 优先于 nav；保持或陈旧输入可让最终 `/cmd_vel` 为零。 |
| body driver | `control_mode=body_speed`，把最终 Twist 转为 mm/s、mrad/s，经 nonce + 两个 i16 的 **0x16** 发送；20Hz，输入超时 0.25s，body 反馈超时 0.3s，IMU 年龄上限 100ms | SI→整数转换、状态/时效检查；编码范围不是车辆能力。`max_wheel_mm_s=600` 和 `ros_driver_pwm_limit_permille=600` 在 body 模式是兼容声明，不通过 0x16 下发主机轮速/PWM上限。 |
| MCU body 参考源码 | `max_body_mm_s=800`、`max_yaw_mrad_s=3500`、`max_wheel_mm_s=800`；闭环 PWM 600‰；watchdog 500ms | 解码超界拒绝、四轮目标统一比例限幅及 PWM 限幅属于该源码的确定性边界。0x16 不带这些配置；扩展/nonce ACK 不证明已烧录二进制身份或实物输出。不能将源码中的 0.8m/s、3.5rad/s 称为当前已验收硬件上限。 |

DWA 的 `scaling_speed=0.25` 用于速度相关的 footprint 缩放，不是另一个前进速度上限。软件上限与硬件可达速度须分开：即使降低软件上限，低速转向也可能欠响应；即使提高软件上限，也不能修复 hold、障碍、反馈拖尾或机械干涉。

依据：本地 `src/ugv_bringup/config/{dwa,teb,slicer_guard,slicer_driver_body_speed}.yaml`，`navigation_core.py::planner_limits`、`arc_velocity_guard.py`、`slicer_manual.py::ManualMux`、`c30d_driver_node.py`、`c30d_link.py::body_speed`。当前现场证据是 `deliverables/slicer_odom_review_20261026/car_snapshot/runtime-ros-snapshot.json` 以及其 `current_run_logs/be88c1f0-d097-11f1-b97a-3c6d661ed63d/slicer_nav_arc_guard-2.log`；MCU 参考为同一已采资料的 `external_source/Slicer_Workspace/firmware/c30d_body_closed_loop/`，仅只读核对。

## 已有记录能区分什么

`.test-results/body-nav-20261025/body-response-comparison.json` 中，两次现场发送全部为十进制 kind=22，即 0x16；body/发送最大接收间隔小于 67ms，IMU flags=7、最大年龄4ms、无 fault。这两段记录不支持“串口断流导致全程很慢”。

01 的纯左转累计约12.8秒：MCU接受目标角速中位 +0.074rad/s，实测中位 +0.002；纯右转约12.04秒：目标 -0.089，实测 -0.005。这些目标低于0.25上限，仍有显著下游欠响应。02 直行32帧的目标前速中位0.125m/s、编码器均速中位0.1205，较快左弧372帧为0.125/0.124；这说明某些工况确实到达当前软件前速上限，不能据此泛化到低速原地转向。编码器均速不是尺量地面速度，阶段中位数也不代替连续跟随误差分析。

01 末段另有上游等待：原始 DWA 指令已为零，融合 odom 角速拖尾到 `theta_stopped_vel=0.01` 附近才恢复转向。已采分析见 [两次导航结果](BODY_NAV_RESULTS_20261025.md)。该页也记录后续机械干涉清除和单次复核改善；不能把清除前欠响应当成当前故障已经重现，也不能从一次转向成功宣称二维低速工况全部修复。本轮不重复基础底盘运动试验。

下一包按同一 bag 接收时间对齐以下链路；先看持续窗口，再看峰值，保留动作结果和至少5秒停止尾段：

| 观测关系 | 可定位的层 |
| --- | --- |
| global/local plan、costmap/footprint 与 `nav_raw` 本身持续偏低或归零 | 规划、输入数据和目标末段；结合 `/rosout_agg` 判断计划失败/振荡，不能只看到低速就认定限幅。 |
| `nav_raw` 非零但 `/cmd_vel/nav` 更小或归零 | Guard；结合 `hold_reason`、`adjustment`、LIO/scan/TF/costmap 时效。正常限加速度也会造成短时差异。 |
| `/cmd_vel/nav` 非零而 `/cmd_vel` 为零或来自别的输入 | Mux；核对 `manual_state` 的 hold、selected_source 与 teleop/mission 输入。 |
| `/cmd_vel` 与 driver command/serial_tx 不一致 | driver 的 stop_reason、超时、ready、反馈/IMU健康及实际控制模式。 |
| driver 已发、MCU target 已接受，0x93 实测 vx/wz 仍持续跟不上 | 下游跟随；按 MCU 自己的推进时钟核对轮反馈/IMU/故障，不把 ACK 或软件 duty 当作执行证明。 |

原有两段默认包没有 plans/costmaps，不能离线补造规划画面。`--navigation` 为下一次完整链路记录增加两层地图和 updates/footprint、GlobalPlanner、TEB/DWA global/local plans、current/simple goal、Guard adjustment、rosout。未运行的 planner 话题为空是正常情况。

## 现场只读采集命令

以下命令交给用户在已配置好 ROS 与本工程的采集终端执行；采集器不启动 driver、不发速度/目标、不 release 或 hold。`--duration` 从采集器开始计时，准备/等待时间也会消耗窗口；看到 `CAPTURE COMPLETE` 后该包已停止，后续操作不会补录进去。现在开始、结束均有明确提示。输出目录必须是新的，不能重用。

静态感知20秒，包含原始 Livox、IMU、机体/注册/过滤点云及各级 scan；点云数据较大，因此仅显式选择 `--perception` 时启用，不录累计 `/Laser_map`：

```bash
python3 tools/collect_odom_test.py --namespace ugv --navigation --perception --duration 20 --output evidence/perception-static-$(date +%Y%m%d-%H%M%S)
```

导航/障碍复核沿用小包，发目标前执行，等 `RECORDING STARTED`；600秒是自动结束上限，现场动作和停止尾段完成后可 Ctrl-C 收包：

```bash
python3 tools/collect_odom_test.py --namespace ugv --navigation --duration 600 --output evidence/nav-obstacle-$(date +%Y%m%d-%H%M%S)
```

默认已有 `/odom`、`/lio/odom`、`/wheel/odom`、`/Odometry`、TF、scan、三段导航速度、teleop/mission、Action goal/status/result/feedback、Mux状态和 driver diagnostics（含0x90/0x93、发送与 stop_reason）。`--perception` 另加 `/livox/lidar`、`/livox/imu`、`/cloud_registered`、`/cloud_registered_body`、`/cloud_nav_filtered`、`/scan_raw`、`/scan_front` 和默认旁路 `/ugv/perception_test/scan`。若实际话题改名，按 manifest 核对，并用显式 `--extra-topic /实际话题` 补充；不使用全话题录制，也不订阅 `/cup_nav/manual_intent`。

`--perception` 还在开始、结束两次快照中读取这些明确参数路径：`/slicer_cloud_to_scan`、`/slicer_arm_cloud_filter`、`/slicer_scan_filter`、`/slicer_geometry`、`/ugv/perception_test/cloud_to_scan`；FASTLIO 的 `/preprocess`、`/publish`、`/filter_size_surf`、`/filter_size_map`、`/point_filter_num`、`/frame_id`、`/user_config_path`、`/common`、`/mapping`。只读取这些机器人参数并保留脱敏；缺失路径记录 unavailable，不遍历整个参数根，也不根据参数里的文件路径读取文件。旁路的 scan 和节点参数前缀随 `--namespace` 改变。

arm/body filter 与 geometry 只将配置文件路径放在私有 ROS 参数中，不把所有内部阈值发布出来；其掩码和安装值仍需附本次实际启动使用的配置相关节及 SHA256，不能声称仅 manifest 已采齐这些有效值。

交付新采集目录的 `motion.bag`、`manifest.json`、`rosbag.log` 和人工记录的障碍位置/尺寸、出现移走时刻、车体实际反应；未知量留空。检查 `bag_finalized=true` 与新目标/障碍确实在包内，不能把录制进程存在当作已经订阅确认。

本轮验证记录：`.test-results/nav-capture-20261026/`；纯 collector 测试先复现缺少两组选项/感知参数快照，最终16项通过；Noetic 容器内全部 tools 71项通过。增强后的 ROS 采集检查使用 `--navigation --perception`，自动结束、Ctrl-C 两例均通过，实际 PointCloud2/Path/OccupancyGrid 消息已写入并读回，开始/结束感知参数已记录且脱敏；bag 已收尾、订阅已确认且未新增速度发布或 manual_intent 订阅。容器使用本地独立 master、`--network none`、无设备，只证明软件采集行为，不代表现场验收。
