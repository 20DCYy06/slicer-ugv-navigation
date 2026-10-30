# EKF 速度响应：本地回放结果与上车验证（2026-10-27）

只保留 **Q(wz)=0.02→0.2** 这一处估计参数变化。没有新增 IMU 输入，未改 DWA 阈值、Guard、MCU 或队友源码。本文是离线准备记录；随后已完成实车部署与联调，见[当日现场结果](EKF_IMU_FIELD_20261027.md)。

**复现与输入。** 使用 `nav-dwa-20261025-201924/motion.bag` 四次成功试验，真实 `robot_localization 2.7.7` 节点、独立 master、无设备且 `--network none`。仅回放原始 `/lio/odom`、`/wheel/odom` 和模拟时钟，保留消息时间戳、测量协方差和采集时接收顺序。

- before/after EKF 参数一致：LIO x/y/yaw，轮速仅 vx；没有设置过程噪声矩阵，实际使用库默认值（已直接查询链接库，并用显式默认矩阵回放得到相同结果）。
- LIO 实测 10Hz，yaw 方差 0.0025；轮速 vx 方差 **0.1**，轮速和融合输出约 17.91Hz。配置 30Hz 不等于实际输出 30Hz。
- LIO 接收减时间戳中位 54ms、P95 61ms；轮速约 0.84/2.05ms。没有时间回退。这个量级不能解释数秒拖尾。
- wz 由 yaw 位置观测间接估计；默认 yaw 过程噪声 0.06、wz 为 0.02。增大后者允许角速度状态更快跟随 yaw 变化；测量可信度未被人为改写。

**停止对照。** 起点为末次最终 `/cmd_vel` 归零后，MCU gyro、MCU vx、轮速 vx 均进入绝对值 0.01 阈值并持续 0.5s 的首个样本。下表是从该起点到融合 vx/wz 同时进入相同阈值、持续 0.5s 的时间；不是物理刹停距离。

| 成功试验 | 实录与默认回放 | 采用 Q(wz)=0.2 | 仅测试 Q(wz)=2.0 |
|---|---:|---:|---:|
| 原地 π | 5.25s | 1.56s | 0.45s |
| 后方目标 | 4.28s | 1.69s | 0.51s |
| 二维目标 | 2.69s | 已在阈值内¹ | 已在阈值内¹ |
| 简单障碍目标 | 4.47s | 1.63s | 0.56s |

¹ 下一条融合样本已满足条件，不能解释成精确 2ms 的物理响应。各组 vx 都在下一样本进入阈值，主要改善来自 wz。2倍/4倍回放的上述停止时间全部一致；异步节点在少数运动中间样本并非逐点一致，详见重复性记录。

**位姿代价。** 公平使用四次成功后5–12秒、反馈确认完全静止的相同窗口：去趋势航向抖动合并 RMS 从 0.000354 增至 0.000391rad（约 0.0203°→0.0224°，+10.5%）；充分静止后的 wz 噪声 RMS **从 0.000269 增至 0.000594rad/s**。这是实际噪声代价，不能把拖尾变短说成噪声降低；绝对量仍远小于0.01停止阈值。运动中最大相邻航向变化总体约 0.0300→0.0285rad，没有新增 wrap 跳变。LIO 参考下的相对位移增量 RMS 约 2.0–3.4mm，变化小于 0.004mm；航向增量 RMS 从 0.0061–0.0075 降至 0.0044–0.0053rad。**LIO 是共同输入，不是独立定位真值；不能据此宣称真实定位误差改善。固定输入回放也不等于 DWA 闭环实车验收。**

另测 yaw 过程噪声降至 0.006/0.0006，虽缩短拖尾，却增加 LIO 参考下的航向增量误差，因此不采用。Q(wz)=2.0 更快，但静止角速度及航向抖动更大；先保留 0.2 作为保守上车候选，不声称它是最优值。

**交付与证据。**

- 参数：`src/ugv_bringup/config/ekf_velocity_response.yaml`；225项中仅索引176不同于真实库默认值。
- 定位入口：`roslaunch ugv_bringup slicer_sensors.launch`，包含原有 `slicer_bringup/sensors.launch`，只额外加载上述 EKF 参数；不启动底盘或导航。
- 重放/比较：`tools/replay_ekf_response.py`、`tools/analyze_ekf_response.py`；指标单测覆盖停稳持续时间、缺帧、航向跨π和坐标原点变化。
- 原始结果：`.test-results/ekf-response-20261027/` 下的 `input-facts.json`、`comparison/actions.csv`、`comparison/summary.json`、`launch-validation.json`、`parameter-server-validation.json`、`repeat-validation.json` 及构建/回放日志。实际参数服务器已验证调优→回退→调优→回退，不会继承旧Q。
- 四次停止响应曲线（原归档未随仓库收录：`../../.test-results/ekf-response-20261027/stop-response.png`）。
- 重现命令（仅本地隔离环境，输出目录须为新目录）：

```bash
./tools/noetic.sh python3 tools/replay_ekf_response.py evidence/nav-dwa-20261025-201924/motion.bag .test-results/ekf-check-new --profiles .test-results/ekf-response-20261027/profiles.json
./tools/noetic.sh python3 tools/analyze_ekf_response.py evidence/nav-dwa-20261025-201924/motion.bag .test-results/ekf-check-new .test-results/ekf-check-analysis
python3 tools/test_analyze_ekf_response.py
```

**上车最小验证（下次安装本轮文件后，由现场执行）。**

1. 保持控制 held、无活动目标，结束旧定位 launch；在原 ROS shell 用下面入口替代旧定位启动命令。不能同时运行两份 EKF，重启定位后 odom 原点会重置。

```bash
roslaunch ugv_bringup slicer_sensors.launch root:=/slicer
```

2. 在操作终端确认新进程参数，期望 `0.2`；另开采集终端，复用现有采集器。导航继续使用原 DWA，底盘继续 body_speed，其他参数不变。

```bash
python3 -c 'import rospy; print(rospy.get_param("/slicer_ekf/process_noise_covariance")[176])'
python3 tools/collect_odom_test.py --navigation --duration 180 --output "evidence/ekf-response-$(date +%Y%m%d-%H%M%S)"
```

3. 在空旷场地做一次原有 45°纯转和一次 0.5m 直行；沿用现有 release/send/hold 接口。记录实测角度、位移、停车余程，并观察停止后至少5秒。检查 MCU 已停后融合 wz 是否更快降到0.01、位置和航向是否跳动；若出现明显跳变、漂移或故障，停止本轮并回退。

回退：结束定位入口后，重开 `roslaunch ugv_bringup slicer_sensors.launch root:=/slicer tune_ekf:=false`，沿用队友生成的原 EKF 配置。勿只在线 rosparam 修改后以为滤波器已重新读取。

**TEB 次任务结论。** 标准 0.9.1 的 forward-drive 是有限软惩罚，初始化开关不能提供硬约束；`max_vel_x_backwards<=0` 的分支仅报警，动态参数最小值实际为0.01。现有真实 TEB 六场景中四场景仍有负 raw，现场原地目标也有直接反例。因此本轮不提交“严格前进”TEB参数，不改 Guard、不裁剪输出；保留 DWA。源码依据：[运动学惩罚](https://github.com/rst-tu-dortmund/teb_local_planner/blob/0.9.1/include/teb_local_planner/g2o_types/edge_kinematics.h)、[速度提取及限幅](https://github.com/rst-tu-dortmund/teb_local_planner/blob/0.9.1/src/teb_local_planner_ros.cpp)。
