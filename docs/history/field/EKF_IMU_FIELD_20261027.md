# 2026-10-27 实车 EKF / IMU 联调结果

**结论：保留已部署的 Q(wz)=0.2，继续现有 DWA＋0x16 导航；不接入原始 MID360 gyro。** 本轮没有修改队友源码、MCU、RTK、TEB、GlobalPlanner 或 Guard/Mux 安全限制。

| 实车目标 | 实际结果 | 停车观察 |
|---|---|---|
| 相对左转30° | SUCCEEDED；EKF约19.92° | 用户确认 A轮连续、自停 |
| 相对左45°、右45° | 两次 SUCCEEDED；EKF +34.16° / −34.00° | 用户观察左约30–35°，两段后回到初始方向约差1–2°；自停 |
| 相对前进0.5m | SUCCEEDED；实测37cm，EKF增量0.3777m | A轮正常、自停、无干预 |
| 绝对 odom (1,0.5)，yaw=45° | SUCCEEDED，19.30s；odom目标残差0.0884m | 用户确认正常 |
| 绝对 odom (2.5,−3)，yaw=−45° | SUCCEEDED，45.10s；odom目标残差0.1371m | 用户确认到目标附近、自停、无手控干预 |

角度/位置容差仍为0.25rad/0.15m，未为凑结果调整。表中 odom 残差不是外部定位精度。30°与左45°之间，用户手动调回了初始方向；没有定位归零。原(1,1)因固定障碍周围高代价无法规划，未发 Action；后来由用户改成(1,0.5)。正y=3只做过预规划，实际远目标是用户最终确认的 **y=−3**；(3,4)没有执行。

**EKF 拖尾实车验证。** 使用本次干净的左右转同段 bag，在无设备、`--network none` 的真实 robot_localization 2.7.7 节点中回放三配置：

| 配置 | 左转：MCU静止→融合停止 | 右转：MCU静止→融合停止 |
|---|---:|---:|
| 默认 Q(wz)=0.02 | 5.188s | 5.095s |
| 已部署 Q(wz)=0.2 | **1.821s** | **1.792s** |
| Q=0.2＋未去偏置的原始 MID360 wz | 未达阈值 | 未达阈值 |

停止判据为 MCU gyro、MCU/轮速 vx 已进入0.01阈值后，融合 vx/wz 进入0.01并连续保持0.5s；不是经视频同步测得的精确物理刹停时间。Q=0.2回放与本次实录对应停止时间一致；直行、两个二维段的实际收敛分别为0.562/1.779/0.627s。LIO参考位姿增量没有明显变差；LIO是共同输入，不能作为独立真值。

**IMU 实际情况。** `/livox/imu` 约200Hz；时间戳持续推进，静止抽样平均接收延迟约0.9ms；frame为`livox_frame`。实际 driver2 源码直接复制原始 gyro，角速度为rad/s，左转Z正、右转Z负已用实车数据确认；该驱动加速度仍为g，本轮不融合加速度。原始 orientation 和各协方差均为零；线上没有`livox_frame`的TF链。

离线候选仅把消息**副本**的frame名称改为`body`，依据实际源码中“body为FAST-LIO IMU坐标”及同一已配置外参，使用bag中的真实静态TF；没有修改原bag、向量、时间戳或线上TF。gyro方差显式取本段静止对零的MSE=0.0003919866（1600个样本），这是敏感性试验输入，**不是标定好的噪声协方差**。未扣偏置、未融合orientation/加速度。

不同静止窗口原始Z均值约0.035/0.026/0.020/0.013rad/s，变化原因未确认。候选充分静止后的融合wz均值仍为0.01909/0.01287rad/s，无法连续满足停止阈值；观测窗口无缺测。LIO本身已使用该IMU，二者也不是独立信息。因此本轮不部署gyro融合，不用协方差掩盖偏置。

**遇人停车。** 用户在远目标后半程站到前面，走开后车自行续行到点。22:36:54.912 Guard先拦住仍非零的规划命令，随后DWA也输出零并短暂无有效局部路径；前方scan/局部致命格出现后清除，22:36:58.808自动恢复前进。同一Action于22:37:16.854成功，无取消、换目标或手控接管。证据支持“停车避让、自动续行”，**不等于绕行通过**。

**仍保留的问题。** A轮在部分低速转向阶段存在短暂无计数而软件duty非零，实物观察正常；本次未记录到“持续零计数且≥800 timer ticks”或MCU fault，不能宣称所有低速跟随问题已解决。软件duty也不是实际电机转矩。

**部署与最终状态。** 只向独立工程安装已验证的EKF参数/定位入口，ARM64构建通过，13个受保护的driver/Guard/Mux/DWA文件哈希未变。启动时发现用户参数误输入`publish_tf=false~`，纠正为boolean false并重启后，所有运动段`odom→base_link`只有EKF发布。ROS最终使用电台`192.168.10.33:11311`；临时热点master已停用，原电台master恢复。最后确认Mux held、四轮速度和软件duty为零；随后SSH掉线，未完成掉线后的实时复核，也未再发命令。

**数据。** 5个原始bag均已拷回、索引可读且manifest确认finalized。group01前段有重复TF及driver重启，只使用其中修正后的30°运动窗口；其余4份采集无这些检查项。记录的是完整命令链、MCU原始反馈、LIO/wheel/融合odom、原始IMU、TF、Action、规划及Costmap，没有伪造测量。

- 原始bag：`.test-results/ekf-imu-field-20261027/motion.bag`（group01）及`evidence/ekf-imu-20261027-{group02,group03,nav1-half,nav2p5-minus3}/motion.bag`。
- 三配置停止曲线（原归档未随仓库收录：`../../.test-results/ekf-imu-field-20261027/group02-stop-response.png`）、逐转向回放结果（原归档未随仓库收录：`../../.test-results/ekf-imu-field-20261027/group02-comparison/actions.csv`）、其余实车段分析（原归档未随仓库收录：`../../.test-results/ekf-imu-field-20261027/remaining-trials-analysis.json`）、遇人响应证据（原归档未随仓库收录：`../../.test-results/ekf-imu-field-20261027/nav-person-response.json`）。
- 现场观察与授权记录（原归档未随仓库收录：`../../.test-results/ekf-imu-field-20261027/operator-observations.json`）、本地数据哈希（原归档未随仓库收录：`../../.test-results/ekf-imu-field-20261027/local-data-sha256.json`）。14个bag/元数据文件已在本地；group01的rosbag.log未单独取回，最后源端哈希回查因SSH掉线未完成，不标成已校验。
- 离线IMU回放扩展在`tools/replay_ekf_response.py`；7项新增测试与5项既有指标测试通过。合成工具测试与真实运动数据分开保存；没有上线新IMU融合。

**继续使用。** 当前定位入口为`roslaunch ugv_bringup slicer_sensors.launch root:=/slicer`；底盘仍用`control_mode:=body_speed publish_tf:=false`，导航仍是现有DWA/12m入口。回退时结束定位后以`tune_ekf:=false`重开同一入口，不要并行启动第二份EKF。

在车端操作终端进入现有ROS shell后，观察和停车命令仍为：

```bash
rosrun ugv_slicer_control control.py state
rosrun ugv_slicer_control control.py hold
```

下一次运动仍先确认现场人员、路线和接管条件；本轮没有新增自动任务或恢复机制。
