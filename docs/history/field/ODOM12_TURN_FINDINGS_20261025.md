# 12m odom导航：本次转向失败的证据与结论

分析日期：2026-10-25。仅取回并离线分析用户指定的 `evidence/odom12-20261025-133308/`，另保存本次进程对应的driver/Guard日志；未修改导航、控制或固件，未发送控制指令。终端统一称定位、导航、底盘、操作、采集终端。

原始bag SHA256：`cfa2495995d295df6b443b3f4b82ff0909a555b1d59033b4e6368df534947368`，WSL与车端一致。bag时长205.65秒，覆盖后5次目标。用户首次未录制的 `(1,1), yaw=0` 不在包内，不能据本包比较它与开启观察前后的因果关系。

## 已证实

| 目标 | 结果/活动时间 | 全局规划 | LIO起止位移/转角 |
|---|---|---|---|
| (1,1), yaw=0.7854 | ABORTED oscillation，20.20s | 21条非空路径；目标格0 | 约0.41cm / −0.029° |
| (2,2), yaw=0.7854 | 无有效路径，5.10s | 51条空路径；目标格99 | 约0.14cm / −0.009° |
| (2,−1), yaw=0.7854 | ABORTED oscillation，20.20s | 21条非空路径 | 约0.14cm / +0.011° |
| (2,1), yaw=0.7854 | ABORTED oscillation，20.20s | 21条非空路径；目标格0 | 约2.96cm / +0.249° |
| (4,1), yaw=0.7854 | ABORTED oscillation，20.20s | 21条非空路径；目标格0 | 约0.64cm / +0.038° |

这里的位移/角度来自LIO自身起止姿态增量，不是尺量或真实物理角度标定。小变化可能包含定位噪声。两张地图实际为240×240和80×80、分辨率0.05m；12m配置已经生效。`(2,2)`格99对应障碍膨胀的不可规划代价，不能通过换终点yaw解决。

速度链与底盘证据：

- 四段oscillation期间Mux均已released、选择nav，无急停锁存；Guard未报告负vx、足迹、LIO/TF等拦截。raw/nav/cmd的8802条样本中vx均非负。
- `/cmd_vel`角速度绝对峰值约0.102rad/s，而非配置上限0.25。四段角速度有36/20/14/33次变号。Guard有正常限加速度处理，Mux未把这些速度整体归零。
- driver运行源码哈希与本地匹配，实际模式为wheel_speed；3752条成功串口发送全部为十进制kind=16，即 **0x10**。常见纯转目标为ABCD `[7,-7,7,-7]`、`[4,-4,4,-4]` mm/s，未出现纯转±19mm/s。
- 5152条MCU遥测均无fault位，configured/enabled/powered位持续成立；MCU tick持续每帧推进40000µs。没有本次故障锁存、配置丢失或命令超时证据。
- `(2,1)`中有3.941s连续同向纯转、72条指令刷新；99条匹配ACK的反馈中97条四轮速度全零，LIO转角约0.014°。这排除了“所有不转都只是极短脉冲”的解释。
- 指令反复变号还触发driver已有的换向插零（全包148帧）。但连续同向数秒的段也未起转，故不能把问题仅归因于插零。未删除或放宽该逻辑。
- 遥测duty是软件timer ticks，不是实测电桥输出、电流或转矩；ACK只证明接受命令。现有数据无法进一步单独判定摩擦、PI/前馈或功率级哪个是底层原因。

## 为何上限0.25，实际只有约0.10

实际DWA配置为 `controller_frequency=10Hz`、`acc_lim_theta=1rad/s²`、`use_dwa=true`。标准1.17.3实现按反馈速度及一个控制周期建立动态窗口：

`最大候选角速度 = min(max_vel_theta, 反馈角速度 + acc_lim_theta × sim_period)`

近静止时是 `min(0.25, 0 + 1×0.1) ≈ 0.10rad/s`。按当前0.15m轮距，纯转对应轮速约±7.5mm/s，与串口实际±7/8一致。这是标准算法机制，不能把0.25参数当作每次实际命令。

底盘未对低指令产生足够运动，反馈角速度继续接近零，DWA候选范围便仍停留在低速附近。这一闭环衔接与本包相符；底盘最低稳定起转能力及改变何种参数才合适仍须受控验证，不能由此猜数值。

实现依据：[SimpleTrajectoryGenerator 1.17.3](https://raw.githubusercontent.com/ros-planning/navigation/1.17.3/base_local_planner/src/simple_trajectory_generator.cpp)（动态窗口），[DWAPlanner 1.17.3](https://raw.githubusercontent.com/ros-planning/navigation/1.17.3/dwa_local_planner/src/dwa_planner.cpp)（从controller_frequency计算sim_period）。

## 队友IMU闭环与下一步

既有[IMU交接](HANDOFF_TO_MAIN_2026-10-25.md)已确认队友车体IMU角速度外环的入口是 **0x16**，旧0x10轮速入口仍保留。本包实际只走0x10，不能把队友IMU外环算成本次已启用或已验收。12m部署只改全局配置及辅助资源，没有接入body_speed。

下一步应围绕这一接口差异与低速跟随处理：先决定继续验证0x10低轮速能力，还是复用已有0x16车体闭环；后一项需要对接现有driver的协议/反馈路径并做离线验证，不能仅改配置名。两者均继续使用现有Guard→Mux→唯一driver→Watchdog，不增加导航框架。

现阶段保留hold，不再用不同坐标重复相同失败，不通过延长oscillation_timeout、清图、放宽Guard或猜PWM来掩盖该现象。本次没有执行任何控制链修复或部署。

## 分析产物

工作区 `.test-results/odom12-incident-20261025/`：

- `map_plan.json` / `map_plan.py`：每目标地图格值、起点及全局路径数量。
- `velocity-chain/report.json`、`velocity_samples.csv`、`events.json`：三段速度、Mux/Guard与目标状态。
- `mcu-response/mcu_response.json`、`brief.json`、`extracted.json`：串口目标、匹配ACK反馈、持续同向段与姿态变化。
- `slicer_c30d_driver-1.log`、`slicer_nav_arc_guard-2.log`：本次运行日志。按roslaunch声明尝试获取move_base日志时对应文件不存在，未将其列作已取回；规划/结果证据来自bag。

原始证据位于 `evidence/odom12-20261025-133308/`。分析时间使用bag接收时间，未将不同主机时间戳相减当作物理延迟。
