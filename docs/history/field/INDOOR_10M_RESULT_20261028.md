# 室内10m首轮：0.20m/s，尚未到点

## 已完成的软件准备

- 已部署并启用 `indoor_10m_020.yaml`：滚动全局图24×16m、0.05m；保持 `GlobalPlanner.allow_unknown=true`。实测480×320格，局部图仍80×80格。
- DWA `max_vel_x` 与 `max_vel_trans` 均0.20m/s。Guard随导航一起重启，启动时读取该上限；角速0.25rad/s、加速度0.2/1.0、停止阈值0.01/0.01及所有原安全规则不变。
- 保留已验证yaw=0.12rad、xy=0.10m，EKF Q(wz)=0.2、0x16与原安全链。两份新配置已ARM64安装，8份受保护原文件未变。
- `indoor_10m_030.yaml`仅作后备，**没有启用或运动验收**。原12m与0.125m/s配置保留。
- 已有goal.py保持不变；新增只读预览工具冻结一次新鲜TF，核对实际地图余量并调用GetPlan，打印同一绝对odom目标命令。几何/窗口/冻结测试5项通过。

## 起点调整后的定位恢复

用户抬起搬动小车后，FAST-LIO持续报 `No Effective Points`，处理数分钟前旧数据，位置严重发散；`/lio/odom`、scan不再更新。源雷达仍约10Hz且时间戳新鲜。没有执行当时生成的preview01目标。

保持held并在用户确认放稳、无遮挡后，只重启原定位链，未改算法或参数。新LIO/scan恢复约10Hz，静止odom回到新原点附近，Q(wz)=0.2不变。旧目标明确作废，使用preview02重新冻结：`odom x=10.026788345, y=-0.028003174, yaw=-0.003776801`。此目标距当前全局窗口最近边界约1.97m。

## 静态地图证据

25秒静止感知bag已完整取回WSL：`evidence/indoor10m-static-20261028-01`。LIO/scan约10Hz，最大接收间隔约0.181/0.178秒；global/local实际发布约2.43/2.51Hz，最大间隔约0.565/0.472秒。`odom→base_link`仅观察到EKF发布者，无重复/倒退时间戳。

全局图约87.14%为未知；preview02的404点路径中，241点落在最后静止地图的未知区，163点为空闲。约3m内454个scan返回均邻近障碍占用格，支持近场返回已入图；不能把点簇命名成桌腿/横梁，也不能据此确认10m全程净空。用户回复“ok”确认现场检查，未提供通道净宽或最低横梁高度的测量值。

路线预览图（原归档未随仓库收录：`../../.test-results/indoor10-20261028/preview-route.png`） 是最后静止地图与约6.5秒后冻结路径的离线叠图，非同时刻真值。原始静态分析见 JSON（原归档未随仓库收录：`../../.test-results/indoor10-20261028/static-environment-analysis.json`）。

## 本轮运动结果（恢复副本已核对）

用户明确确认全程在旁、路线/终点安全、急停与人工接管可用后，发送一次上述固定目标，0.20m/s上限、180秒超时，无自动重发。

- **没有到点。** 终端实际返回 `state=4 Robot is oscillating. Even after executing recovery behaviors.`，随后hold确认 `manual_hold=true, selected_source=zero`；用户确认车已停止并换电。
- Action持续141.201秒；终点odom约 `(10.54383,-1.02615)`，相对起点前进10.518m、右偏0.992m，距冻结目标1.124m。逐点曲线长15.341m（含抖动与小往复，不是地面实测路程），左右绕行偏移约±1.81m。用户补充：目标落在实际不可通行门槛附近，选择不合适，前段导航表现尚可。因此本轮不能认定完整端到端到点通过。
- 前20秒软件前进约3.522m；20～50秒仅再前进0.535m，包含规划失败、转向和约9.012秒Guard占用停止。最后20秒净位移仅1.12cm，随后触发oscillation abort。并非全程都慢。
- 最终命令分段：平移95.584秒、纯转27.449秒、近零18.169秒（阈值为vx 0.005m/s、wz 0.01rad/s，描述请求而非真实运动）。Guard占用13.806秒，与上述时间有重叠，不能相加。
- **平均巡航速度**：最终vx>0.02m/s的82.984秒内，MCU轮速推导body vx均值0.16154m/s，odom增量投影车头方向均值0.16245m/s；最终vx≥0.19的44.213秒内，两者分别0.20872、0.20991m/s。均为软件/编码器观测，无独立地面测速。
- MCU纯转目标约28.494秒，其中gyro Z接近零累计21.961秒；多段低角速滞转期间DWA raw与Guard仍持续非零。该证据与上一轮卡顿一致，但不能直接区分内部控制与机械原因。
- Action期间3534条body ACK均匹配已发送目标、无fault/IMU失效；2762条Mux状态均nav。DWA有29次路径相关失败记录，约第91.925秒有一次GlobalPlanner `NO PATH`/提取路径异常；全局/局部规划并非全程断更。
- 终端后约2.088秒观测到hold。+5～+10秒有126条body反馈：轮速四路零、软件duty四零，body vx为零，gyro绝对值不超过0.001rad/s，与用户确认停车相符。

换电时采集器未正常封口。原始 `.bag.active`、manifest与log已完整取回本地 `evidence/indoor10m-20261028-01` 并保持不变；仅在WSL副本执行rosbag reindex，恢复文件位于 `.test-results/indoor10-20261028/recovered/motion.bag`。可读覆盖326.314秒、约9.9万条消息，包含终端及其后68.475秒；不能据此声称没有任何丢包。

本轮实际轨迹与首末规划（原归档未随仓库收录：`../../.test-results/indoor10-20261028/navigation01-trajectory.png`） 使用abort前Costmap致命格作背景，不表示全程环境的同时刻真值；末端规划线连接到目标，也不能证明低门槛可通行。

瞬时ActionGoal未录到；由accepted current_goal、唯一新ACTIVE ID和同ID ABORTED结果关联目标，且与preview02一致，未虚构输入包。manifest仅有before，after及原bag_finalized仍未知。参数依据before与现场回读，未伪造after。恢复方法与哈希见 恢复来源（原归档未随仓库收录：`../../.test-results/indoor10-20261028/recovered/recovery-provenance.json`）；运动分析见 JSON（原归档未随仓库收录：`../../.test-results/indoor10-20261028/navigation01-analysis.json`）、简报（原归档未随仓库收录：`../../.test-results/indoor10-20261028/brief.txt`）。换电后仅取回数据，没有自动恢复运动或启用0.30。

## 门槛、低障碍与脱困的后续判断

当前DWA `min_vel_x=0`，Guard禁止负vx；`recovery_behavior_enabled=false`、`clearing_rotation_allowed=false`且恢复列表为空。因此没有自动后退或解困旋转；日志“Even after executing recovery behaviors”是通用失败文案，不能当成确实执行过这些动作。普通DWA原地转向仍需满足扫掠足迹无碰撞，低角速反馈不足也仍是独立问题。对于用户确认不可跨越的门槛，应标记不可通行并重规划；无安全路径则停车报告失败，不能靠强制转动越过。

`/scan`的angle_min/max改变水平投影范围，不能把硬件光束向下转。当前高度带为 `base_link z=0.04～0.50m`；低于该坐标下界的点会被丢弃，不能直接把它等同离地4cm。须在门槛处静止核对原始点云→过滤点云→scan→Costmap；本次原始点云只录于起点，不能证明终点门槛在哪一级消失。

用户随后确认门槛约12cm高，车辆已经挪离现场。该高度应明确视为此底盘不可通行障碍；不能把漏检简单归于“低于4cm过滤阈值”，也不能在别处采样声称已验证该门槛。下次最小验证为约12cm固定物、0.5/1/2m三个静止距离，同时看原始点云、过滤点云、scan与Costmap；另实测雷达光学中心距地高度，才有依据计算安装俯角。当前没有进行这组验证。

MID360水平安装时向下覆盖约7°；物理下倾可能改善前方低障碍覆盖，但要核对后方覆盖并更新实际车体外参/投影，重新验证LIO。单纯改软件角度不会创造原始点云中没有的点。依据：[Livox安装与视场说明](https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf)、[pointcloud_to_laserscan参数说明](https://index.ros.org/r/pointcloud_to_laserscan/)、[Noetic move_base恢复开关](https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/move_base/cfg/MoveBase.cfg)。本轮未改变安装角度、高度过滤或恢复规则。

## 后续与回退

本轮未修改MCU、Guard保护、评分或停止阈值。0.20整段尚未跑通，0.30对照暂缓。应先用本轮bag区分低角速跟随不足、局部占用/未知格停止与全局路径可达性，再决定下一次最小对照。

回退：停车held、无活动目标，导航/Guard一起以原 `rolling_global_12m.yaml` 重启；需保留已验证容差时再载入现有yaw/xy补丁。具体现场命令见 [操作单](INDOOR_10M_TEST_20261028.md)。
