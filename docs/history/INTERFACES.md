# 标准 ROS 接口

以下默认 namespace 为 `/ugv`，Action server 为 `/ugv/move_base`。接口名由 launch 的 `namespace` 决定；客户端 `--action` 必须同步指定。本文是接口契约，实车操作顺序及限制见[现场短单](field/SLICER_ODOM.md)；实际运动由用户现场执行。

## 启动与输入输出

```bash
source devel/setup.bash
roslaunch ugv_bringup odom.launch
```

| launch 参数 | 默认值 | 约定 |
| --- | --- | --- |
| `namespace` | `ugv` | move_base 所在 ROS namespace。 |
| `odom_topic` | `/odom` | `nav_msgs/Odometry`；位置 m、速度 m/s 与 rad/s，通常 `header.frame_id=odom`、`child_frame_id=base_link`。 |
| `scan_topic` | `/scan` | `sensor_msgs/LaserScan`，既有 Slicer 360° 扫描，`header.frame_id=base_link`；量程 m、角度 rad。 |
| `cmd_vel_topic` | `cmd_vel/nav_raw` | 相对名解析为 `/ugv/cmd_vel/nav_raw`，类型 `geometry_msgs/Twist`；linear.x 为 m/s、angular.z 为 rad/s，只允许接 Guard raw 入口。 |
| `global_frame` | `odom` | 两个滚动 costmap 的规划坐标系。 |
| `base_frame` | `base_link` | 机器人基座坐标系。 |
| `config_dir` | `ugv_bringup/config` | YAML 参数目录。 |

必须由既有定位提供有效、时间同步的 `odom → base_link` TF 和 odometry；scan 在非默认 frame 时必须有真实有效 TF 能变换到规划坐标系。导航入口不发布生产 TF，也不启动驱动、静态图或 AMCL；独立driver默认`publish_tf=false`，保持既有融合TF唯一。目标 frame 必须能变换到 `global_frame`；默认没有 `map → odom`，不能只把目标标签写成 `map`。

更换 namespace 的软件示例：

```bash
roslaunch ugv_bringup odom.launch namespace:=test_ugv
rosrun ugv_navigation goal.py --action /test_ugv/move_base send --x 0.5 --y 0 --yaw 0 --frame odom
```

实车接线入口 `roslaunch ugv_bringup slicer_odom.launch connect_control:=true` 将raw接 `/cmd_vel/nav_raw`，并配置选定适配版Guard的planner namespace、costmap、实际只读源配置，关闭turn boost/patrol spin。Mux保持启动锁定，明确授权新 `/ugv/control`；导航入口不包含driver。无控制接线时用 `connect_control:=false` 和独立namespace检查输入/规划。禁止将raw输出改到 `/cmd_vel`。

Guard在`max_reverse=0`时整条拒绝负vx请求，输出零并发布`/slicer_nav_arc_guard/hold_reason`；不再保留其中角速冒充原倒车圆弧。原始`vx=0`纯转仍可经过限幅和扫掠检查。这是出口约束，不是TEB前向可执行性证明；不将`max_vel_x_backwards=0`当作TEB 0.9.1的硬禁倒车开关。

## MoveBaseAction：发送、反馈、结果和取消

类型为 `move_base_msgs/MoveBaseAction`，名称 `/ugv/move_base`。通过 actionlib 发送 `MoveBaseGoal.target_pose`（`geometry_msgs/PoseStamped`）：

- `header.frame_id` 为目标所属 TF frame，例如 `odom`，不带前导 `/`；`stamp` 为 ROS 时间，通常用 `rospy.Time.now()`。
- `pose.position.x/y/z` 单位 m；平面目标 z=0。
- `pose.orientation.x/y/z/w` 为归一化四元数；平面 yaw 用 rad，绕 +z 逆时针为正，四元数为 `(0, 0, sin(yaw/2), cos(yaw/2))`。

目标应处于当前已观测滚动窗口的可达区域。无静态地图，不承诺窗口外远距离导航；`allow_unknown=false`，不会把未观测区域当可行路。

`MoveBaseFeedback.base_position` 是导航当前估计位姿（PoseStamped），不是距离、进度百分比或物理到货反馈。`MoveBaseResult` 本身为空消息，成功/失败依据 actionlib terminal status 和 status text：SUCCEEDED 表示所选标准规划器按当前软件位姿/容差判断到点，不能证明物理配送完成、制动停稳或控制权归还；ABORTED 表示规划/控制失败等终止；REJECTED、PREEMPTED、RECALLED、LOST 也不能当成功。PENDING、ACTIVE、PREEMPTING、RECALLING 为过程状态。

`SimpleActionClient.cancel_goal()` 请求取消本客户端当前 goal；`cancel_all_goals()` 请求取消该 server 的全部 goal，可能影响其他客户端。取消是异步请求，必须观察相应 goal ID 的终态确认，不能把“已发布 cancel”当作 ACK 或物理停车证明。新的目标可能抢占旧目标，队友需自行协调同一 server 的使用。

命令行客户端：

```bash
# --action 必须放在 send/cancel 子命令之前；yaw 单位 rad。
rosrun ugv_navigation goal.py --action /ugv/move_base send --x 0.5 --y 0 --yaw 0 --frame odom --timeout 120

# 只规划，不发Action；取当前base前方.5m，转换一次并固定到odom。
rosrun ugv_navigation goal.py plan --relative --x 0.5 --y 0

# 用户现场或隔离模型的相对短目标：TF仅采样一次。
rosrun ugv_navigation goal.py send --relative --x 0.5 --y 0 --yaw 0 --timeout 30

# 明确请求取消该 Action server 全部目标；随后检查 status/result。
rosrun ugv_navigation goal.py --action /ugv/move_base cancel
```

`send` 超时或 Ctrl-C 会取消它自己发送的 goal，不自动重发；脚本等待有限时间确认，因此超时输出或退出码不等于已确认取消。`send` 仅在收到 SUCCEEDED 时返回 0；失败/超时返回 1，Ctrl-C 返回 130。`cancel` 子命令返回 0 只表示已发出全部取消请求，须继续观察 ACK。

队友只需 ROS 标准包，不依赖本工程内部 Python 目录：

```python
#!/usr/bin/env python3
import math
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node('teammate_goal', disable_signals=True)
client = actionlib.SimpleActionClient('/ugv/move_base', MoveBaseAction)
if not client.wait_for_server(rospy.Duration(10)):
    raise RuntimeError('move_base unavailable')
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'odom'
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 0.5  # m
goal.target_pose.pose.position.y = 0.0
yaw = 0.0  # rad
goal.target_pose.pose.orientation.z = math.sin(yaw / 2)
goal.target_pose.pose.orientation.w = math.cos(yaw / 2)
client.send_goal(goal)
try:
    if not client.wait_for_result(rospy.Duration(120)):
        client.cancel_goal()
        acknowledged = client.wait_for_result(rospy.Duration(3))
        raise RuntimeError('timeout; terminal ACK=' + str(acknowledged))
    print('state:', client.get_state(), client.get_goal_status_text())
    if client.get_state() != GoalStatus.SUCCEEDED:
        raise RuntimeError('navigation did not succeed')
except KeyboardInterrupt:
    client.cancel_goal()
    print('cancel terminal ACK:', client.wait_for_result(rospy.Duration(3)))
finally:
    rospy.signal_shutdown('client finished')
```

## Service：只规划与清图

| 默认服务 | 类型 | 语义 |
| --- | --- | --- |
| `/ugv/move_base/make_plan` | `nav_msgs/GetPlan` | 请求 `start`、`goal`（PoseStamped）和 `tolerance`（m），返回 `plan`（nav_msgs/Path）；只计算路径，不发送运动目标。空路径不能当作成功。 |
| `/ugv/move_base/clear_costmaps` | `std_srvs/Empty` | 显式请求重置 costmap 数据；不是解除障碍/传感器故障或恢复任务的证明。 |

`make_plan` 的 frame/TF/窗口约束与 Action 相同。本工程 `make_plan_clear_costmap=false`、`make_plan_add_unreachable_goal=false`；调用者应检查返回 Path。move_base 的服务行为受运行状态约束，活动目标期间不能依赖 make_plan 提供独立并行规划。

清图需谨慎：清掉观测数据不代表障碍消失，不能用重复调用突破 unknown。清图后仍须有新鲜 scan、有效 TF 和可行路径；不将该服务接成自动盲目恢复循环。本工程禁用旋转/清图 recovery。

## 观察与排查

| Topic | 类型/用途 |
| --- | --- |
| `/ugv/move_base/status` | `actionlib_msgs/GoalStatusArray`；按 goal ID 看状态及终态 ACK。 |
| `/ugv/move_base/feedback` | `move_base_msgs/MoveBaseActionFeedback`；当前估计位姿与对应 goal ID。 |
| `/ugv/move_base/result` | `move_base_msgs/MoveBaseActionResult`；终态 status/text；result payload 为空。 |
| `/ugv/move_base/current_goal` | `geometry_msgs/PoseStamped`；导航当前目标观察值。 |
| `/ugv/move_base/GlobalPlanner/plan` | `nav_msgs/Path`；全局规划路径。 |
| `/ugv/move_base/TebLocalPlannerROS/local_plan`、`global_plan` | `nav_msgs/Path`；TEB 局部路径和参考路径；后一个名字同样位于 `TebLocalPlannerROS/` 下。 |
| `/ugv/move_base/DWAPlannerROS/local_plan`、`global_plan` | `nav_msgs/Path`；选择DWA时的局部路径和参考路径。Slicer入口默认DWA，通用odom默认TEB；启动参数`local_planner:=dwa|teb`，公共Action/GetPlan不变。 |
| `/ugv/move_base/global_costmap/costmap`、`local_costmap/costmap` | `nav_msgs/OccupancyGrid`；分别位于 `global_costmap/` 和 `local_costmap/` 下，查看观测、障碍和 unknown。 |
| `/odom`、`/scan`、`/tf`、`/tf_static` | 查看输入频率、frame、时间戳及真实 TF 链。 |
| `/ugv/cmd_vel/nav_raw` | `geometry_msgs/Twist`；仅说明规划器 raw 输出，不是底盘实际速度或安全链放行结果。 |

表中缩写的相邻 topic 须使用完整前缀；namespace/输入 remap 变化后相应调整。例如：

```bash
rostopic echo /ugv/move_base/status
rostopic echo /ugv/move_base/result
rostopic echo /ugv/move_base/current_goal
rostopic hz /odom
rostopic hz /scan
rosrun tf tf_echo odom base_link
```

取消目标后观察终态、最终Twist与新鲜轮速，再确认Mux保持状态；任何软件命令都不能代替物理急停。软件验证范围及未验证事项以[本轮实车接线记录](field/INTEGRATION_VALIDATION.md)为准。

## 显式Mux控制权交接

```bash
rosrun ugv_slicer_control control.py state
rosrun ugv_slicer_control control.py release
rosrun ugv_slicer_control control.py hold
```

客户端真实身份默认 `/ugv/control`，由Mux私参 `manual_release_owner` 明确授权；namespace改变时使用 `--namespace /新namespace`，launch默认owner随namespace变化。每次命令短生命周期，无自动重试、释放或任务恢复。state/release/hold以当前唯一非锁存Mux状态绑定实例/代次，并等待本请求ID的确认。release在有活动目标、急停或新鲜手动包时拒绝；等待期间实例或代次变化也拒绝，须重新明确执行release。hold先锁定Mux，再通过标准Action cancel取消本导航器目标并确认非busy状态；取消失败仍保持held，非零说明未确认部分。

底层复用现有 `std_msgs/String` 的 `/cup_nav/manual_intent` 与 `/cup_nav/manual_state`，只是适配边界，不对队友引入新的JSON任务协议。hold会取消该 `/ugv/move_base` server全部目标；队友需协调server使用。独立driver保留原Watchdog和串口保护，手动程序的使用仍按现场短单执行。

## 独立C30D driver与诊断

`roslaunch ugv_bringup slicer_driver.launch`只启动`/slicer_c30d_driver`，不启动导航、Mux或传感器，不解除控制保持。节点要求显式`~config_file`；launch的`config_file`默认指向`ugv_bringup/config/slicer_driver.yaml`，可通过`publish_tf:=false`保持TF不重复。实际串口所有者必须唯一；只由用户停止原driver后替换，本次新增轮速模式尚未在实车执行。

默认PWM模式下，新工程的`c30d_driver_node.py`实际调用本包`control_core.wheel_pwm`，后者调用连续映射；旧driver仍会导入自己的旧模块，不能靠旁边多一个同名文件修复。启动日志、startup诊断及`/slicer_c30d_driver/provenance`参数记录实际配置与导入路径/hash；可用`rosparam get /slicer_c30d_driver/provenance`核对，不依赖订阅到非锁存的startup消息。连续映射保留纯直行/纯转端点、限幅、立即零、反向过零及Watchdog，不等于实速标定或制动验收。

`~control_mode`为空时使用配置文件的`control_mode`，缺省为`pwm_reference`；仅支持`pwm_reference|wheel_speed`，未知值拒绝。显式`wheel_speed`使用现有`0x12`符号配置和`0x10`四轮ABCD int16 mm/s协议，MCU负责闭环。速度模式要求独立`pwm_signs`和`encoder_signs`，不借用`test_pwm_signs`，不混发非零PWM，不自动降级。零/超时立即停零，轮反向先四轮全零；配置丢失、direct/calibration或fault时停止并退出。ready还需新鲜configured、EN和power，不代表物理方向/单位或固件PI已验证。

候选配置`slicer_driver_wheel_speed_reference.yaml`尚未实测，本轮未部署，不作为现场切换命令。其符号和几何依据及下一步见[底盘速度接口核查](field/BASE_CONTROL_FINDINGS.md)。`0x10`没有主机PWM限幅字段；该模式仅接受600‰的参考兼容声明，但**不能在主机执行此PWM上限，也不能证明实际固件上限**。默认PWM配置保持原主机限幅。

| Topic | 类型与语义 |
| --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist`，仅由Mux输出供driver订阅；不是导航可直接发布的入口。 |
| `/wheel/odom` | `nav_msgs/Odometry`，保留既有轮速里程计接口，不代替外部地面位移。 |
| `/slicer_c30d_driver/ready` | 保留既有就绪接口，锁存true不证明最新命令已被MCU执行。 |
| `/slicer_c30d_driver/diagnostics` | `diagnostic_msgs/DiagnosticArray`，由唯一串口所有者发布`startup/command_rx/command_tx/serial_tx/configuration_ack/telemetry/fault/shutdown`事件。 |

诊断带ROS stamp和`host_monotonic_s`，分别观察命令接收、目标/发送参考及发送序号、主机收到的MCU帧序号/tick/ack/flags/电压/编码器。`command_tx`记录`control_mode`及本模式参考；`serial_tx`带串口写入起止时间、序号和success，写入失败不记作成功。`kind=17`的`command_pwm_permille_abcd`单位为‰，`kind=16`的`speed_mm_s_abcd`单位为mm/s，两者分别统计。`pwm_timer_ticks_abcd`来自固件软件`duty[]`，单位timer ticks，不是CCR寄存器读回，不能与发送值直接等量比较。`accepted_command_seq`仅表示固件接受命令，不等于电桥或车体执行确认；主机发送事件也不等于MCU收到。Twist无Header，多个主机及MCU时钟未对齐时，话题接收时差不能当精确执行延迟。

参考固件源码中的零态为`00/coast`；实际刷入版本、电桥极性/使能及地面滑行仍未证实。本轮不改固件。诊断用于同步取证，不新增任务状态机或自动恢复。
