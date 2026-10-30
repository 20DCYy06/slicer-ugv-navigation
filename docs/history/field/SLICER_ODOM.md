# Slicer odom 实车短单

适用新工作区`/home/potato/slicer_ugv_ros1`。用户已允许同步修复；新driver已由用户启动，真实release/send/hold均由用户现场执行。已有终端与数据进展见[继续修复验证](COMPLETE_FIX_VALIDATION.md)，不要为重复本短单重启已工作的定位或driver。先完成直行与有效转向，再检查基础障碍响应；不重做WASD基础运动测试，不迁移RTK或自动恢复。

## 先认识现有接线

```text
已有Slicer LIO/轮速EKF → /odom、odom→base_link、/scan
/ugv/move_base → /cmd_vel/nav_raw → 选定适配版Guard → /cmd_vel/nav
                                               ↓
                                    选定适配版Mux → /cmd_vel
                                                       ↓
                                     独立受控C30D driver → MCU
```

新`ugv_slicer_control`保留Guard/Mux的选定适配版，并增加独立受控C30D driver副本和标准诊断；来源见`src/ugv_slicer_control/SOURCES.json`。它和队友源码不是同一个版本，不能靠rosparam让旧进程获得这些能力。新driver实际导入连续PWM函数；仍运行原driver时，新包旁的同名函数不会修复其旧映射。导航launch不自动启动driver，没有新任务状态机、守护进程或控制转发层。

禁倒车时Guard整条拒绝负vx请求并发布hold_reason，不再把倒车圆弧改成纯转；原始纯转仍允许。Slicer导航入口默认标准DWA（min_vel_x=0），`local_planner:=teb`保留原TEB对照；通用odom入口默认TEB。Guard从实际所选插件读取限幅，与已有源限制取交集；不改GlobalPlanner、unknown、外廓、角速上限、到点容差或固件。DWA不使用TEB的`min_obstacle_dist=.144`参数；相同外廓/膨胀不保证相同障碍间隙，实物响应需现场验证。

新Mux启动保持 `manual_hold=true`，只通过参数 `manual_release_owner=/ugv/control` 授权这个真实的新节点；`control.py` 每次执行后退出，不冒充 `/cup_nav_executor`。它适配已有 `/cup_nav/manual_intent`/`manual_state` 协议，检查当前实例、接管代次、唯一非锁存状态及本次ACK。`release` 不发目标，有活动导航目标时拒绝释放；若命令等待期间Mux重启或发生新接管，拒绝沿用本次释放请求，须重新明确执行release。`hold` 锁存接管并取消新导航器的目标，不自动归还或续任务。

## 1. 保留定位，由用户启动唯一driver

先由现场启动已有Slicer定位/雷达链，保持唯一 `/odom` 与 `odom→base_link`；本工程不重复启动LIO、轮速EKF或TF。

此前用户启动的`slicer-navigation`是整套map导航，包含required的旧move_base/Guard/Mux。不要只杀其中节点；required退出会带停整套launch。切换时由用户先停车、结束原启动器，传感器沿用现有独立入口，不重新构建或修改队友树。

以下用于从停止状态启动，已运行的新driver不必重开。先完成本版本ARM64独立构建和无设备验证，确认新工程install可见，`slicer_driver.yaml`中的设备、符号、轮端参数和限幅与原运行配置一致。`tools/slicer_shell.sh`默认不映射设备；只有显式设置`UGV_SERIAL_DEVICE`时，才映射该一个`/dev`下字符设备并加入其GID。不枚举或自动选择串口，不改变现有容器挂载，也不会自动启动driver。实际C30D设备路径须现场核对，不能只凭tty编号选择。

1. 用户在当前Mux实例执行hold并按第6节确认停车。保持held，不发活动目标。
2. 用户停止原driver所在**终端1**的启动器，确认其进程及串口占用已结束。保留传感器和新Guard/Mux，不启动第二个driver，不用重名节点强制替换。
3. 按[现场操作单](ODOM_FIELD_CHECK.md)首次安装固定USB身份规则后，在宿主机新终端以 `/dev/c30d` 打开带该单个串口的环境。规则匹配 `1a86:55d4 / 5CF0016377`，USB重插或重启后无需记忆tty编号。此命令只进入shell：

```bash
cd /home/potato/slicer_ugv_ros1
UGV_SERIAL_DEVICE=/dev/c30d ./tools/slicer_shell.sh
```

入口在Docker运行前拒绝不存在、非字符设备或不在`/dev`下的路径；将固定别名解析为真实tty后映射，以兼容driver的auto扫描，不挂整棵`/dev`或添加`--privileged`。映射设备不等于已确认C30D身份：driver还会检查`1a86:55d4 / 5CF0016377`，并以`TIOCEXCL`和`flock`独占串口；这些检查不能代替第2步停止旧owner。确认共享master与原有Guard/Mux仍在、Mux保持held后，仅由用户在该shell内显式启动一个新driver：

```bash
roslaunch ugv_bringup slicer_driver.launch publish_tf:=false
```

`config_file`默认是本包`ugv_bringup/config/slicer_driver.yaml`；需要不同路径时显式传`config_file:=该环境中的实际文件路径`，不覆写队友配置。启动不解除held。在另一个导航环境终端依次核对，持续观察命令用Ctrl-C结束：

```bash
rosnode info /slicer_c30d_driver
rostopic info /cmd_vel
rosparam get /slicer_c30d_driver/provenance
rostopic echo /slicer_c30d_driver/diagnostics
```

`provenance`和启动日志保存实际配置及导入路径/hash，不依赖订阅到非锁存startup诊断；再确认新鲜`ready`及轮速/遥测。新driver保留原限幅、立即零、反向过零、Watchdog和遥测保护，诊断不是物理停车证明。

全链必须保持`raw → Guard → Mux → driver`，不能有第二个Mux、Guard、导航raw生产者或同名`/ugv/move_base`。旧Mux由其启动者结束；不另开串口读取遥测，不绕过Guard重测基础运动。

电台地址为 `192.168.10.33`，不是过期地址。以下环境入口默认沿用它，不改现有电台、网卡或共享roscore。纯车端本机观察时可显式用localhost，但不要据此改无线协作配置。

## 2. 打开新工程的ROS环境

车端每个需要的终端执行：

```bash
cd /home/potato/slicer_ugv_ros1
./tools/slicer_shell.sh
```

这只启动Noetic环境shell：host网络、新工作区、队友树只读挂载到`/slicer`，读取已有`devel_navigation`作为Slicer依赖，再叠加新工程install；未设置`UGV_SERIAL_DEVICE`时无设备映射。入口不启动master/传感器/driver，不释放控制权。普通shell供导航、观察与记录使用；driver仅由用户按第1节打开显式映射串口的shell后执行标准roslaunch。与WSL的`tools/noetic.sh`隔离测试环境不同，不能在此运行软件模型冒充实车。

若刚结束旧整套导航、传感器也随之停止，在一个新环境shell只启动原有定位/雷达launch（没有建图、AMCL或driver）：

```bash
roslaunch slicer_bringup sensors.launch root:=/slicer
```

如果传感器已运行则跳过。所有定位、几何、雷达和轮速配置均沿用队友当前文件，不重新生成或覆写。

## 3. 先观察与规划，不接速度链

终端A，在新环境shell内：

```bash
roslaunch ugv_bringup slicer_odom.launch namespace:=ugv_check
```

默认 `connect_control=false`，只有 `/ugv_check/move_base`；raw为 `/ugv_check/cmd_vel/nav_raw`，没有Guard订阅它。终端B检查：

```bash
rostopic info /odom
rostopic info /scan
rostopic hz /odom
rostopic hz /scan
rosrun tf tf_echo odom base_link
rosrun ugv_navigation goal.py --action /ugv_check/move_base plan --relative --x 0.5 --y 0
```

每个观察命令用Ctrl-C结束，再执行下一个。必须有真实、唯一且连续的新鲜输入，scan的frame经真实TF可变换，不能把只有subscriber的topic当作定位就绪。`plan`只调用GetPlan，打印固定odom目标和路径点数，不发送Action。Costmap启动后要先有实际扫描观测，服务出现不代表目标单元已经free。本轮首次即调用为空，收到12帧真实costmap后.5m目标才明确为free并返回21点；不把固定等待时间当通用就绪保证。非空路径才算本次短目标可规划；失败时保留输出、查当前scan/costmap/TF，不用清图或放开unknown凑通过。

当前默认参考：costmap观测周期上限 `.20s`、TF容差 `.30s`，Guard TF新鲜度 `.50s`。这些来自既有配置，不是停车或传感器性能验收。观察costmap可用 `rostopic echo -n 1 /ugv_check/move_base/local_costmap/costmap`，实际障碍和unknown必须反映现场。确认完成后在终端A Ctrl-C，退出检查实例。

## 4. 正式接线，仍保持锁定

确认无同名节点和冲突发布者。若更换已运行的导航器，用户先在B执行hold并确认停车，再在A退出原launch；定位与D保持。终端A重新进入新环境后：

```bash
roslaunch ugv_bringup slicer_odom.launch connect_control:=true
```

只启动 `/ugv/move_base`、`/slicer_nav_arc_guard`、`/slicer_velocity_mux`。无driver、无TF发布者、无控制释放。终端B：

```bash
rosrun ugv_slicer_control control.py state
rostopic info /cmd_vel/nav_raw
rostopic info /cmd_vel/nav
rostopic info /cmd_vel
rosparam get /slicer_nav_arc_guard/planner_namespace
rosparam get /slicer_nav_arc_guard/costmap_topic
rosparam get /slicer_velocity_mux/manual_release_owner
rosrun ugv_navigation goal.py plan --relative --x 0.5 --y 0
```

应分别为`/ugv/move_base`、`/ugv/move_base/local_costmap/costmap`、`/ugv/control`；state为held，唯一driver订阅Mux输出。换版后核对新driver启动诊断中的路径/hash；未换版则仍是旧映射，不能混记结果。无目标时Guard显示`command missing/stale`属正常，不要求假造非零命令消除提示。`ready`旧锁存true不是持续遥测证明。

每次重新启动本launch都会产生新Mux实例并回到startup锁定，之前的release不沿用。held时move_base仍能接收目标、规划并产生raw，Guard也可能打印非零转向；这些都不能证明Mux已放行。先确认本次实例状态，准备运动时才按下一节明确release。

## 5. 用户现场发送短目标

开阔区域确认可接管后，终端B：

```bash
rosrun ugv_slicer_control control.py release && \
rosrun ugv_navigation goal.py send --relative --x 0.5 --y 0 --yaw 0 --timeout 30
```

`&&` 使放行失败时不会继续发送目标；不要跳过 release 报错。录包器可被动订阅控制意图，客户端仍要求实际Mux订阅，并校验唯一Mux状态发布者及本次实例/代次/ACK。

`--relative` 读取一次当前 `odom←base_link`，用标准TF变换得到固定odom目标并打印；不是朝odom原点行驶，也不是跟随base移动的目标。TF缺失/过期即失败。超时或Ctrl-C取消自己的goal，无自动恢复。状态：

```bash
rostopic echo /ugv/move_base/status
rostopic echo /ugv/move_base/result
```

SUCCEEDED只说明当前软件容差到点，还要做下面的停车确认。保留已有`.125m/s`、`.25rad/s`调试上限、Guard限速交集、禁倒车与Watchdog，不自动提高起转补偿。新driver连续映射保留纯转端点，`.25rad/s`仍约71‰；300‰是既有WASD操作值，最低持续起转门槛未测得。修复不保证起转或历史超调消失，不能把请求值当实际速度。若转向不足，先记录，不绕过Guard或擅自提高限速，不重复WASD基础测试。

## 6. 取消、接管和停车确认

```bash
rosrun ugv_slicer_control control.py hold
rosrun ugv_slicer_control control.py state
rostopic echo -n 1 /cmd_vel
rostopic echo /wheel/odom/twist/twist
```

确认对应目标为PREEMPTED/RECALLED或其他终态，state为held且无导航输出，最终Twist为零，轮速里程计有**新鲜连续数据**且已停稳。既有软件参考为线速≤`.03m/s`、角速≤`.05rad/s`连续1s；不是制动距离或精度验收。没有新鲜轮速/遥测不等于停车成功。最后一个观察命令用Ctrl-C退出。

`hold`先确认Mux已接管，再取消该导航器全部目标；取消链路故障也不会解除held。若返回非零，读取提示区分“held已确认但取消未确认”和“未取得held ACK”。单独取消Action可用 `rosrun ugv_navigation goal.py cancel`，但需要接管停车时先用hold，避免取消链路等待推迟接管。物理急停始终保留。导航结束后保持held，再在launch终端Ctrl-C；不要用“杀进程”替代停车确认。

若使用队友原WASD键盘入口，先执行本 `hold`，再使用原键盘程序（只起键盘，不重复起driver/Mux）。旧键盘没有新的接管消息；自主期间仅按WASD不会自动解除/取得本Mux的控制权。人工归还必须重新显式release，并重新发新目标。

人工挪车前同样先hold并确认停车，再挪车；Ctrl-C取消目标本身不锁定Mux。相对目标的正y为发送瞬间车体左侧、负y为右侧，表示目标位置，不是横移速度。当前max_vel_y=0，需转向和前进才能到侧方；relative yaw=0要求终点朝向与发送瞬间相同。

## 7. 基础障碍响应

短距离到点与有效转向通过后，按同样短目标检查路径中基础障碍：真实scan→costmap标障，观察所选规划器/Guard绕行或停止，堵死允许按既有patience终止。需要取消时用上述hold/取消链，不能自动clear/旋转/续任务。保留实际结果；软件模型墙体阻挡不是实车障碍验收。

2026-10-23现场补记：0.5m/1m目标均软件SUCCEEDED，但实际分别约0.8m以上/1.3m；短距离到点验收尚未通过，先定位超调，不推进障碍测试。详细事实见[验证记录](INTEGRATION_VALIDATION.md)。下次仅由用户决定并执行复测：换版和新鲜诊断确认后，在另一个导航环境终端至少提前3秒启动标准ROS记录，再按第5节明确release/send一个0.5m目标。本轮未执行以下记录或运动命令：

```bash
mkdir -p /workspace/evidence
rosbag record -o /workspace/evidence/odom_stop \
  /odom /lio/odom /wheel/odom /Odometry /tf /tf_static /scan \
  /cmd_vel/nav_raw /cmd_vel/nav /cmd_vel \
  /ugv/move_base/goal /ugv/move_base/current_goal /ugv/move_base/feedback \
  /ugv/move_base/result /ugv/move_base/status /ugv/move_base/cancel \
  /cup_nav/manual_intent /cup_nav/manual_state /cup_nav/emergency_stop \
  /slicer_nav_arc_guard/hold_reason /slicer_nav_arc_guard/adjustment \
  /slicer_c30d_driver/ready /slicer_c30d_driver/diagnostics /rosout_agg \
  /ugv/move_base/GlobalPlanner/plan \
  /ugv/move_base/TebLocalPlannerROS/global_plan \
  /ugv/move_base/TebLocalPlannerROS/local_plan \
  /ugv/move_base/global_costmap/costmap /ugv/move_base/global_costmap/costmap_updates \
  /ugv/move_base/local_costmap/costmap /ugv/move_base/local_costmap/costmap_updates
```

记录前保存当次导航、Guard/Mux、driver、融合参数，并用`rosparam get /slicer_c30d_driver/provenance`保存配置/导入路径及hash；`/Odometry`须核对现场实际原LIO话题。同步拍摄固定车体标记和地面刻度，记录目标/goal ID。终态后继续至少5秒；需接管时按第6节hold并标记时刻，人工接管段不能算自主停车通过。前进异常时不追加目标；仅在该段允许继续时，再由用户选择一个相对yaw约45°的纯转导航目标，仍走同一全链，不提高角速。

比较同一goal的成功判断、首次raw/Guard/Mux零、driver发送零、MCU接受及回报、轮速零、地面停稳，不预设不同话题的接收顺序。诊断`accepted_command_seq`只表示接受；发送PWM为permille，`pwm_timer_ticks_abcd`为timer ticks；两者及轮速都不是地面制动力真值。参考固件零态意图为`00/coast`，仍不能证明实际Flash、电桥状态或滑行距离。Twist无Header，ROS/主机/MCU时钟需关联，接收时差不能直接当精确执行延迟。输出只保存本次新试验，不读取或复制旧历史bag。
