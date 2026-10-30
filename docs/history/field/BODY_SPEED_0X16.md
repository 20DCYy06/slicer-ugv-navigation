# 0x16车体IMU闭环：切换与首次验收

本页用于已安装body_speed版本的切换与验收。2026-10-25实车握手、零速接口和首次地面左转已验证；一次目标成功，A轮欠速仍待核对，详见[本次结果](BODY_TURN_RESULT_20261025.md)。只切换我方driver的模式，继续使用现有定位、12m导航、Guard/Mux及MCU Watchdog，不改固件、融合或TF。当前默认仍为PWM参考模式，新模式必须明确选择。

2026-10-25已部署到Jetson独立工程并完成ARM64构建安装；控制包130项测试记录、工具68项均通过，无失败或跳过。测试只运行于断网、无设备容器的独立ROS环境，没有重启车端真实节点或发送控制指令。部署与失败测试证据保存在`.test-results/body-speed-20261025/`，其中`deploy/result.json`记录安装模块/配置哈希；不能作为实车转向通过依据。

## 切换底盘终端

定位、导航终端保持运行。在操作终端的ROS shell执行：

```bash
rosrun ugv_slicer_control control.py hold
```

确认自主保持且车停稳后，在**底盘终端**按Ctrl-C结束原driver；若回到`cup@…:/workspace`，执行`exit`回到车端`potato@…`宿主机，再按下面固定命令启动。普通ROS shell与带串口的shell提示符相同，但只有进入容器时带`UGV_SERIAL_DEVICE`的容器才能访问底盘；在普通cup shell里补设变量不会新增Docker设备映射。

## 日常启动：统一从车端potato宿主机执行

每个角色一个终端；已运行的部分保持即可。空cup shell先`exit`回到potato，再执行对应完整命令。全部使用独立工程根目录及其安装文件，不在宿主机展开`rospack`。

**定位终端**：LIO、scan、EKF及TF。

```bash
cd /home/potato/slicer_ugv_ros1
./tools/slicer_shell.sh roslaunch slicer_bringup sensors.launch root:=/slicer
```

**导航终端**：12m滚动全局地图、DWA、Guard和Mux。

```bash
cd /home/potato/slicer_ugv_ros1
./tools/slicer_shell.sh roslaunch ugv_bringup slicer_odom.launch connect_control:=true control_config:=/workspace/install/share/ugv_bringup/config/slicer_driver_wheel_speed_reference.yaml global_costmap_config:=/workspace/install/share/ugv_bringup/config/rolling_global_12m.yaml
```

这里的wheel_speed配置供Guard/Mux读取，不决定底盘实际使用的协议。

**底盘终端**：唯一串口owner、body_speed／0x16。

```bash
cd /home/potato/slicer_ugv_ros1
UGV_SERIAL_DEVICE=/dev/c30d ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_driver.launch config_file:=/workspace/install/share/ugv_bringup/config/slicer_driver_body_speed.yaml control_mode:=body_speed publish_tf:=false
```

2026-10-26现场曾在没有设备映射的普通cup shell启动driver，日志为`Expected unique C30D ... found []`；宿主机当时的`/dev/c30d`正常指向`ttyACM0`。按底盘命令重新进入带设备的容器，才能解决这类启动入口问题；不需要修改串口协议或解除故障保护。

不要另起第二个driver。正常启动会显示CONNECTED与`body_speed ... 0x16`；启动不release、不自动发任务。没有故障时无需为了启动软件额外切Motor OFF；真实fault、缺扩展或IMU未就绪则保留报错，不改成其他模式掩盖失败。

启动后在普通**操作终端**的ROS shell先只看接口：

```bash
rosparam get /slicer_c30d_driver/provenance/control_mode
rosparam get /slicer_c30d_driver/provenance/body_extension
rostopic echo -n 1 /slicer_c30d_driver/ready
rosrun ugv_slicer_control control.py state
```

预期body_speed；扩展version=1、who_am_i=234(0xea)、nonce匹配，当前imu_flags=7；Mux仍保持。ready还要求Motor使能与动力存在，动力关闭时false不等于接口错误。持续反馈在原`/slicer_c30d_driver/diagnostics`中的`body_telemetry`事件，须检查实际IMU反馈推进；启动快照不能证明它一直健康。

**每次Mux／导航重启，或手动hold之后，需要现场显式release才能执行新目标。** `goal.py send`只发目标，不会解除控制保持；当前会话已经released时则不需要每个目标都重复授权。发送前可用`goal.py plan`检查全局路径，成功也不代替实车局部避障与跟随验收。`--timeout 120`仅是客户端等待上限，不能覆盖move_base自己的振荡／规划失败判定。

## 单次纯转验收

用户在车旁、地面有足够转向空间时执行。IMU外环反馈的是**车体转动**；架空并固定车身会令测得角速近零、外环继续加轮速，不能把这类台架结果当作角速跟随。

采集终端先执行以下一行，等RECORDING STARTED。这个采集器已包含body诊断，不必新增IMU话题：

```bash
python3 tools/collect_odom_test.py --duration 90 --output evidence/body-turn-$(date +%Y%m%d-%H%M%S)
```

操作终端只做一次相对当前位置左转45°目标：

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation goal.py send --relative --x 0 --y 0 --yaw 0.7854 --timeout 15
rosrun ugv_slicer_control control.py hold
```

当前yaw容差0.25rad（约14°），本次判据是能够起转、方向与IMU反馈一致、取消/hold后停稳；不把SUCCEEDED解释为精确45°。异常时直接取消/hold或现场物理急停。结束留5秒停车尾段，再Ctrl-C结束采集；不连续重试新目标。

应记录：Action最终状态、实际转向/角度、是否自行停稳及干预，0x16双轴目标（mm/s、mrad/s）、0x93实测wz（rad/s）、source=3的活动段、IMU age/flags、ACK、四轮反馈/duty、电压/fault和停止尾段。确认以上后再回到(1,1)二维目标。

## 接口与软件边界

- `0x16 + <Ihh>`：session nonce、vx毫米/秒、wz毫弧度/秒；**诊断kind=22**，kind=16仍是旧0x10。
- 配置仍先经0x12极性握手，0x17/0x84确认body扩展，0x93给出IMU与车体反馈。输入、Motor/动力、配置或反馈无效时发0x16零；异常关闭也不回退0x10/PWM。
- 0x90轮速遥测不能替0x93保活；默认body接收/时间推进超时0.30s、IMU报告age最多100ms，来自当前协议参考。普通遥测硬超时及MCU Watchdog保持原值。
- MCU负责车体外环、轮速换算、PI和电桥输出；主机不加PID或PWM补偿。0x16没有主机PWM上限字段，低角速度仍可能令MCU提高轮速/输出。
- `/wheel/odom`继续由原0x90累计编码器生成；没有把IMU再接进EKF，没有新增TF或导航节点。导航终端原wheel_speed参考配置可继续供Guard/Mux读取，无需为这次切换重开导航。
- 软件测试使用匿名管道、假时钟或隔离ROS环境，只证明编码、停零和接口行为；不能证明实际转向、低速死区消失或物理停车。
