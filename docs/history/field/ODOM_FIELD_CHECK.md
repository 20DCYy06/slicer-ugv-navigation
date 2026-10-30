# odom 现场一页操作单

**当前定位、A、D均已运行，真实输入与零速接口检查通过，前方0.5m GetPlan得到22个路径点；尚未发运动目标。** ROS控制版本 `f4bd9ee` 已完成Jetson ARM64安装，包含获批LIO新鲜度检查；采集器TF兼容修复 `598c899` 也已同步。合并固件 `956eca3` 仍无本次非零验收，先做第2节离地检查，不重跑WASD基础测试。详细结果见[交付记录](ODOM_READY_20261025.md)。

## 1．启动前检查

本次同步、安装已由助手完成，无需再执行复制和构建；[源码更新文档](ODOM_SOURCE_UPDATE.md)仅留作重新安装参考。`/dev/c30d` 已指向当前C30D串口，也无需重复安装规则。今后换串口硬件或重装系统时，才按USB身份重新核对规则。

**本次换电后已经恢复，不用重开。** 定位与导航分别在 `ugv-sensors`、`ugv-navigation` 后台容器运行，D由用户在原终端启动。宿主机可用 `docker logs --tail 30 -f ugv-sensors` 或 `docker logs --tail 30 -f ugv-navigation` 观察，Ctrl-C只退出日志；需要结束对应链时用 `docker stop 容器名`。以下启动命令仅供今后重启使用，已有同一链运行时不要重复启动，不另起fusion/master。

定位（仅当前定位未运行时）：
```bash
cd /home/potato/slicer_ugv_ros1 && ./tools/slicer_shell.sh roslaunch slicer_bringup sensors.launch root:=/slicer
```

终端A：
```bash
cd /home/potato/slicer_ugv_ros1 && ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_odom.launch connect_control:=true
```

终端D：
```bash
cd /home/potato/slicer_ugv_ros1 && UGV_SERIAL_DEVICE=/dev/c30d ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_driver.launch config_file:=/workspace/install/share/ugv_bringup/config/slicer_driver_wheel_speed_reference.yaml control_mode:=wheel_speed publish_tf:=false
```

以上启动不release、不发目标。A启动DWA、Guard、Mux；D明确选wheel_speed，默认配置仍是PWM参考模式，不能省略这次选择。若D报错，保留最后几行，不换旧driver重试。

终端B进入操作环境：
```bash
cd /home/potato/slicer_ugv_ros1 && ./tools/slicer_shell.sh
```

本次已只读确认：LIO/scan约10Hz、融合odom/轮odom约19Hz；`odom→base_link`唯一发布者为EKF。运行中provenance为我方wheel_speed，CONFIG nonce匹配且configured/ready为真，MCU tick推进、故障位清零，采样中仅有0x10四零，无PWM回退；Mux仍startup held、teleop不活跃、最终Twist为零。GetPlan-only已成功，未发Action。可以进入下一节；这些结果不代表非零或物理停车已经通过。每次新采集目录换序号，已有目录不会覆盖。

## 2．离地低速（人在旁边操作）

四轮稳固离地，无活动move_base目标及其他操作者输入。B先只执行采集这一行：
```bash
python3 tools/collect_odom_test.py --duration 60 --output evidence/raised-01 &
```
**等终端出现 `RECORDING STARTED: evidence/raised-01 (read-only; no motion authorization)` 后，再单独粘贴下一行。** 若输出路径显示为绝对路径，以实际路径为准；采集未启动或已退出时不发非零，不必等60秒录完才动：
```bash
rosrun ugv_slicer_control control.py release && timeout --signal=INT 5s rostopic pub -r 20 /cmd_vel/nav_raw geometry_msgs/Twist '{linear: {x: 0.05}, angular: {z: 0.0}}' || true; rosrun ugv_slicer_control control.py hold
```
速度行在交互Bash整行执行；release失败不发速度，末尾hold；timeout退出码124表示发布进程窗口结束（含启动/连接时间）；`|| true`只为继续执行hold，其他报错仍须保留并停止。0.05是测试目标、不是已测安全速度，0x10低轮速不保证低PWM。正常：四轮朝前起转、结束停住；通过后才单独执行左转轮速检查（左侧向后、右侧向前）：
```bash
rosrun ugv_slicer_control control.py release && timeout --signal=INT 5s rostopic pub -r 20 /cmd_vel/nav_raw geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.25}}' || true; rosrun ugv_slicer_control control.py hold
```

不转、只抖、方向不符、fault、任何数据停更或hold后仍有运动：立即hold，必要时物理急停/Motor OFF，保持本次记录，不连续重试/自动清错。轮子实际停止须现场观察，不能只看 `/cmd_vel=0`。

## 3．地面短直线（前置条件未满足则不执行）

**须离地前进、纯转与停止均通过，且当前LIO/TF/轮速持续有效。** 独立LIO检查已随 `f4bd9ee` 安装，初始超时0.5s仍需观察现场抖动，不代表物理停稳时间。空旷地，固定车体测量点；先用同样后台采集命令，输出改为 `evidence/ground-01`。复用上面的5秒0.05直行段，量真实起止位移、横偏、停零到停稳余程。不能由 `0.05×5` 充当测量。

短直线正常后才做odom导航：
```bash
rosrun ugv_navigation goal.py plan --relative --x 0.5 --y 0 --yaw 0
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation goal.py send --relative --x 0.5 --y 0 --yaw 0 --timeout 30
rosrun ugv_slicer_control control.py hold
```
plan只求路径、不发目标；无路径先查scan/TF/costmap。send的Ctrl-C取消本目标；另开终端可 `rosrun ugv_navigation goal.py cancel`，并用hold停链。Action成功不等于距离/停车合格：当前xy容差0.15m。结束至少观察5秒再动下一次，保存测量与最终状态。

## 4．填写测量并导出

采集退出后得到 `motion.bag`、`manifest.json`、`measurements.template.json`。复制模板填写，未知值保留 JSON 的 `null`，不要写空值或猜测；真实位移和停车距离均以同一固定点实测，区分人工搬车与自主运动：
```bash
cp evidence/ground-01/measurements.template.json evidence/ground-01/measurements.json
python3 tools/analyze_odom_test.py evidence/ground-01 --measurements evidence/ground-01/measurements.json
```
保持数据目录整体，退出容器后仅导出本次目录，例如 `scp -r cup-jetson:/home/potato/slicer_ugv_ros1/evidence/ground-01 ./evidence/`。分析比较各自起点的位移增量，不比较不同原点的绝对坐标；没有真实测量或缺停止尾段就报告unknown，不补写通过。无需录点云、图像或整幅costmap。
