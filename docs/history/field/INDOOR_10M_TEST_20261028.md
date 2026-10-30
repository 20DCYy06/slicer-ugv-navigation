# 2026-10-28 室内 10m 试验操作单

首轮仅 **0.20m/s**；DWA、0x16 body speed、EKF Q(wz)=0.2、Guard/Mux 接线沿用，yaw/xy 容差为 **0.12rad/0.10m**，角速上限0.25rad/s、线/角加速度0.2m/s²/1rad/s²不变。全局 rolling costmap 为 **24×16m、0.05m/格**，局部图与障碍参数不变。0.30m/s仅为未启用后备，不在本轮自动切换。

**驱动启动、release/hold与运动由现场用户执行；助手默认只读，获本组明确确认后才可发已冻结的Action。** 保留 `nav_raw → Guard → Mux → 唯一 driver` 接线，不向 `/cmd_vel` 发布，不重复基础底盘测试。

1. **准备与启动（车保持 held，无活动目标、目视停稳）。** 在Jetson独立工程现有ROS shell中操作；不另起master，不重复启动driver、Mux或导航。若需要新开shell：

   ```bash
   cd /home/potato/slicer_ugv_ros1
   ./tools/slicer_shell.sh
   ```

   用户先停旧导航，再将导航与Guard一起重启，使其重新读取本轮上限；勿叠加实例，Mux启动应held。`control_config`沿用Guard/Mux配置，实际driver仍由用户保持0x16：

   ```bash
   roslaunch ugv_bringup slicer_odom.launch local_planner:=dwa connect_control:=true control_config:=/workspace/install/share/ugv_bringup/config/slicer_driver_wheel_speed_reference.yaml global_costmap_config:=/workspace/install/share/ugv_bringup/config/indoor_10m_020.yaml
   ```

   另一个ROS shell回读：

   ```bash
   rosrun ugv_slicer_control control.py state
   rosrun dynamic_reconfigure dynparam get /ugv/move_base/DWAPlannerROS
   rosparam get /ugv/move_base/global_costmap
   ```

   回读确认DWA的`max_vel_x=max_vel_trans=0.20`、yaw/xy=0.12/0.10、角速/加速度，以及全局图24×16、resolution=0.05；核对0x16、Q(wz)=0.2、Guard逻辑及接线。以实际运行值为准。

2. **静止看障碍和通道。** WSL执行 `./tools/rviz_wsl.sh navigation`。检查scan、两层costmap、footprint和TF；人工查桌腿、低横梁与车体外廓，确认标记稳定、footprint方向/覆盖正确，通道与终点有实测余量。24×16长边沿 **odom x轴**，车头偏离时10m目标可能超出短边；必须查目标及全路径在图内并有边界余量。

   障碍标记范围约3m，静止不能一次确认远处全部桌腿；人员先查整段，行进中随rolling图更新分段观察。未知区求出路径不保证安全通行；远处未观测、腿/横梁漏检或余量不足时暂停，不能清图或放宽Guard。

3. **先采集，再冻结并预览一次目标（仍held）。** 先由用户调好起点；采集与预览两个ROS终端设置相同`ugv_run`，每组换全新目录。等`RECORDING STARTED`才预览/发目标；600s包含等待，发目标前确认仍在录。

   ```bash
   ugv_run=evidence/indoor10m-20261028-01
   python3 tools/collect_odom_test.py --namespace ugv --navigation --duration 600 --output "$ugv_run"
   ```

   预览终端（目录名与采集一致）：

   ```bash
   ugv_run=evidence/indoor10m-20261028-01
   python3 tools/preview_forward_goal.py --distance 10 --output "$ugv_run/fixed-goal.json"
   ```

   该只读工具按现有`goal.py --relative --x 10 --y 0 --yaw 0`转换逻辑采样一次，保存绝对`odom`目标、检查图边界并调用GetPlan，**不发Action**。检查非空plan、空闲终点及避障余量；只复用其打印的真实绝对send命令，禁止另跑相对send。车姿/odom/TF异常或换目标须held后重建本组确认。默认不录整程点云；漏检排查可另开20s静态包，显式加`--perception`。

4. **本组确认后只发一次。** 用户先确认通道、人员、急停、输入与采集均就绪，再执行release和同一冻结目标的Action；超时/失败不自动重发，不启用0.30后备。

   ```bash
   rosrun ugv_slicer_control control.py release
   ```

   随后复制本次预览工具打印的`goal.py send --frame odom ... --timeout 180`整条绝对命令，保留真实x/y/yaw，不用示例坐标、不重新relative采样。起点尚在调整时止于准备，不发目标。

   连续看车体、障碍与Guard；漏检、擦碰风险、漂移、异常修正、陈旧输入或fault立即停止。正常自停后录至少5s尾段，用户hold，再Ctrl-C采集，等`CAPTURE COMPLETE`。保存本次bag/manifest/rosbag.log、冻结目标与回读；记录Action结果、实际位移/朝向、终点残差、最小间隙（未测留空）和干预。`SUCCEEDED`不等于精度/避障通过，超时、取消、重发、移障/接管如实记。

需要停止时不等尾段；用户可先hold并取消，再核对状态，异常立即用现场物理急停/Motor OFF，软件命令不代替目视停稳：

```bash
rosrun ugv_slicer_control control.py hold
rosrun ugv_navigation goal.py cancel
rosrun ugv_slicer_control control.py state
```
