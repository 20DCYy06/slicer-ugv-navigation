# 2026-10-27 到点容差与固定箱体绕行短单

**先做 yaw 单变量对照，再做 xy；绕障目前仅准备，须现场另行确认运动条件。** 沿用原 DWA、EKF Q(wz)=0.2、0x16、GlobalPlanner、12m 全局/4m 局部地图与原 Guard/Mux；速度上限仍为0.125m/s、0.25rad/s。底盘由现场启动；每组运动先确认现场安全条件，再按本轮明确授权执行。以下为现场手动命令参考。

车端进入独立工程现有 ROS shell：`cd /home/potato/slicer_ugv_ros1 && ./tools/slicer_shell.sh`。先确认 `control.py state` 为 held、无活动目标、车体停稳、输入新鲜且无 fault；保持现有 raw→Guard→Mux→唯一 driver 接线。

```bash
ugv_cfg="$(rospack find ugv_bringup)/config"
dwa_srv=/ugv/move_base/DWAPlannerROS
rosrun dynamic_reconfigure dynparam get "$dwa_srv"
```

三个 YAML 是该动态配置服务的**平铺补丁**；运行中的 DWA 必须用 `dynparam load` 触发回调，单独 `rosparam load` 不生效。原 `dwa.yaml` 启动值仍为 yaw=0.25、xy=0.15；在线改值不持久化，重启导航后先 held、重载选定补丁并 get 核对，不能假定保留。

每阶段载入并核对参数后，单独启动采集并等 `RECORDING STARTED` 再发目标，输出目录每次换新；结束后录足至少5秒静止尾段再 Ctrl-C，确认 `CAPTURE COMPLETE`：

```bash
python3 tools/collect_odom_test.py --navigation --duration 600 --output evidence/nav-precision-$(date +%Y%m%d-%H%M%S)
```

每次载入均在 held、无活动目标时执行，随后 `rosrun dynamic_reconfigure dynparam get "$dwa_srv"` 留存实际 yaw/xy；停止阈值、速度、加速度和其他 DWA 参数均不改。

| 顺序 | held 时载入 | 本阶段目标与固定项 |
|---|---|---|
| A 原值 | `rosrun dynamic_reconfigure dynparam load "$dwa_srv" "$ugv_cfg/dwa_goal_baseline.yaml"` | 左/右各45°；yaw=0.25、xy=0.15 |
| B 仅 yaw | `rosrun dynamic_reconfigure dynparam load "$dwa_srv" "$ugv_cfg/dwa_goal_yaw_012.yaml"` | 同样左/右各45°；仅 yaw→0.12，xy=0.15 |
| C xy 基线 | 保持 B 验证通过的 yaw；若 B 回退则保留0.25；确认 xy=0.15 | 相对前进0.5m、yaw=0 |
| D 仅 xy | `rosrun dynamic_reconfigure dynparam load "$dwa_srv" "$ugv_cfg/dwa_goal_xy_010.yaml"` | 同样前进0.5m；仅 xy→0.10，C/D 的 yaw 必须相同 |

A/B 在同一空旷区使用同一车体角度标记；C/D 使用同一车体测距点与方向，不重置 odom。每条目标独立开始，正常自停后录5秒、执行 `control.py hold` 再进行下一条；每次 release 由用户确认后执行，下面三条 send **按阶段选一条，不能整块粘贴执行**：

```bash
rosrun ugv_slicer_control control.py release
rosrun ugv_navigation goal.py send --relative --x 0 --y 0 --yaw 0.7854 --timeout 60
rosrun ugv_navigation goal.py send --relative --x 0 --y 0 --yaw -0.7854 --timeout 60
rosrun ugv_navigation goal.py send --relative --x 0.5 --y 0 --yaw 0 --timeout 60
```

记录每次固定目标、Action 结果、真实转角/位移、odom 残差、自停与融合速度拖尾、是否抖动/干预。yaw 对照完成并选定已验证的 yaw 后才进入 C/D；偏差改善且可靠自停才能保留候选，`SUCCEEDED` 本身不等于精度验收。若超时、反复修正、拖尾恶化或 fault，停止并保留失败，不自动重发。

回退 yaw 或整组：held 后载入 `dwa_goal_baseline.yaml`（恢复0.25/0.15）。仅回退 xy：`rosrun dynamic_reconfigure dynparam set "$dwa_srv" xy_goal_tolerance 0.15`，保留当时已验证的yaw；每次 get 核对。未通过的候选不写成新启动默认值。

**固定箱体绕行准备（容差对照结束后再安排）。** WSL 用现有 `./tools/rviz_wsl.sh navigation` 看 scan、两层 Costmap、GlobalPlanner 与 DWA local plan；使用本轮最终验证的容差，另开一份上述 `--navigation` 采集。

1. 车保持 held，前方空旷；先对前方2m求无障碍基线，记录输出的固定 odom 目标和全局路径：

   ```bash
   rosrun ugv_navigation goal.py plan --relative --x 2 --y 0 --yaw 0
   ```

2. 在这条路径前方约1m处放已确认能稳定检测的不透明箱体，记录箱体尺寸、离车距离，两侧留足车身通道。静置至少5秒；确认 scan 与全局/局部 Costmap 均稳定标记箱体，目标位置空闲；held 中重跑同一 plan，确认新路径绕开箱体且可到目标。车未移动时复用相对目标；若车姿改变，先重新建立整组基线。
3. 无标记、无可行绕行路径、目标被占或输入异常则止于准备，不清图、不放宽 Guard。现场确认后，用户 release 并**只发一次**以下 Action；箱体整段保持固定，不移动箱体让车续行：

   ```bash
   rosrun ugv_slicer_control control.py release
   rosrun ugv_navigation goal.py send --relative --x 2 --y 0 --yaw 0 --timeout 120
   ```

4. 通过须同时具备：同一 goal_id、全局/局部规划与实际轨迹对应、用户目视确认车从箱体一侧绕过且无接触、无取消/重发/手控、最终到点自停。只停住、移箱后续行或换目标成功，均不算绕行通过。记录实测最小间隙、是否干预；未测量留空。

需要停止时不等尾段，立即由用户执行；异常用现场已有物理急停/Motor OFF，软件零不代替目视停稳：

```bash
rosrun ugv_navigation goal.py cancel
rosrun ugv_slicer_control control.py hold
rosrun ugv_slicer_control control.py state
```

交付本次新目录的 bag、manifest、rosbag.log 与参数回读/人工观察。原有“遇人停车后续行”只支持避让停车，背景见 [EKF 实车结果](EKF_IMU_FIELD_20261027.md)；不作为此次固定箱体绕行通过证据。
