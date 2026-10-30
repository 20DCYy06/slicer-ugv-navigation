# 2026-10-27/28 到点精度、速度与固定箱体绕行

本轮保留 DWA、0x16、EKF Q(wz)=0.2、Guard/Mux/Watchdog。未融合原始 gyro，未改 MCU、速度上限、加速度、安全规则或 DWA 停止阈值。以下结果仅对应本次场地和已采试验。

## 到点容差单变量对照

先仅收紧 yaw，左右各45°；随后固定 yaw=0.12，仅收紧 xy，各走一次0.5m。所有目标均 SUCCEEDED、正常自停；左右转后未手动调回，直行两轮无人工干预。

| 对照 | 原值 | 候选 | 实际结果 |
|---|---|---|---|
| yaw | 0.25rad | 0.12rad | 停止后5秒 odom 航向残差：左11.35°→6.34°，右10.65°→5.22° |
| xy，yaw固定0.12 | 0.15m | 0.10m | 现场实测前进37→41.5cm；停止后5秒 odom 距离残差13.24→8.64cm |

yaw 候选右转曾有一次停顿/修正，用户估计最后相对最初朝向多转约1～2°；不能写成完全无修正。左右转的 odom 残差不是独立量角真值。xy 两轮软件相对起点右偏均约2.5cm，用户也观察到右偏，未据此认定右偏已解决。

左右各一轮和直行各一轮已支持本次容差改善，尚不代表多场地重复精度。yaw 候选左转的瞬时 ActionGoal 包未录到，但 accepted current_goal、唯一新 ACTIVE ID 与同 ID 的 SUCCEEDED 结果完整，已在分析中显式标明恢复来源。

当前实车运行参数为 yaw=0.12、xy=0.10；源 `dwa.yaml` 仍保留0.25/0.15。动态修改重启后不保留。运行中须用 dynamic_reconfigure，不能仅 rosparam load：

```bash
ugv_cfg="$(rospack find ugv_bringup)/config"
rosrun dynamic_reconfigure dynparam load /ugv/move_base/DWAPlannerROS "$ugv_cfg/dwa_goal_yaw_012.yaml"
rosrun dynamic_reconfigure dynparam load /ugv/move_base/DWAPlannerROS "$ugv_cfg/dwa_goal_xy_010.yaml"
```

以上在 held、无活动目标时执行。本次换电后已重新载入并回读。整组回退：载入 `dwa_goal_baseline.yaml`；仅回退xy、保留已验证yaw：

```bash
rosrun dynamic_reconfigure dynparam set /ugv/move_base/DWAPlannerROS xy_goal_tolerance 0.15
```

## 速度瓶颈

复用既有 `(1,0.5)` 与 `(2.5,-3)` 两份二维导航 bag：

- 长程 DWA 平移请求约63.8%的时间接近0.125m/s上限；MCU 稳态轮速反馈中位约0.126m/s，没有显示巡航速度被 MCU 持续压低。轮速反馈不是独立地面速度真值。
- 长程 Action 45.10秒中，最终命令平移36.16秒、纯转6.82秒、全零2.12秒。这些是请求分段，不能等同真实运动时间。
- Guard 占用停止累计约0.425秒；Mux 在 Action 期间保持 nav，没有人工保持造成持续限速。
- 低角速仍有独立响应不足证据：长程 MCU 纯转目标绝对值0.075～0.125rad/s的125帧中，约77.6%的 gyro Z 反馈接近零。只能定位到目标已送达而反馈未充分跟随，不能直接判定机械或 MCU 内部原因，也不能据此猜新的调参值。

本轮不提高0.125m/s、0.25rad/s上限。速度分析见 机器结果（原归档未随仓库收录：`../../.test-results/nav-precision-20261027/speed-budget.json`） 及 说明（原归档未随仓库收录：`../../.test-results/nav-precision-20261027/speed-budget-result.txt`）。

## 固定箱体绕行

换电后重新建立 odom 基线。用户将宽30cm、高40cm箱体固定放在车前约1m，目标固定为 odom `(1.99650,-0.01181)`、yaw≈0.00062rad；箱体放置前后车辆未移动，仅有约1mm的 odom 抖动。

| 静止预规划证据 | 空场 | 放箱体后 |
|---|---:|---:|
| 正前±20° scan 最小距离 | 3.381m | 0.992m |
| 固定前方ROI的高占用格，global/local | 0/0 | 72/66 |
| 同一目标全局路径长度 | 2.000m | 2.295m |
| 最大右侧偏移 | 0.010m | 0.481m |

ROI 为初始朝向前方0.6～1.7m、左右±0.4m；高占用指 OccupancyGrid 值≥99，不等于物体实际面积。两层目标格均空闲。快照与完整地图保存在 本次静止证据（原归档未随仓库收录：`../../.test-results/nav-precision-20261027/static-obstacle02`）。

本次唯一 Action 返回 SUCCEEDED，随后 hold。用户明确确认完整绕过固定箱体、无接触、自行停稳、无人工干预。因此记录为**一次静态箱体绕行成功**，与之前遇人停车、走开后续行分别记录；最小物理间隙未测。用户观察转向仍有卡顿，尚未解决。

该目标实际持续36.60秒。odom 逐点累计轨迹长约2.534m（含采样抖动），最大右偏0.398m；终点及停止后5秒的距离残差分别2.15、1.87cm，5秒航向残差约−0.0904rad。它们是软件定位量，不替代外部测距。Action 期间715条Mux状态均为nav，未见manual_hold或emergency；916条body反馈未见fault/IMU失效，ACK目标与已发送0x16一致。A轮连续转动本次未单独获得现场确认。

规划与实际 odom 轨迹图（原归档未随仓库收录：`../../.test-results/nav-precision-20261027/static-obstacle02/trajectory-plans.png`） 使用冻结空场起点坐标；与实际 Action 起点口径约有2.4mm的最大横偏差异。GlobalPlanner在Action内有37条规划；DWA global/local plan各292条，最后约7.39秒未见新的DWA路径消息，转向命令仍有更新，不能宣称local plan全程持续发布。

转向卡顿的最小定位：约9.26秒纯转目标中，gyro接近零累计约3.09秒；4段持续至少0.3秒的低角速静默区间内，DWA raw与Guard仍持续给出非零转向。例如目标开始后约31.98～32.94秒，MCU目标中位约−0.1015rad/s，gyro Z约−0.002rad/s、轮速中位为零。Guard仅起始约0.0054秒stale，其余无hold。这里支持“已送达低角速目标，底盘反馈间歇未跟随”，尚不能区分MCU控制律和机械因素；不调整安全规则或猜补偿值。详见 本次分析（原归档未随仓库收录：`../../.test-results/nav-precision-20261027/static-obstacle02/static-obstacle-analysis.json`）。

## 交付与剩余问题

- 三份容差补丁已完成 ARM64 构建/安装；八份受保护原文件哈希未变。离线分析器33项测试通过。
- 本轮六份完成的 bag 均已取回 WSL：yaw基线左/右、yaw012、xy015、xy010、`static-obstacle-20261027-02`。换电前 `static-obstacle-20261027-01` 不作为本次绕行运动证据。
- yaw对照（原归档未随仓库收录：`../../.test-results/nav-precision-20261027/yaw-comparison.json`）、yaw覆盖来源（原归档未随仓库收录：`../../.test-results/nav-precision-20261027/yaw012-coverage.json`）、xy对照（原归档未随仓库收录：`../../.test-results/nav-precision-20261027/xy-comparison.json`） 区分软件测量、用户实测及缺失证据。
- 本轮未解决转向卡顿和略右偏，未验证提速收益或复杂动态绕障。继续优化应先用已录速度链确认卡顿发生层级，不能把一次绕行成功推广为所有障碍均可靠通过。

具体启动、载入、停止和采集命令见 [现场短单](NAV_PRECISION_OBSTACLE_TEST_20261027.md)。
