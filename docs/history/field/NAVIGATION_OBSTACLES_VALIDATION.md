# 本轮离线交付与验证（2026-10-26）

现场短单：[NAVIGATION_OBSTACLES_FIELD.md](NAVIGATION_OBSTACLES_FIELD.md)。本页记录部署前的离线准备阶段：仅修改 WSL 工程，没有 SSH/部署、连接串口、启动实物驱动、发实车速度/目标或控制权指令。未修改 MCU、EKF、RTK、Guard/Mux/Watchdog 逻辑或规则。测试只在无设备、`--network none` 的 Noetic 容器内运行。用户随后授权的车端同步与 ARM64 构建结果见[部署补记](NAVIGATION_OBSTACLES_DEPLOYMENT.md)，不将本页的软件结果升级为实车结果。

## 具体改动

- `slicer_planner.yaml`：仅显式 TEB 时复用队友现用路径/时间权重 `1.5/.2`，保持共同速度、加速度、footprint、避障间距；`teb.yaml` 补充标准 TEB 软约束说明，DWA 默认和原 TEB 基线保留。已有 `local_planner:=dwa|teb` 足够，不新增导航入口或管理器。
- `small_obstacle_projection.launch`、同名 YAML：标准投影的可选旁路，仅发布诊断 scan，不接导航；实际感知阈值没有改，因为尚无水瓶在哪一级消失的证据。
- `small_obstacle.rviz`：过滤前后点云、scan、地图、footprint、两种 local plan；原 `odom_navigation.rviz` 补充 TEB local plan。均不含目标/速度发送工具。
- `collect_odom_test.py`：复用原只读采集器，增加 `--navigation` / `--perception`、显式感知参数白名单快照和录制窗口开始/结束提示。点云默认不录，仅静态短采开启；没有新增运行状态机或控制发布。
- 新增两个 TEST_ONLY 检查脚本，增强原采集器检查和单元测试；补充感知、TEB、速度说明与现场短单。`ugv_bringup/package.xml` 增加标准投影依赖，原目录安装规则覆盖新配置。

## 实际执行结果

| 项目 | 结果 | 证据目录（工作区相对路径） |
| --- | --- | --- |
| Catkin 构建与安装 | 通过 | `.test-results/navigation-obstacles-20261026/catkin-{build,install-adapted}.log` |
| 两规划器 launch、0x16 driver 仅参数解析、诊断/RViz安装一致性 | 5项通过；未启动 driver | 同目录 `launch-install-validation.json`、`launch-install-adapted.log` |
| 控制链未改 | 6个核心文件与上轮收取源码逐字节相同 | `launch-install-validation.json` |
| 真实原过滤脚本+标准投影 | 11/11通过，含0.5/1/2m低点、高度/距离窗、arm掩码、车身NaN、最近点、空格inf | `small-obstacle-projection/result.json` |
| 既有 DWA 12m 回归 | 5/5通过，含mark/clear、(1,1)/(3,4) GetPlan、重规划/取消；局部34项参数不变 | `dwa-rolling-regression/result.json` |
| 采集器单元检查 / Noetic tools 全套 | 16项 / 71项通过 | `.test-results/nav-capture-20261026/` |
| 新采集选项的真实 ROS 消息录入/读回 | 自动结束、Ctrl-C两例通过；PointCloud2/Path/OccupancyGrid、参数脱敏、订阅与bag收尾均检查 | `ros-options/result.json`（上行目录内） |
| DWA 同场景动作对照 | 前进/原地90°通过；后方目标振荡ABORTED；原地180°单独通过 | `dwa-comparison/`、`dwa-turnaround/` |
| TEB旧基线 / 队友偏好适配版 | 原地转向有限病例通过；其余出现负vx，严格Guard兼容失败；最终仅2/6通过，工具退出码1 | `teb-baseline-v2/`、`teb-final/`、`teb-turnaround/`、`teb-slicer-adapted/` |

上述表中导航、投影和动作均使用合成传感器/理想运动模型。未运行 FAST-LIO 或真实融合，未测水瓶检测率、物理绕障、底盘跟随或制动。TEB 动作成功不等于严格 Guard 下成功，障碍场景没有额外的物理碰撞裁决，不能写成实车绕障已过。

## 失败保留与明确缺口

1. 最终 TEB 适配版6例均返回Action成功，但前进/侧向/后方/障碍过程含负vx，因此4例严格兼容失败；DWA后方目标20秒振荡中止。详细数值见 [TEB说明](TEB_ODOM_NOTES.md)，没有以调大权重、截负值或改Guard凑通过。
2. 小障碍尚未取得专门的原始点云→各级scan同时记录，不能定位实物水瓶消失层。静态观察顺序与最小确认方法见 [感知说明](SMALL_OBSTACLE_FINDINGS.md)。后三个过滤/几何节点从文件直接加载的值不全在ROS参数中，仍需附本次明确配置相关节和指纹，不把参数文件路径当完整有效配置。
3. 测试工具初版 footprint 字符串/数组比较误报、完成后多余 cancel 的客户端错误均已修正，原目录/日志保留；修正后最终日志无 ROS ERROR。另保留初次插件检查缺少 `rg`，及一次性 launch 校验辅助脚本挂载/日志文件名错误；后续相应检查已实际通过，不把工具错误归因于生产链路。
4. 速度核对依据已有运行快照、源代码和已采日志，本轮没有再次连接车端核对新状态。当前比较配置仍为 `.125m/s`、`.25rad/s`，分别不代表物理巡航速度和实测可达角速；MCU源码极限也不是新刷写身份或性能验收。

回退：取消/hold后，导航入口改回 `local_planner:=dwa`；关闭旁路投影仍保留原 `/scan`。无需改定位或固件。实车测试先静态感知，再原地/后方/二维对照，最后仅用稳定可见障碍绕行并测实际停车。
