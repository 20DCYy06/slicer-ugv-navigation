# 继续修复验证（2026-10-24，进行中）

后续固件阶段：用户确认21日`c30d_vet6`，本轮实际全片双读匹配21日记录；固件身份不再完全未知。仅三处实际dt累计的修复及烧写结果见[C30D记录](../firmware/C30D_TIMEBASE_2026-10-24.md)。本轮授权不包含自动部署或启用新导航/轮速driver，A/B/C验收仍分别保留。

本轮基于 `23728e8` 基线和 `ae4ced2/c467c94` 审查修复继续。旧项目和队友树只读；2026-10-24用户明确取消新工程禁止部署限制。助手可同步和构建独立新工程、只读观察和采集本次试验，真实驱动启动、release/hold及运动目标由用户现场执行。没有将软件模型算作实车结果。

后续边界更新：用户最新提供的本轮指令禁止Jetson部署、真实driver与实车目标；因此本文后述的`wheel_speed`新改动仅完成本地验证。上述授权和ARM64记录属于此前已完成的现场工作，不用于放行本次部署。

## 当前现场事实

- 定位/雷达已由用户启动。真实 `/odom`、`/scan` 约10Hz，`odom→base_link` 为已有EKF发布；前方0.5m的GetPlan返回22点，无Action目标。
- 新driver已由用户启动，实际导入新工程安装目录连续PWM映射，`mock=false`。本轮识别C30D为 `/dev/ttyCH343USB1`，USB `1a86:55d4 / 5CF0016377`；设备名不能跨重启假定。
- 用户误启动原 `start_ros.sh` 时，串口独占拒绝旧进程，但同名ROS注册使新driver退出。原记录保留；用户只重开新driver终端D。没有修改原driver或队友树。
- 之后的一次0.5m目标以ABORTED结束，前置release报 `manual intent must have the unique mux subscriber`。被动rosbag订阅者与真实Mux同时订阅意图，触发了客户端错误的唯一订阅者检查；Mux仍startup held，最终速度和发送PWM全零。这次不能用于评价驱动运动效果。
- 该客户端检查现改为要求实际Mux存在于订阅者中；唯一状态发布者、非锁存状态、实例/代次、目标空闲、新鲜度及本次ACK仍保留。WSL和ARM64各16项unit及实际ROS传输回归通过。新客户端源文件与安装文件SHA256均为 `48807d8d4623e3e593a646c983405eda8c6194e3127e754cffb84ab191390b1d`；无需重启A/D。
- 客户端修复后，用户贴出的两个relative0.5m目标均SUCCEEDED、自行停稳；用户确认尺量合计83cm、无hold/急停/挪动，车体朝向偏左。同步包`straight-client-fixed.bag`中精确匹配target(.261773177,.019534026)及(.699710612,.022937871)；停后odom相对前移约.440m/.406m，合计约.846m，与尺量接近。该包另含一个更早目标，单列不并入83cm；两组之间曾出现零指令期间的定位/编码器变动，不推断为自主驱动。
- 用户两段的odom成功时位移约.390m/.373m，成功后至+5s仍增加约5.1cm/3.3cm。当前xy容差.15m允许提前终止，不能把83cm合计解释为每次超调33cm。LIO与融合数据异步，增量不等于精确地面制动距离。
- 两次串口成功全零分别在SUCCEEDED接收后约15.8ms/11.4ms，新鲜MCU接受零且PWM timer ticks全零约67.6ms/21.8ms；编码器末次变化约496ms/606ms，轮速于约443ms/447ms持续进入软件静止阈值。没有随后非零PWM纠偏。支持“零参考后仍有余动”的观察，不证明具体电桥制动方式。
- 两段最终相对yaw分别约+2.28°/+2.43°，右侧位置偏移约1.31cm/.25cm。车体朝向偏左与位置横移须分开描述。用户随后执行hold，Mux接管确认、generation=1；准备同步并隔离验证DWA。有效转向及实物障碍响应尚未验收。

## 已定位与正在落实的软件问题

| 环节 | 证据与处理 | 验证边界 |
| --- | --- | --- |
| driver速度到PWM | 旧映射在极小非零转向处有跳变；新driver调用连续映射，纯直行/纯转端点、零与反向先零及Watchdog策略保留。 | 有单元和协议诊断测试；实际速度、起转门槛和制动距离未标定。 |
| TEB侧向目标 | 五组fresh隔离对照表明起始需侧转时仍给出毫米级倒车局部解，Guard按禁倒车策略拒绝整条指令；初始朝向已对准路径的对照成功。 | 使用真实move_base/TEB/Costmap/Guard/Mux与理想差速模型，未模拟电机。 |
| 标准DWA候选 | 原始角轨迹分辨率0.1使短距低角速候选仅采到初始姿态。单独改前视点0.325→0.1仍失败；仅改角分辨率0.1→0.025后，七组前进/纯转/左右侧方/斜前目标和短距复现共8次SUCCEEDED；实际所选DWA参数接Guard后再验+.2m成功。 | 限速、容差、外廓、unknown策略不变；DWA没有TEB的min_obstacle_dist参数，同一膨胀不保证相同间隙，实物响应需验。 |
| Guard插件参数 | 原实现硬读TEB树；生产集成改为读取所选标准插件的速度/加速度参数，与已有源限制取交集。 | 不借未运行的TEB参数，不添加补偿floor，不放宽禁倒车。 |
| 观察与授权冲突 | 新客户端允许被动意图订阅者；只有新鲜、唯一、正确实例的Mux状态可确认授权。 | 录包/rostopic不能代替Mux；缺失Mux、冲突状态发布者、接管/重启和急停拒绝语义保留。 |

## 可核对的软件证据

失败与成功均保存在本工作区 `.test-results/`，未覆盖历史记录。

- `complete-arm64-initial.log`：新driver诊断与环境接线初版ARM64完整构建、77个JUnit条目、安装通过。
- `complete-selected-planner-suite.log`及`run-Tmb7xNVf`：本地DWA集成完整构建、86个JUnit条目、0错误/失败/跳过、安装通过。DWA/TEB Guard各11项真实ROS传输检查通过。
- `complete-selected-planner-arm64-suite.log`及车端`run-aM0QNrNA`：提交`2177c76`同步独立新工程后，原生ARM64相同完整86条目、0错误/失败/跳过、安装通过。尚未由助手启动实车目标。
- `complete-intent-observer-unit-red.log`、`complete-intent-observer-ros-red.log`：新增观察订阅者后原客户端失败。
- `complete-intent-observer-unit-green.log`、`complete-intent-observer-ros-green.log`、`complete-intent-observer-arm64-green.log`：修复后的两端验证。
- `complete-dwa-20261024-obstacle-chain/summary.json`：独立fresh master的四类全链软件场景全部通过：取消为PREEMPTED且后续1s最终输出零；扫描丢失约.507s停零、恢复扫描后同一Action续动；动态墙体进入两图并按真实ABORTED停零；有限矩形绕行15.51s后真实SUCCEEDED、误差.13755m/.000785rad、未观察到离散积分足迹相交。理想模型没有PWM、死区、惯性或碰撞夹停，不能证明实车障碍响应。9项几何回归及独立只读审查通过。
- `complete-forward-summary.json`：TEB五组对照；`complete-dwa-20261024-angular-resolution/summary.json`：DWA候选与失败基线、单参因果实验及源码行的索引。
- `complete-live-before-client-fixed-trial.json`：用户下一次运动前的只读快照，Mux保持、最终零、driver新鲜及真实导入路径。
- `complete-straight-client-fixed-analysis.json`：本次现场包逐goal分析；`complete-live-after-client-fixed-trials.json`：自行停稳后的只读快照。MCU接受命令和timer ticks均不是地面停车证明，实际自行停稳来自用户现场确认。
- `complete-motion-analysis-*.log`：离线取证工具回归。分析器关联同一goal_id、实例和接收时间，缺失数据输出unknown；MCU接受命令不等于已执行，轮式里程计静止也不等于地面已停稳。
- `wheel-speed-wsl-full.log`及`run-g5QFVzHk/exit-code`：新增可选轮速模式的本地完整构建、101个JUnit条目、0错误/失败/跳过、安装通过，其中driver39项。默认仍为PWM；新版本未ARM64验证或实车部署。`wheel-speed-analysis-green-noetic.log`：离线分析器20项通过。保留对应RED日志，未使用电机PI模型证明实车能力。

软件probe只在源码工作区的隔离容器运行，例如：

```bash
./tools/noetic.sh roslaunch ugv_navigation probe_dwa_safety.launch \
  scenario:=finite_obstacle output:=/workspace/.test-results/my-finite-obstacle.json
./tools/noetic.sh python3 tools/analyze_motion_bag.py 本次已采集的本地.bag
```

probe输出中的`scenario_pass`、实际Action状态和原始轨迹才是功能证据；进程退出不等于到点成功。`cancel`、`scan_loss`、`blocked_wall`、`finite_obstacle`每组必须新开隔离容器/master。TEB/DWA对照probe保留用于复现失败，不能当正常实车启动入口。

## 下一步

直行取证已完成；纯转在新电池下实际28‰无有效转动，临时上限内诊断71‰也未起效。用户已hold，最低角速恢复原.05。停止导航参数试探，转向[底盘速度接口核查](BASE_CONTROL_FINDINGS.md)：现场原/新driver均走0x11开环PWM，本地现已补上可选0x10/0x12接口，但尚未接通实际MCU验证。先核对实际固件、符号与轮速闭环，之后再做有效转向和实物障碍响应。软件模型绕障通过不代表该底盘已能自主绕障。不开展RTK、自动恢复、无线集成或额外运行管理。
