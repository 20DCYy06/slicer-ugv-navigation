# Slicer odom 接线验证 · 2026-10-23

当前实车结论：**此前样本证明真实输入与ROS接线可用，直行停点精度、有效转向和绕障尚未验收通过。** 本次修复未部署、未连接硬件；以下历史软件通过不替代新版本验证或实车结果。

## 独立审查后的本地控制修复

依据[独立审查报告](../独立审查报告_Slicer_ROS1_2026-10-23.md)，本轮只在新工程本地实现与隔离验证，不修改旧项目或队友树，不启动真实driver、不解除实车保持、不发实车目标或非零速度。历史报告及现场事实保留，不将本次代码变更写成已部署修复。

原driver会把自己的scripts置于导入路径首位，调用原`control_core.wheel_pwm`；新Mux只转发Twist，旁边已有连续函数没有进入原driver。新工程受控driver副本实际调用`control_core.wheel_pwm → slicer_manual.wheel_pwm`，保留纯直行/纯转端点、限幅、立即零、反向过零、Watchdog、遥测保护和轮速接口。独立入口为`roslaunch ugv_bringup slicer_driver.launch`，导航launch不自动启动它；启动日志及`/slicer_c30d_driver/provenance`参数记录实际配置与导入路径/hash，晚订阅诊断也可查询身份。旧driver未替换时，原导入缺口仍在。

唯一串口所有者通过`/slicer_c30d_driver/diagnostics`的标准`DiagnosticArray`记录命令接收、发送PWM/序号和接收遥测，带ROS stamp及主机单调时钟。发送PWM是permille；回报`pwm_timer_ticks_abcd`是原始timer ticks。`accepted_command_seq`只表示固件接受命令，不表示电桥或车体已经执行，不能把`ready=true`或主机write成功当执行证明。无需第二个串口读写进程。

Guard在禁倒车时整条拒绝负vx请求并输出零/明确hold_reason，原始纯转仍允许；不再将倒车圆弧悄然改成纯转。这只修复允许动作的语义，不保证侧方目标前向进展。TEB倒退正上限、GlobalPlanner组合、unknown、外廓、角速、到点容差和固件本轮不改。连续PWM不保证约71‰纯转能起转，也不保证33cm超调消失。

本轮验收分别检查真实driver main及SLC2编解码走假串口、真实Guard节点、只执行Guard/Mux后输出的TEST_ONLY非全向模型，以及安装后的导入路径。精确计数及证据见[本次修复验证](REVIEW_FIX_VALIDATION.md)，历史32条目不覆盖本次新增执行路径。所有软件模型/假传输只在无设备、`--network none`、独立master的容器运行。

本轮隔离控制链特征检查（原归档未随仓库收录：`../../.test-results/guarded-characterization.json`）让真实TEB → Guard → Mux后的最终Twist驱动理想模型，不含driver、电机或地面响应。前进0.5m为state3、模型末端误差约0.14423m，同位置转45°也为state3，两段无负raw；侧方0.2m和0.5m均为state4 oscillation、无平移，各213条负raw被整条拒绝。**侧方前向规划仍未解决**，安全拒绝有效不能写成导航问题已经全部修复。纯转软件成功也不能证明实际71‰足以起转。

后续现场替换仅由用户执行：先hold确认停车、停止原driver的终端1，再在已确认串口可见且能读取新install的Noetic环境只启动一个新driver。当前`slicer_shell.sh`无设备映射，不能直接用于该driver；外层运行环境与挂载须届时确认。本轮未部署新副本或创建硬件容器。[现场短单](SLICER_ODOM.md)列出标准launch、保持全链的换版与rosbag同步采样步骤；不是本轮已执行记录。

## 充电期间的只读综合分析

本节保留修复前的只读分析。依据是当时抓回的日志、3.806秒末段静止bag及Jetson源码；当时控制源码与抓回副本hash一致，见核对结果（原归档未随仓库收录：`../../evidence/2026-10-23-final-capture/09-current-source-hashes.txt`）。该轮充电期间未启动定位/底盘驱动、未发目标/速度/控制权命令，未改运行参数或队友树。当时只剩通信/roscore容器；实时master查询失败已保留，不冒充当前在线定位数据。

| 结论 | 证据与边界 |
| --- | --- |
| 已确认混合Twist到PWM存在不连续 | 相同vx=.125，w=0约142‰，w=1e-6约472‰；分支增益相差3.32185倍。WASD纯直行/纯转验证不会覆盖这个组合输入问题。连续映射修复可以消除该跳变，但不能直接当物理速度标定。详见驱动分析（原归档未随仓库收录：`../../evidence/2026-10-23-final-capture/driver-analysis.json`）。 |
| 转向停滞符合当前执行力度偏弱，但未测得起转门槛 | 原驱动纯转w=.25只给约71‰，原WASD配置300‰。用户确认已release后只转一点即停；这与纯转响应不足相符。300‰是已有操作值，不是测得的最低门槛；低电量、负载和地面状态也未同步记录，不能无依据把角速度提高到1.06。 |
| 规划轨迹与允许执行的运动存在差异 | 本轮rosout有52个负vx被截为0、角速保留的日志样本。TEB的倒退上界=.02是优化器正值，而Guard严格禁倒车；持续输出负vx说明不能把TEB轨迹当实际已执行轨迹。仅改max_backwards=0不是可靠修复。 |
| 33cm超调已被静态位置交叉支持，动态原因仍未定 | 第二次目标(.666,.009)，后来未挪车的odom约(.993,.058)，误差.331m，反推位移约1.331m。TEB0.9.1到点分支不检查实测速度；当前xy容差.15m，因此软件成功不是停稳验收。缺成功/零命令前后完整时序，尚不能区分实际滑行、参考执行延迟和运动时定位误差。 |
| 避障已接入，实际绕障能力未证明 | 末段scan、两个costmap均有数据，local无unknown、机器人半径.5m邻域全free，图中有侧方占据及膨胀。只能说明该静止时刻的观测，不证明此前摆放物体被正确识别，也不能把所有NO PATH归因为真实封路。 |

独立审查还指出：部分原始日志将`NO PATH`与`legal potential was found`成对记录，这些事件位于GlobalPlanner合法势场后的路径提取阶段，与TEB trajectory infeasible、Guard sweep hold或普通空路径不同。现有证据不能区分零梯度、势值、越界或步数耗尽；也不能用队友注释中的路径点数证明同图同起终点可行。精确比较四组合需失败时原始cost字节、origin/resolution、TF、起终点及运行参数；OccupancyGrid压缩cost，回灌只能称近似复现。本轮不凭缺失证据切换配置或放开unknown。

TEB 0.9.1可行性检查和Guard策略也不能等同：相关TEB分支把footprintCost为-1视为碰撞，不能声称该处已拒绝全部unknown(-2)/outside(-3)；本包Guard仍需按填充足迹拒绝占据、未知和越界，不为凑可行关闭该层。

末段bag独立量化见JSON与逐帧统计（原归档未随仓库收录：`../../evidence/2026-10-23-final-capture/08-final-passive-snapshot-analysis.json`）：odom约19.34Hz、scan约10Hz；odom横向/纵向跨度约5mm/2.5mm，轮式odom静止，TF仅/slicer_ekf发布odom→base_link。Guard/Mux各75条Twist全零，Mux为released，Action为PREEMPTED。当前launch所引用的融合配置选项（原归档未随仓库收录：`../../evidence/2026-10-23-final-capture/13-current-fusion-selected.json`）为LIO x/y/yaw + 轮速vx，不融合wheel绝对pose/yaw/wz；它是当前文件，不能冒充历史参数快照。不同轮式/LIO积分原点的绝对坐标差不能直接当定位错误。

末段代价地图/扫描叠加图（原归档未随仓库收录：`../../evidence/2026-10-23-final-capture/snapshot-costmap.png`）使用bag中真实TF转换到车体坐标系，scan与local map相差约110ms并已标注。图仅展示静止观测；不能作为之前放置物体被识别或绕行成功的证据。

按时间线分析（原归档未随仓库收录：`../../evidence/2026-10-23-final-capture/timeline-analysis.json`）拆开4次/ugv导航启动后，rosout可计5次软件到达、4次oscillation中止。33条TF超时全部位于20:30的传感器关闭/切换窗口，不能解释21点后所有转向失败。另有同名Guard/Mux替换记录；早期根/move_base的1.06rad/s补偿日志不属于后续新/ugv实例配置。

停止源码路径中，精确零不经过缓降：Guard、Mux和原driver均给零参考。串口0x11零PWM没有主动刹车模式字段；参考固件源码明确将零态写为`00/coast`，所以源码意图可见，但没有把它绑定到当前Flash及实际电桥极性/使能，不能认定实车主动刹车、制动距离或实际PWM已归零。原driver已解析电压和PWM等遥测，但未提供该轮连续ROS记录出口；动力电池对转向力度的贡献仍未知。新副本诊断补记录出口，不补造历史执行证据。

最小处理顺序：先在独立新工程准备并离线审查连续PWM映射修复，保留纯直行/纯转端点、零、限幅及Watchdog；不覆盖队友树。充电后只补同一Guard/Mux链内的导航组合输入、纯转响应和零指令停车记录，记录实际yaw/位移并复用现有串口所有者的电压/PWM遥测。随后再决定匹配哪些控制参数，复验短距离停点，最后验证基础障碍响应。纯侧方目标是合法的非全向最终位姿，但需要转向/前进/回正，不适合作为尚未匹配好转向响应时的第一项验收。

已完成本轮授权范围：接线审查、ARM64原生构建/安装、隔离授权检查、真实输入及无Action目标的GetPlan。短距离自主到点、实车基础障碍响应仍由用户现场执行，未记为通过。没有RTK迁移、自动恢复、底盘基础运动复测或部署系统优化。

## 实现与来源

- 最初的`ugv_slicer_control`选择性保留五个既有适配源文件；本轮修改Guard并加入独立C30D driver副本，不能继续将当前包称为五文件逐字保留。来源及变更见[SOURCES.json](../../../src/ugv_slicer_control/SOURCES.json)，不覆盖队友包。
- `slicer_odom.launch` 用已有ROS参数接 `/cmd_vel/nav_raw → Guard → /cmd_vel/nav → Mux → /cmd_vel`。Guard绑定实际planner/costmap；Mux以`manual_release_owner=/ugv/control`授权真实新身份，启动held。
- 一次性 `control.py state/release/hold` 适配原协议；校验当前实例/代次/ACK、新鲜状态与唯一owner，拒绝活动目标释放。hold先锁定，再取消；取消失败仍held。没有后台manager或新任务状态机。
- `goal.py plan --relative` 只调用GetPlan；`send --relative` 用标准TF转换一次为固定odom目标。没有以base_link移动frame反复更新目标。
- 当前队友源：Mux仅优先级切换、Guard硬编码`/move_base`；不能用参数获得上述能力。故仅保留已验证适配版本，**不是声称车上旧进程已支持新授权**。初次远程检查时没有切换原导航；后续由用户按短单现场切换。
- 初次接线取证的队友总配置hash为`fcf88de8…`，控制配置`c365d778…`，来源见11（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/11-config-sources.log`）、12（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/12-current-geometry-navigation.log`）；最终抓回核对又有后续hash，不能把前者无时间范围地称为当前配置。早先选定源生成外廓、后续队友快照与每次运行参数分别记录。field覆盖保留该次核对的`.144m` TEB间距，其他调试上限/较大包络不放宽，不伪称实测标定。

## 历史已通过：本次修复前版本

| 检查 | 结果与证据 |
| --- | --- |
| WSL新代码回归 | 14-review-fixes-wsl.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/14-review-fixes-wsl.log`）：29个JUnit条目，0错误/失败/跳过；build/install成功。 |
| ARM64原生构建/安装 | 19-arm64-final-build-test.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/19-arm64-final-build-test.log`）：同样29条目全通过。原生aarch64镜像 `d9b9ee3be352…`，无设备、network=none、独立master。源码hash在日志开头核对。 |
| 最后控制边界回归及安装 | 31-WSL（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/31-final-control-wsl.log`）、32-ARM64（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/32-final-control-arm64.log`）：两端重新构建、运行受影响的控制包测试并安装；13项控制unit和实际ROS交接场景通过，累计JUnit为32条目、0错误/失败/跳过。未重复未改动的导航场景，完整导航回归见[14]/[19]。 |
| 授权/接管边界 | 13项unit + 一个实际ROS场景，覆盖新owner、旧owner拒绝、实例/代次/ACK、锁存拒绝、活动目标拒绝、接管/取消缺链仍held、RPC延迟与原header新鲜度、急停不能由false解除，以及释放等待期间发生新接管/Mux重启必须拒绝。不是实车运动测试。 |
| 真实配置的完整接线启动 | 29-arm64-held-wiring.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/29-arm64-held-wiring.log`）：ARM64隔离master读取当前队友只读配置，实际Guard订阅`/cmd_vel/nav_raw`和`/ugv/move_base/local_costmap/costmap`；Mux启动held、selected_source=zero，最终Twist全零，driver订阅不存在；未调用release或发目标。 |
| 安装/环境与namespace | 17（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/17-final-install-parameters.log`）与28（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/28-arm64-runtime-environment.log`）：安装入口可用；自定义namespace绑定正确；现场shell能找到队友现有sensor包和原driver包，读取已有devel_navigation，不修改/重建它。新接线launch无driver/TF发布者。 |
| 真实输入采样 | 21-real-input-samples.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/21-real-input-samples.log`）：4秒78个odom样本约19.24Hz、40个scan约10.00Hz。唯一odom发布者/slicer_ekf，唯一scan发布者/slicer_scan_filter；odom→base_link唯一owner为/slicer_ekf。末样本odom约23ms、scan约134ms、TF约23ms。真实scan为base_link，720束，512个有限返回。仅当前采样，不是全天/所有工况保证。 |
| 交付前再次采样 | 33-latest-input-samples.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/33-latest-input-samples.log`）：同样只读采样，odom约19.41Hz、scan约10.02Hz，TF约58ms，发布来源仍唯一，inputs_valid_this_sample=true。 |
| 真实输入可规划 | 27-observed-real-getplan.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/27-observed-real-getplan.log`）：独立`/ugv_check/move_base`收到12帧真实costmap后，当前base前方.5m目标在free cell，GetPlan返回21点；.3m前方13点、左右.5m各22点。实际CLI plan再次返回21点。无Action目标、无观测到的raw运动输出；检查raw只有临时观察订阅，结束后无订阅，旧Guard仍只订阅原/move_base的raw。 |

当时累计32个JUnit条目包含3个rostest包装，不能当32项独立功能：22单元测试、5导航场景、1客户端流程场景、1控制交接场景及3包装。导航模型直接积分raw，未经过Guard/Mux/driver；控制交接场景起真实Mux，但不发速度、不起Guard/driver。它们不能证明本次driver导入/发送或Guard运动执行路径已通过。[21]/[27]/[33]才是该轮真实传感器输入检查。

## 失败与修正保留

- 04-relative-red.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/04-relative-red.log`）：新增relative/plan实现前失败；06（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/06-relative-green.log`）通过。
- 09-arm64-build-test.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/09-arm64-build-test.log`）：首轮ARM64有1项失败。测试固定等待1.5秒就在客户端尚未完成ROS注册时发送SIGINT，未实际执行它声称测试的“已发送目标取消”路径。改为真实发送日志+raw控制事件后才中断，取消断言不变；[19]最终通过。
- 16（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/16-arm64-final-build-test.log`）：用户重启期间SSH不可达，传输/验证未完成；恢复后重新核对hash再构建，不把连接失败算通过。
- 独立代码审查发现hold可能因先等待cancel连接而未接管、新鲜度在阻塞RPC前取时两处问题。添加回归RED后修复，最终单元/ROS回归均通过。自定义namespace的owner默认也同步派生。
- 最后控制边界检查发现release等待期间若改绑最新实例/代次，可能覆盖期间发生的人工接管。现固定初始释放授权，实例/代次变化即拒绝；新增3项回归覆盖接管、重启及hold仍可锁定。两端针对回归及安装见[31]/[32]。现场短单也统一为先hold接管，再由其取消并确认终态。
- 22-real-getplan.log（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/22-real-getplan.log`）及25（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/25-real-costmap-getplan.log`）：新rolling costmap尚未积累完整扫描观测时，前方.5m仍unknown，GetPlan空路径。保留原失败；没有清图、放开unknown或降低footprint。继续收取12帧观测后，该目标实际为free，才取得[27]的非空路径。启动时暂时“服务存在”不等于已观测可规划。
- 起初真实链未启动；用户重启并启动后20（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration/20-live-inputs.log`）确认实际节点。`192.168.10.33`是用户确认的电台地址，重启后该接口与雷达`192.168.1.5`均存在，未改网络配置。

本轮WSL/ARM64最终JUnit和首次ARM64失败JUnit均保存在 evidence（原归档未随仓库收录：`../../evidence/2026-10-23-slicer-integration`）。证据均来自本轮新工程，没有复制旧项目或队友历史日志。

## 现场剩余两项与收尾

按 [SLICER_ODOM.md](SLICER_ODOM.md) 由用户完成短距离自主到点、基本障碍响应与每次停车确认。初次检查时运行的是用户原整套map导航；后续用户已经按短单分别启动原driver、sensor和新导航，确认三段Topic接线与新实例GetPlan的22点路径。实际到点和障碍响应仍未通过。

本轮未修改队友树或旧项目、未启停用户原进程、未启动真实driver、未向实车发送目标或非零速度、未在真实链调用release/hold。仅在Jetson独立新目录保存源码并构建，清理自己启动的隔离容器与`/ugv_check`检查节点。

现有`.25rad/s`调试cap/禁turn boost不保证有效转向；300‰是既有WASD操作值，最低持续起转门槛未测得。历史现场使用旧driver映射，本轮新副本尚未部署。软件SUCCEEDED不能替代新鲜轮速、地面停稳、急停/Watchdog或制动距离验收。不重做基础WASD测试，也不自动提高参数。

## 现场release身份错误修正

用户首次现场执行 `control.py release` 返回 `release requires bound executor owner`。这是Mux的既有身份检查拒绝：授权参数为 `/ugv/control`，但普通根命名空间终端中的客户端实际初始化成 `/control`。旧报错沿用executor措辞，不要求启动旧执行器。

根因是客户端首次导入rospy后才向 `init_node(argv=...)` 传入 `__ns`；Noetic已在导入时缓存初始命名空间。此前真实ROS测试继承了rostest的 `/ugv` 环境，漏掉普通终端场景。将测试子进程环境明确设为 `ROS_NAMESPACE=/` 后，未修复代码重现相同拒绝：失败输出（原归档未随仓库收录：`../../evidence/2026-10-23-release-owner/01-root-namespace-red.log`），失败JUnit也已保留。

最小修复仅调整新客户端：使用rosgraph解析参数，在首次导入rospy前设置真实进程的ROS命名参数，并在创建Topic前核对实际节点名。没有更改五个复用源文件、Mux授权、Guard或driver。WSL及原生ARM64均重新运行完整隔离回归与安装，各32个JUnit条目、0错误/失败/跳过：WSL（原归档未随仓库收录：`../../evidence/2026-10-23-release-owner/02-wsl-full-green.log`）、ARM64（原归档未随仓库收录：`../../evidence/2026-10-23-release-owner/03-arm64-full-green.log`）。新版客户端已安装到独立新工程，现场现有launch不需重启；实际release及运动仍由用户执行。

安装后只读检查（原归档未随仓库收录：`../../evidence/2026-10-23-release-owner/04-installed-readonly-state.log`）：源文件和安装文件hash一致，普通根命名空间的state成功通过实际节点名检查，实车仍manual_hold=true、selected_source=zero，最终Twist全零。last_intent_reason仍保留用户此前失败请求的记录，未由助手再次调用实车release。

## 实车距离超调：到点验收未通过

用户随后实际执行相对0.5m与1.0m目标，均返回 `state=3 Goal reached.`，但同一车体参考点测得约0.8m以上、1.3m。第二次目标打印为odom `(0.666, 0.009)`、四元数z/w约`.0496/.9988`。用户确认第二次结束后没有挪车或再发目标。

结束后只读快照（原归档未随仓库收录：`../../evidence/2026-10-23-overshoot/01-current-pose-driver-readonly.log`）显示odom约`(.9933,.0583)`，距该目标约`.331m`，最终Twist为零、Action保留status3。根据打印目标与相对1m反推起点，估计前向位移约1.331m，与用户物理测量接近。这是后续静态快照，不能替代成功瞬间与零指令到达时刻的数据，不能直接把超调归因惯性、定位尺度或驱动某一项。

已确认两处需要区分的事实：

- 安装的TEB 0.9.1只检查当时位姿进入xy/yaw等条件，不以实测停稳作为成功条件。`free_goal_vel=false`是规划末速度边界。[0.9.1到点源码](https://github.com/rst-tu-dortmund/teb_local_planner/blob/0.9.1/src/teb_local_planner_ros.cpp#L257)。move_base接到成功后发布零速并结束，不持续复核原目标。因此历史status3与后续33cm误差并不矛盾。
- 当前原驱动 `control_core.wheel_pwm` 在`w=0`与`abs(w)>1e-9`切换两套增益；相同`vx=.125`，`w=0`四轮PWM幅值约142‰，`w=1e-6`约472‰。这是代码计算的PWM跳变，不能当实际速度3.32倍。当前配置纯计算（原归档未随仓库收录：`../../evidence/2026-10-23-overshoot/02-mapping-calculation.log`）没有ROS、串口或电机输出。现有WASD基础直行验证未覆盖导航同时带微小转向的输入。

Guard/Mux/原驱动代码中，精确零目标均立即清零参考输出；实际MCU制动方式与滑行距离没有由此得到验证。尚缺同一goal的result、TF/odom、raw→Guard→Mux速度时间序列，需下一次用户现场自主导航试验同步记录，不重做WASD基础运动测试。此次未改参数、未改队友树/驱动、未由助手发目标或控制权命令。基础障碍验收暂不推进；先定位距离超调。

## 后续不动/oscillating试验：本次Mux未释放

用户放置障碍物及移至开阔位置后发送前方、侧方目标，报告车根本不动，部分目标以 `state=4 Robot is oscillating`结束，其余手动取消。日志中Guard持续保留wz=±.25、拒绝极小负vx。只读状态（原归档未随仓库收录：`../../evidence/2026-10-23-obstacle-turn/01-current-state-readonly.log`）发现新Mux实例`495e5851…`，manual_hold=true、hold_reason=startup、没有接受过本次实例的释放请求；最终输出及配置（原归档未随仓库收录：`../../evidence/2026-10-23-obstacle-turn/02-current-output-config-readonly.log`）也保留了现场快照。用户随后确认：这批试验前重启过终端A，没有重新release；手动挪车是在取消目标之后，未hold。

该批完全不动的直接原因已定位为本次启动锁定。move_base和Guard不拥有Mux释放权，能产生raw/Guard输出不代表最终速度已放行。move_base当前oscillation_timeout=20s、oscillation_distance=.05m，按XY位移重置计时；静止也会报该文案，不代表观察到物理往返震荡。避障层已接入，但这批试验不能作为绕障验收结果，亦不能归因为空间不足或转向PWM不足。

独立的转向接口限制仍保留：纯转wz=.25按原映射约71‰，原WASD配置300‰；后者不是已标定起转最低值。需要在确认Mux放行后另行验证实际转向响应，不能凭锁定状态下的不动认定其因果，更不能盲目提高角速。前一轮已经释放状态下的33cm距离超调仍待同步记录定位；本轮没有修改任何参数、源码或运行进程，也没有由助手执行实车release/hold/目标。
