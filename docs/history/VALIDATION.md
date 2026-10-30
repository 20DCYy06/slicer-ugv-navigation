# 第一阶段验证记录 · 2026-10-23

本文保留第一阶段历史软件验证。独立审查后的连续PWM driver副本、诊断与Guard修复为后续变更，其验收范围见下一节；以下旧测试数字不能作为新变更通过证明。最新实车结论见[接线验证](field/INTEGRATION_VALIDATION.md)：直行停点、有效转向与绕障未通过。

第一阶段结论：新独立 workspace 的 **odom 理想模型导航闭环通过**；硬件与物理安全链未验收。当时无Jetson部署、无队友树修改、无真实驱动或实车运动目标。旧项目仍停在原HEAD `65308aa`，原先唯一未跟踪压缩包仍在，未修改旧树。

## 独立审查后修复的验收范围

本轮在新工程本地准备受控C30D driver副本、标准诊断和Guard负vx整条拒绝；不部署、不连接硬件、不发实车目标、速度或控制权命令。driver不由导航launch自动启动，实际导入连续PWM函数，避免只测试未被旧driver调用的同名模块。

| 检查 | 必须验证的执行边界 |
| --- | --- |
| driver与连续映射 | 实际driver main、导入链和真实SLC2编解码走假串口；检查零点附近连续性、纯直/纯转、混合/饱和、非有限输入、立即零、反向过零、命令与遥测超时及关闭零。 |
| 诊断 | 唯一串口所有者记录命令接收、发送PWM/序号和接收遥测；ROS stamp与主机单调时钟可追溯。ack表示接受，MCU PWM是timer ticks，不充当执行或物理停车确认。 |
| Guard与控制链模型 | 实际Guard节点验证正向、原始纯转、负vx整条拒绝及hold_reason、零/超时、TF/grid失效和occupied/unknown/outside扫掠；模型仅执行Guard/Mux后的输出。 |
| 安装路径 | 确认安装driver及实际导入模块来自新工程，配置与导入路径/hash可观察；导航入口仍不启动driver。 |

可重复入口仍为根目录`./tools/test_noetic.sh`；软件模型和假传输必须在无设备、`--network none`、独立master的容器运行。完整回归、安装检查及确切证据以[本次修复验证](field/REVIEW_FIX_VALIDATION.md)为准，此处不预填最终测试数量。

本轮已运行控制链特征检查（原归档未随仓库收录：`../.test-results/guarded-characterization.json`）：真实TEB → Guard → Mux后的最终Twist驱动理想非全向模型，没有driver、电机或地面动力学。前进0.5m为state3，模型末端误差约0.14423m；同位置转45°为state3，两段均无负raw。侧方0.2m与0.5m均为state4 oscillation、无平移，各有213条负raw被Guard整条拒绝。它证明拒绝策略有效，也明确暴露侧方前向规划仍未解决，不能将整条链称为已修好或据此证明约71‰能在地面起转。

本轮不改GlobalPlanner四组合、unknown、外廓、角速上限、到点容差或固件。没有失败时完整原始cost字节、origin/resolution、TF、起终点和参数，不能精确复现合法势场后的提取失败；OccupancyGrid压缩后的cost不能还原原始值，回灌只能称近似复现。修复不保证侧方进展、约71‰纯转起转或历史超调消失，实车仍需用户同步采样。

## 环境及可重复命令

WSL2 x86_64；宿主无 ROS1，使用现有 Noetic 镜像 `industrial-cup/nav-slicer:20261021`，ID `sha256:9ee218b7afe4dfda22852eee4483e6618f2d4747b959cbbb2bdc8e0559cb4da7`。move_base/GlobalPlanner 1.17.3、TEB 0.9.1、rostest 1.17.4。容器仅挂新工作区，当前用户运行，`--network none`、无设备映射、cap-drop ALL；各 rostest 自建独立 master。

```bash
cd /home/rescue/slicer_ugv_ros1
./tools/test_noetic.sh
```

该命令实际完成 `catkin_make -j2`、客户端 unittest、`catkin_make run_tests -j2`、`catkin_test_results --all`、`catkin_make install -j2`。每次结果独立保存至 `.test-results/run-*`，失败不覆盖。Dockerfile 是可另行构建的环境定义，本轮没有联网构建该新镜像。

## 已通过的软件检查

最终输出：10-complete-suite.log（原归档未随仓库收录：`../evidence/2026-10-23/10-complete-suite.log`），原始本地运行目录 `.test-results/run-sfAnEV6Y`，关键 JUnit 保存在 final-junit（原归档未随仓库收录：`../evidence/2026-10-23/final-junit`）。

| 检查 | 实际验证范围 |
| --- | --- |
| 构建与安装 | 两个 Catkin 包构建及 install 成功。 |
| 客户端 6 项 unittest | 坐标/yaw 四元数、可选 frame/action、非有限坐标拒绝、无效超时拒绝、空 frame 拒绝、显式 cancel 解析。无 ROS master。 |
| 已观测规划 | 实际 OccupancyGrid 的前方 free cell 对应真实 GetPlan 非空路径。 |
| unknown/窗口拒绝 | 3.5m模型量测外、8m窗口内的目标实际为 `-1`，GetPlan 空路径；4.5m窗口外目标也拒绝。unknown 目标前方没有阻隔墙，避免把“墙挡住了”误记成 unknown 检查。 |
| 标准ROS导航组件驱动理想模型到点 | 实际 move_base/GlobalPlanner/TEB 参与；非零raw Twist直接驱动模型积分，Action SUCCEEDED，检查软件末端误差、差速命令及输出界限。没有经过Guard/Mux/driver，不是实车停点证明。 |
| 取消不复活 | 真实 Action PREEMPTED；取消后2s无非零命令、位姿不再移动，无目标重发。 |
| 临时 scan 丢失 | 停发 scan 约2.5s；失联期间停止输出运动，仍为同一 ACTIVE goal，输入恢复后原目标 SUCCEEDED，没有第二次 send_goal。并非RTK恢复或长期故障恢复。 |
| 墙体阻挡 | 真实扫描在costmap标记全宽墙，GetPlan拒绝；Action ABORTED，逐积分样本的调试 footprint 未穿墙。只验证阻挡，没有声称硬件避障或复杂障碍绕行通过。 |
| CLI 真实进程 | 直接运行 goal.py：成功、.7s超时、SIGINT、独立 cancel 命令；真实结果序列 `[SUCCEEDED, PREEMPTED, PREEMPTED, PREEMPTED]`，退出码分别0/1/130/取消发送者0，之后无非零输出。该场景也验证源码工作区的 software_demo.launch。 |
| install 接口 | 仅 source install 后 rospack 指向 install/share，生产启动仅 `/ugv/move_base`，已安装客户端 --help 可运行，`namespace:=team` 参数展开正确；见 09-install-interface-fixed.log（原归档未随仓库收录：`../evidence/2026-10-23/09-install-interface-fixed.log`）。替代 namespace 的实车/端到端运行不在本轮结论内。 |

最终 JUnit 汇总 **14 tests、0 errors、0 failures、0 skipped**，其中包含2个 rostest 包装用例；实质为6个单元测试、5个导航场景及1个含4种CLI流程的场景，不把包装重复计作额外功能。

第一阶段模型在`ugv_navigation/test/`：理想差速积分、固定房间及可选墙、360°射线，发布odom/scan/动态TF。它直接订阅raw Twist，没有目标知识、到点吸附、碰撞钳制、停车watchdog，也不发布Action result。结果由标准ROS导航组件产生，但未覆盖Guard/Mux/driver的执行路径、PWM、MCU或地面响应。演示文件只供源码工作区使用，不安装到install。

## 失败证据及处理

所有文件均为本轮新工程生成，没有搬运旧运行日志。

| 证据 | 结果/原因 |
| --- | --- |
| 01-client-red.log（原归档未随仓库收录：`../evidence/2026-10-23/01-client-red.log`） | 实现前缺少 goal.py，测试加载失败；仅是开发初始失败，不算行为验收。 |
| 02-client-green.log（原归档未随仓库收录：`../evidence/2026-10-23/02-client-green.log`） | 客户端6项通过。 |
| 03-first-integration.log（原归档未随仓库收录：`../evidence/2026-10-23/03-first-integration.log`） | Catkin顶层未设置 TOPLEVEL_FIND_PACKAGE，导致找不到 catkin_workspace 命令。按官方工作区初始化方式修正。 |
| 04-integration.log（原归档未随仓库收录：`../evidence/2026-10-23/04-integration.log`） | 当时6项单测+5场景通过；其 unknown 目标在墙后，之后加强该断言，因此不作为最终版本证据。 |
| 05-final-suite.log（原归档未随仓库收录：`../evidence/2026-10-23/05-final-suite.log`）、failed-range-junit（原归档未随仓库收录：`../evidence/2026-10-23/failed-range-junit`） | 加强unknown模型后4个场景失败。模型将量测设为恰好 range_max，标准投影器丢弃，前方未被清为free，导航正确拒绝。 |
| 06-install-interface.log（原归档未随仓库收录：`../evidence/2026-10-23/06-install-interface.log`） | 命令返回0但出现Catkin setup错误，不接受为干净检查。环境shell传入命令中的 --help 被 setup误读。 |
| 07-manual-demo.log（原归档未随仓库收录：`../evidence/2026-10-23/07-manual-demo.log`） | 同一量程模型错误导致真实Action ABORTED，客户端非零退出；容器/launch已退出。 |
| 08-range-fix-suite.log（原归档未随仓库收录：`../evidence/2026-10-23/08-range-fix-suite.log`） | 将模型量测修正为 range_max−.001，保持unknown/导航断言后6单测+5场景通过。 |
| 09-install-interface-fixed.log（原归档未随仓库收录：`../evidence/2026-10-23/09-install-interface-fixed.log`） | 环境shell保存命令参数后清空setup位置参数，重新验证无setup错误。 |
| 10-complete-suite.log（原归档未随仓库收录：`../evidence/2026-10-23/10-complete-suite.log`） | 新增CLI真实进程场景后最终完整套件通过，构建/安装成功。 |

量程边界依据 [laser_geometry Noetic 源码](https://github.com/ros-perception/laser_geometry/blob/noetic-devel/src/laser_geometry.cpp)：只保留严格小于 range cutoff 的量测。修正的是模型输入合法性，没有更改生产算法、放宽unknown或删除失败断言。scan停发、不可达目标产生的WARN/ERROR是预期负向场景；未静音日志。

第一次软件交付时的独立只读审查核对了架构、namespace/remap、install清单、模型真实性及控制边界；指出应以最新失败而非旧通过作为结论、补CLI取消流程和install入口检查。当时按上述结果处理。后续独立审查确认了driver导入缺口、PWM不连续和Guard运动改写等问题，不能沿用此历史段落宣称当前无已知缺陷。

收尾核验 11-final-check.log（原归档未随仓库收录：`../evidence/2026-10-23/11-final-check.log`） 确认了Noetic观察工具：使用 `rosrun tf tf_echo`，镜像中 `tf2_ros` 没有 `tf2_echo` 可执行文件；已修正文档并显式声明 `tf` 依赖。12-final-build-install.log（原归档未随仓库收录：`../evidence/2026-10-23/12-final-build-install.log`） 是该元数据变更后的构建/安装与测试汇总检查。测试容器最终已全部退出，未触碰其他运行容器。

## 尚未验证的硬件与后续最小待办

- **局部定位实测**：Slicer LIO/轮速EKF的噪声、漂移、时间同步、重启原点变化与真实TF唯一性。当前仅按已核查接口接入，没有启动它们。
- **物理外廓与底盘**：当前源生成包络与旧profile不一致，先确认真实车体/机械臂/载荷外廓；起转、实际线/角速度、停车延迟/距离、Watchdog、Guard/Mux、急停及人工接管尚未验收。请求Twist不等于真实速度。
- **阶段2最小待办**：从已列来源迁移ENU/时间与坐标对齐、候选全局EKF及质量检查；先用独立定位测试证明 `map→odom` 唯一与正确，再接现有导航链。不能用恒等TF代替对齐。相位中心、基准/历元、时间关系、噪声与漂移预算仍须如实核实。
- **阶段3最小待办**：拆出必要的有界恢复/停稳到点纯逻辑及测试；为现有Mux补清晰标准接口的显式交还，解除旧执行器身份耦合。恢复前检查局部原点连续性，取消/接管不得擅自重启旧任务。长期异常、进程重启、局部原点重置及精细停稳证明当前未实现，需显式重新发目标。
- 本阶段无GNSS拒止SLAM、无人机无线通信、配送协议或全系统 `/ugv` 迁移。默认 namespace 只是本导航入口，不代表整车迁移已完成。
