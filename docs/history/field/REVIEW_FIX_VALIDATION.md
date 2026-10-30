# 独立审查后修复与验证 · 2026-10-23

本轮只在 WSL 新工程修改、构建及隔离验证；未连接车辆、部署 Jetson、修改队友树、启动真实驱动或发布实车目标/控制权命令。历史现场报告原文保留。基线 Git 提交 `23728e8`，工作分支 `fix/review-control-chain`。

## 已实施

| 审查项 | 最小改动与适用边界 |
| --- | --- |
| P1-1/P1-2：映射不连续且新函数未接入原进程 | 新包加入受控 C30D driver 与必要的原协议、里程计、串口模块，实际导入本包连续映射。保持纯直行/纯转端点、符号、600‰ ROS cap、750‰固件cap参考、slew、立即零、反向过零、命令/遥测Watchdog及关闭零。原driver与队友树不改；用户日后不替换原进程，就仍在用原映射。 |
| P1-3：倒车圆弧被改成纯转 | 禁倒车时整条负vx命令归零，并发布明确hold_reason；补偿不能重新放行。合法原始纯转仍允许。它修复动作改写，**没有解决侧方目标的前向规划进展**。 |
| 诊断缺口 | 唯一串口所有者发布标准DiagnosticArray：接收输入、映射/slew后PWM、实际发送序号与发送起止、每一帧遥测。`~provenance`参数和启动日志保留配置及模块路径/hash；不另开串口、不加运行管理器。 |
| 文档过强表述 | 300‰为既有WASD操作值，未测最低起转门槛；参考固件源码零态为coast但未确认当前Flash；TEB成功不含实测停稳；旧模型直接积分raw，不能证明驱动或Guard链通过。 |

入口为独立的`roslaunch ugv_bringup slicer_driver.launch`，不由导航launch自动启动。现场替换还需确认实际串口运行环境、挂载和停止原driver；本轮未进行这些操作，见[现场短单](SLICER_ODOM.md)。

诊断的MCU `accepted_command_seq`表示接受命令，不表示实际执行或停车。发送PWM为permille；`pwm_timer_ticks_abcd`为timer ticks。异常隔离与ROS异步发布队列不构成硬实时延迟保证。

## 软件验证与失败证据

所有ROS节点和模型均使用`tools/noetic.sh`：无设备、`--network none`、独立master。driver fake transport是匿名管道，走实际main、Chassis、packet及Parser，不使用USB。实际电机、起转死区、惯性和电池均不在模型内。

- 修复前基线：32个JUnit条目通过，完整输出（原归档未随仓库收录：`../../.test-results/run-Gpz0VkGv/output.log`）。包括包装条目，不是32个独立实车功能。
- Driver：20项unit通过。包含实际导入及混合PWM、极小角速两侧、纯直/纯转、饱和、非有限输入、零/反向、精确命令/遥测边界、故障/写异常关闭、逐帧遥测、CRC/分片/序号回绕与持久provenance。RED（原归档未随仓库收录：`../../.test-results/review-driver-red.log`）、provenance RED（原归档未随仓库收录：`../../.test-results/review-driver-provenance-red.log`）、GREEN（原归档未随仓库收录：`../../.test-results/review-driver-green.log`）。
- Guard：6项纯逻辑与11项真实双节点场景通过。包含整条拒绝、补偿不能复活、纯转、零、命令/TF/grid过期、未知/占据/越界及未来扫掠。unit RED（原归档未随仓库收录：`../../.test-results/review-guard-red.log`）、ROS RED（原归档未随仓库收录：`../../.test-results/review-guard-ros-red.log`）、独立ROS GREEN（原归档未随仓库收录：`../../.test-results/review-guard-ros-final.log`）。
- 新真实TEB→Guard→Mux→理想非全向模型：startup held确实阻断真实raw；明确授权后前进到点和取消零传递两个场景，首轮输出（原归档未随仓库收录：`../../.test-results/review-chain-first.log`）。模型只积分最终Mux输出，driver执行由上述假串口测试另验。
- source/devel/install三路径导入检查、实际安装入口解析与安装成功：install smoke（原归档未随仓库收录：`../../.test-results/review-install-smoke.log`）。用import-only、`rosrun --prefix echo`和`roslaunch --files`，没有启动driver；devel wrapper委派本工程source，install全部导入安装目录副本。
- 真实ROS消息补充检查：生产driver显式`mock=true`，真实master、参数服务器和DiagnosticArray传输；连续PWM为`[142,-142,142,-142]`、显式零可观测，provenance可晚查询；未打开串口，未生成MCU遥测。输出（原归档未随仓库收录：`../../.test-results/review-driver-ros-smoke.log`）、复现脚本（原归档未随仓库收录：`../../.test-results/check_driver_ros_diagnostics.py`）。该检查不覆盖MCU执行。
- 完整套件首轮：73个JUnit条目，1项失败，原始输出（原归档未随仓库收录：`../../.test-results/review-full-suite.log`）、失败JUnit（原归档未随仓库收录：`../../.test-results/run-uhMPfqj2/junit`）。TF过期场景实际看到command stale。诊断运行中另见TF回拨告警（原归档未随仓库收录：`../../.test-results/review-guard-timing/trial-3.log`）与mono相对ROS/wall偏移变化，不能证明monotonic本身回拨或锁定原失败唯一原因。
- 测试夹具补本轮实际发送与连续新输出屏障，并在setUp失败时清理线程；记录发送间隔、连接和三种时钟。没有放宽生产超时或删除安全断言。屏障首版误用了滚动门槛，导致12项夹具失败（原归档未随仓库收录：`../../.test-results/review-guard-readiness.log`），这是本轮新增测试代码错误，已改为请求后的首条发送；最终单次夹具验证（原归档未随仓库收录：`../../.test-results/review-guard-fixture-final.log`）11/11通过。所有诊断与开发失败保留，不能据此宣称已消除环境时钟风险。
- 最终完整套件单次复跑：**73个JUnit条目，0错误、0失败、0跳过，构建和安装成功**。完整输出（原归档未随仓库收录：`../../.test-results/review-full-suite-final.log`）、原始运行目录（原归档未随仓库收录：`../../.test-results/run-ehnzvvFm`）、JUnit（原归档未随仓库收录：`../../.test-results/run-ehnzvvFm/junit`）。其中48项unit、20项ROS场景、5项rostest包装，不能当作73项独立功能或实车验收。

证据目录`.test-results/`由Git忽略，全部RED、失败JUnit和GREEN保留在本工作区，不复制到队友树或旧项目。

## 保留失败的可达性实验

复现命令（只可在隔离软件环境执行）：

```bash
./tools/noetic.sh bash -c 'source devel/setup.bash; roslaunch ugv_navigation guarded_characterization.launch'
```

原始逐条raw/Guard/final与终态（原归档未随仓库收录：`../../.test-results/guarded-characterization.json`）、实验日志（原归档未随仓库收录：`../../.test-results/review-chain-characterization.log`）。脚本完成仅表示数据收集完成，不是到点PASS；未伪造Action结果。

| 目标（相对发送时车体） | 实际Action | 模型末端误差 | 负vx raw |
| --- | --- | --- | --- |
| 前进0.5m | SUCCEEDED | 约0.14423m | 0条 |
| 同位置转45° | SUCCEEDED | 位置0m、角度约0.21486rad | 0条 |
| 侧方0.2m、保留最终朝向 | ABORTED：oscillating | 0.2m，无平移 | 213条，被整条拒绝 |
| 侧方0.5m、保留最终朝向 | ABORTED：oscillating | 0.5m，无平移 | 213条，被整条拒绝 |

这是理想模型及现有`.15m/.25rad`软件容差下的结果。不能据此说真实车辆停点、转向或绕障通过；侧方失败也不是全向轮需求，而是当前优化轨迹持续落在禁止动作集合中。

## 仍未解决与下一次最小取证

- 当前TEB前向权重及初始化是软策略；禁止倒车出口保留，但侧方前向轨迹尚不可执行。不用max_backwards=0或提高角速制造通过。
- GlobalPlanner合法势场后提取失败仍缺失败时原始cost字节与同步起终点，现有OccupancyGrid不可精确逆还原。四组合、unknown、足迹及恢复配置不改。
- 未改变角速上限、停车容差或固件；约71‰是否能可靠起转、零态物理滑行、编码器/运动定位与历史33cm偏差均需同步实车证据。
- 新版本尚未ARM64重新构建/部署；历史ARM64通过不能覆盖新增driver。现场换版前仍需对本版本做ARM64无设备验证。
- 用户充电后先在同一控制链记录一次0.5m目标的接收→发送→MCU接受/遥测→轮速→外部刻度停止；允许继续时再加45°纯转。前两项未明确前不推进障碍目标，不重复基础WASD运动测试。
