# 换电后零速复核与轮速driver ARM64隔离验证

后续A阶段的真实ROS零速检查及最小诊断快照修复已另行完成，见[无运动接口验收记录](WHEEL_SPEED_ZERO_INTERFACE_2026-10-24.md)。下文保留本记录结束时的状态。

本次继续用户授权的固件无运动核验，并完成新版driver的ARM64隔离构建前置项。没有修改生产逻辑、重烧固件、覆盖Jetson当前源码/install、启动ROS真实驱动/导航或发送release、非零速度/PWM、标定及导航目标。队友开发树保持只读。

## 换电后的实际检查

用户再次确认：车辆静止、四轮架空、Motor OFF、12V电机动力物理断开、驱动及串口程序停止。控制板USB身份仍是`1a86:55d4 / 5CF0016377`，本次路径为`/dev/ttyACM1`，绑定`cdc_acm`；不能沿用换电前的`/dev/ttyCH343USB1`名称。访问前后未观察到其他串口owner。

原检查器`/home/potato/Slicer_Workspace/tools/check_c30d_runtime.py`的SHA256仍为`34b625c9ed17b9b2224d13b206fa0f93c7e17638d336f727863af3609b187301`。首次执行在HELLO等待2秒后超时，未形成通过结果；日志完整保留。随后用原独占Link只接收2秒，没有发送协议字节或主动设置DTR/RTS，接收3570字节、样本含有效SLC2的0x90遥测，说明MCU应用在运行；这次观察没有保存完整字节流，不能据此计算全窗口CRC或tick统计。打开tty本身可能涉及内核的modem控制行为，不能把“未主动设置线状态”等同于电平完全未变。

保持物理条件和检查器不变，仅重试一次，实际exit0：

| 项目 | 结果 |
| --- | --- |
| 时长、遥测 | 5秒、126帧、25.064Hz |
| MCU报告间隔 | 40000us |
| CRC错误 | 0 |
| 零速命令确认 | 97次 |
| 软件duty、Motor及动力 | 全零、OFF、断开 |
| 遥测电压 | 4.299–4.316V；动力断开时的板端读值，不是新动力电池实测电压 |
| 运动/编码器验收 | 未执行 |

没有放宽原脚本的EN/动力/电压/CRC/ACK条件。只进行握手和连续0x10四轮零；未发送0x12方向配置、0x11 PWM或任何非零命令。首次握手超时原因仍未定位，不能说启动暂态、Linux驱动或固件是已确认根因，也不能说问题已修复。

本地证据位于`.test-results/c30d-timebase-20261024/`：`zero-check-after-battery-run.log`、`after-battery-passive-serial.json`、`zero-check-after-battery-retry-run.log`、`runtime/zero_check_after_battery_retry.json`、`after-battery-final-state.log`。原刷前双读、烧写回读和首次EN拒绝证据未覆盖。换电后未再次读全Flash；此前的实际烧写回读证据见[固件记录](../firmware/C30D_TIMEBASE_2026-10-24.md)。

## ARM64构建与隔离安装

输入为提交`f405e0d8f114baa9c2c4be380c3a700ba6e4d585`中的69份`src/`、`tools/`、`Dockerfile`文件，没有运行日志、设备输入或历史归档。源码tar SHA256为`46ffd000971f9f40178a88a1f21c0db2f9acab9fafac51483e9f973836be9c64`；构建后逐一比对69份源文件，差异为零。

Jetson独立测试树：`/home/potato/slicer_ugv_ros1/.test-results/wheel-speed-arm64-20261024/source`。采用已有`tools/test_noetic.sh`，容器`--network none`、不映射设备、独立ROS master；ARM64镜像`industrial-cup/nav-slicer:20261023-dwa`，ID `sha256:bd1daedc41721933507b0017b2acae3a8079b583489498db96707abad069feca`。实际容器`uname -m`为`aarch64`。

`catkin_make`、现有单元/ROS软件测试、`catkin_make install`均exit0。JUnit汇总101个条目、0 errors、0 failures、0 skipped（含39项driver测试）。软件模型仅验证软件，不证明MCU、电机、方向或实车避障。安装只写独立测试树内的`install/`；在同样无设备、无网络容器中核对ROS包及`roslaunch --files`解析，没有启动launch节点。

| 输入/隔离产物 | SHA256 |
| --- | --- |
| driver源码 | `a6cc30f1e812f587679d6852cede3b658fe8e50df851e43c3403c9d427247fcc` |
| c30d_link.py源码及隔离安装 | `f8b66b62ed6ab6261b230cc8de51ce25bd137ad5cd4b1edb348196311168bb55` |
| control_core.py源码及隔离安装 | `3a401f39aea9cf3013aca56764932f895f201c9e6c24c9d55e4debc975db8b9a` |
| wheel_speed参考配置隔离安装 | `d9d0ff6bc99eb614c2700444d58ee9355acc5b890b80d4fc58e03dd21f370f04` |

本地证据`.test-results/wheel-speed-arm64-20261024/`包含源码清单、逐文件比对、完整构建日志、JUnit/退出码、安装hash和解析记录。测试run为`run-T8MEGuQI`；原始测试证据tar SHA256为`b858895aad60257e29217dabab77c22e42cd08c44737c6f9bf4c35828d2bcab0`。车端当前源码的三个driver模块在检查前后保持旧hash；没有将测试安装切为当前运行安装。

## 验收边界与下一步

固件三处源码修复、离线实际C测试、构建、烧写回读及固件零速核验已完成。新版driver仅完成软件/ARM64构建安装，A阶段ROS接口检查尚未执行，B带动力闭环和C导航验收也未执行。

A仍须在用户现场启动后观测：唯一串口owner、Mux `manual_hold=true`且`teleop_fresh=false`/`selected_source=zero`、最终Twist持续零、明确选择wheel_speed参考配置、0x12独立符号/匹配nonce ACK、configured及tick推进、连续0x10四轮零、无PWM降级。Motor OFF时ready=false可以正常；held仍允许人工teleop，因此只看held不能证明最终输出为零。

现有diagnostics不锁存，必须先订阅再启动，才能捕获配置过程。配置ACK日志含nonce，但0x12发送日志未记录请求nonce，外部日志不能独立比较两者；内部代码`configure()`确实只接受本会话nonce，验收要明确区分源码核对与外部观测。新鲜度按主机接收时间判断，不能替代对MCU tick推进的独立检查。上述均为观察边界，本次没有加入新的管理器或校验机制。

B仍需用户验证ABCD轮口、PWM/编码器方向、轮端计数与mm/s单位、低速起转/维持/回零/换向/掉命令停车及地面负载，保持Guard→Mux→唯一driver→Watchdog。候选符号不是实测值；低目标不保证低PWM，0x90 duty不是CCR或转矩，一帧零不证明车轮停稳。B通过后才进行C的短距、纯转、取消/hold、停后误差和基础障碍验收。
