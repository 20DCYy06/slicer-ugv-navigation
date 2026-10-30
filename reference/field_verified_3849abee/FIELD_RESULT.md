# Work候选3849abee：烧写及实测记录（2026-10-29）

来源为用户明确提供的c30d_yaw_fix_20261029_3849abee.zip；包内说明/脚本仅作材料，烧写授权来自用户本轮指令。原包和解包内容未改，未自动合并队友源码。

生产只改BALANCE/balance.c的5行：前后均纯转、yaw请求反号且旧I反对新请求时，仅清yaw I；P/D历史、linear I、轮PI、限幅和保护不变。与已装10144字节92e816版相比，config/generated/main/header等逐字节未改。新镜像10208字节，SHA256 3849abee0451ef94a6ac5bfd5bbe508c7ca9da016ae87afed70ee09ea18a17a4。

车端原独立构建的31个唯一输入/产物逐项校验与包一致，9条原构建/测试命令exit0；本轮未为重复构建覆盖原产物。本轮另在无设备/无网络容器，以同一最终测试分别编译旧/新生产代码：997检查旧版965失败、新版0失败；旧yaw-suite双方通过，2项纯协议载荷通过。审核未发现烧前阻塞，但不是实车通过。

用户确认本轮架空、Motor OFF、12V物理断开、串口程序已停止后开始ROM流程。preflash/包含同板512KiB完整双读、选项字节及日志。刷写仅sector0；完整回读与实测状态见RUN_RESULT.json及install/。UID区域若NACK，不解除保护，也不宣称UID已验证。

本轮不改driver、Guard/Mux、EKF、DWA或MCU参数，不依照包内README扩展到0x10或其它运动测试。运动由用户现场确认后执行；助手只采集。测试拟保持上一轮±0.08rad/s、20Hz，恒定/每2秒交替对照。

回退材料是本轮同板preflash/snapshot/flash_read_1.bin及其中前16KiB；恢复时仍须断动力，只写sector0，随后完整512KiB与原备份比对，选项字节不写。本轮不自动回退。

## 当轮实车结果

| 项目 | 实际结果 | 判断 |
|---|---|---|
| 恒定左转，目标+0.08 | 主要连续段MCU角速中位数+0.083rad/s；首次超过+0.01约1.08s；左转raw窗口内Guard拒绝约1.268s | 能左转，但该段不应当作完整无干扰跟随试验 |
| 恒定右转，目标−0.08 | MCU角速中位数−0.085rad/s；首次低于−0.01约0.20s；该raw窗口Guard一直clear | 能持续右转 |
| 每2秒左右交替，幅值0.08 | 用户确认实际左右交替、A轮正常、自停；三段负向角速积分−0.07036/−0.06672/−0.05264rad，上版为+0.00540/+0.00736/+0.00228rad | 方向交替功能在本轮恢复 |
| 交替段速度质量 | 三段负向请求积分−0.15976/−0.15744/−0.15232rad，仅跟随约44.0%/42.4%/34.6%；角速中位数−0.004/−0.008/−0.004rad/s | 仍有脉冲/起转迟滞，不能宣称精确低速跟随通过 |
| fault及结束状态 | 9311帧MCU反馈，876帧非零目标，0fault；最终目标、四轮反馈、软件duty归零，Mux manual_hold=true | 已停车保持 |

整组最后六个2秒raw窗口Guard均clear。首段左转的原始拒绝字符串为`swept footprint occupied/unknown/outside grid`，这是联合原因，未据此断言一定碰到某种实物。没有放宽Guard。

角度来自MCU角速度的25Hz矩形积分近似，边界至少一帧不确定性；不是尺量/外部视频量角。软件duty为timer ticks，按已知pwm_sign归一也不等于测得电桥电压或转矩。未录到内部轮目标/yaw I/wheel I，不把整个动态现象都归因于某一个积分。

与上一轮的变化：上一版三段反向均未形成净反转，本版三段均形成净反转，现场观察一致。与此同时正负交替仍只有脉冲式运动，最低稳定速度、静摩擦/电流和轮PI低速输出建立问题未解决。没有因本次方向改善而自动增加速度、改增益或继续更长导航；本轮也未做短直行。

## 证据与复跑

- `local-check/`：本轮无网络无设备native新旧对照；`remote-artifact-verification.json`为车端已有构建逐文件核对。
- `preflash/`、`install/`：同板备份、只写sector0的日志及完整512KiB回读。新完整回读SHA256为`62ce0a97c3199f5115c5c2e77cfc1cacfec3d600fbdcb0387d543c14d408b435`。
- `field/work-yaw-zero-20261029-01/`、`field/work-yaw-turns-20261029-01/`：本轮明确采集的原始bag、manifest、完整rosbag收尾。视频由用户拍摄，尚未提供文件路径，不能宣称已打包。
- `turns-analysis/frames.csv`、`summary.json`、`response.png`：逐帧实测和曲线；`alternate-comparison.json`与`guard-windows.json`区分新旧版本和上位机截停。
- 本机复跑实测分析：`./tools/noetic.sh python3 .test-results/c30d-yaw-integral-20261029/analyze_turns.py /workspace/.test-results/c30d-workfix-20261029/field/work-yaw-turns-20261029-01/motion.bag /workspace/.test-results/c30d-workfix-20261029/recheck-analysis`；末尾输出目录必须不存在。没有实时ROS master或设备参与。

当前板卡仍是3849abee候选，未自动回退。回退应恢复本轮同板10144字节92e816基线所在sector0，详见刷前备份；不要拿更早的0x10/旧发布固件代替。队友源码及提供的原始包保持原样。
