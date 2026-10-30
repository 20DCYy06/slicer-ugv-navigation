# Jetson odom 测试准备交付（2026-10-25）

用户授权助手完成车端准备后，已更新 `/home/potato/slicer_ugv_ros1` 的源码、工具及 `install`。ROS控制来源 **`f4bd9ee`**，包含另行批准的最小LIO补丁；采集器TF兼容修复 **`598c899`** 也已同步，最终重装包以598c899为基线。未更新MCU、未修改队友工程。车端目录没有Git元数据，这里的提交号指源码包来源，不冒充车端Git HEAD。

## 已实际完成

- Jetson主机 `aarch64`，本地运行镜像 `industrial-cup/nav-slicer:20261023-dwa` 为 `arm64`。已在该主机断网、无设备映射的容器中执行 `catkin_make --force-cmake install`，固定安装到 `/workspace/install`。
- driver及Guard安装入口正文、四个底层helper与本次源码一致；wheel_speed配置文件现已存在。实际运行环境解析三个我方包到 `/workspace/install/share`，Slicer定位包来自只读 `/slicer/catkin_ws/src/slicer_bringup`。
- driver Launch解析确认 `control_mode=wheel_speed`、`publish_tf=false`、`observe_motor_registers=false` 和正确的安装配置路径。导航Launch解析确认 `/slicer_nav_arc_guard/lio_topic=/lio/odom`、`lio_timeout=0.5`。
- `/dev/c30d` 当前指向 `/dev/ttyCH343USB2`；通过已有容器读取USB sysfs身份为 `1a86:55d4 / 5CF0016377`。只读取sysfs，没有打开串口。
- 原A由助手SIGINT结束，更新后A/D由用户启动。定位整链曾退出，助手核对进程/ROS图后，在现有环境中以后台容器 `ugv-sensors` 恢复原 `sensors.launch`；无串口映射，队友树只读。助手未启动底盘driver，未执行release/hold、速度、Action目标或刷写。

## 验证结果

| 范围 | 结果与边界 |
|---|---|
| ARM64 driver单元检查 | **50/50通过**，无硬件设备、无ROS master |
| ARM64采集/分析/Shell检查 | **51/51通过，无跳过**，其中真实rosbag读写/CLI检查使用临时TEST_ONLY数据 |
| 实际生产Guard的LIO失效检查 | **6/6通过**；LIO停更、冻结header、旧header等失效时归零，raw/TF/grid原超时行为保持 |
| 原Guard传输回归 | **11/11通过**，覆盖补偿启用/禁用，夹具显式提供独立合成LIO |
| DWA→Guard→Mux软件闭环 | **2/2通过**，保留实际非零输出、到点及取消归零断言；理想模型不代表实车 |
| 当前真实输入 | LIO9.98Hz、scan9.98Hz、融合odom18.88Hz、轮odom18.48Hz；时间戳均推进，无回退/重复/未来时标；TF唯一动态owner为EKF，静态owner为slicer_geometry |
| wheel_speed零速接口 | 运行provenance和模块哈希正确；独立PWM/编码器符号及CONFIG nonce匹配、configured/ready为真；176条MCU遥测tick连续推进，故障位均清零；129条实际录到的发送均为0x10四零，174条遥测接受序号匹配，无PWM回退 |
| 当前保持/规划 | Mux startup held、teleop不活跃、最终Twist为零；前方0.5m GetPlan返回22点，未发送Action目标 |

早期driver停止时的 `sensors-before` 采样与随后定位缺失的 `live-started` 均保留，不与当前全部有效的 `live-ready-01` 混为通过。零反馈与零指令不代替外部物理停稳观察。

Guard安装SHA256：`24357146072ad12f092cfc507e5a205dff7067466987ec062d649b5358dbf683`。统一Guard参数SHA256：`8a5f3522e89bf36286f0276e4e393afeb806982ee0cb0358afb3031146700fa2`。

## 证据与保留原文件

WSL目录：`.test-results/jetson-odom-update-20261025/`；车端同名目录位于独立工程根下。

- `before.json`、`live-before.json`：更新前源码/进程及held/零速观测。
- `stop-and-preserve.json`、`pre-update-source.tar.gz`：仅保存本次会覆盖的现有文件及原缺失清单，没有扫描历史归档；车端另有 `pre-update-extra-tools.tar.gz` 保存随后纳入更新的测试工具旧文件。
- `arm64-install.log`、`arm64-install-result.json`：实际ARM64安装和安装内容核对；初轮工具检查因probe覆盖PYTHONPATH跳过一项rosbag集成，原日志保留。
- `arm64-tools-final.log`：保留正常ROS Python路径后重新执行51项，全部通过且无跳过。
- `readonly-install-check.json`：正常overlay下的包路径、纯配置加载、USB sysfs身份和两组Launch参数解析。
- `sensors-before/`：本次明确采集的真实5秒观测及bag；没有历史bag。
- `live-started/`、`live-ready-01/`：用户启动A/D后的定位缺失记录与恢复后的8秒真实采样，后者findings/decode_errors均为空；`getplan-only.log`、`zero-interface-check.json`保留非运动规划和零速协议证据。
- `serial-owner-rdev.json`：通过只读/proc字符设备号匹配，确认实际driver持有该端口；部分其他用户进程权限不可读，未将此声称为全系统可见性证明。host fuser因Docker设备命名空间未报告owner，不据此推定串口无人占用。
- `lio-production-green/result.json` 与 `.test-results/odom-preflight/lio-applied-fixtures/summary.json`：生产补丁的隔离行为与回归证据。

首轮保留操作发现车端不是Git仓库，在停止进程和覆盖源码前退出，随后改为保存确切覆盖文件；记录为 `preservation-initial-error.txt`。初轮只读probe错误覆盖PYTHONPATH导致roslaunch子进程无法导入，修正probe后通过，记录为 `readonly-check-initial-error.txt`。这些没有被归为底盘或软件控制故障，也没有抹掉失败记录。

TF采集修复前的原始兼容错误也保留为 `collector-tf-red.log`（typed TFMessage没有AnyMsg的_buff）；修复后增强集成断言与真实ARM64采样均通过，TF边及owner完整。只更改采集工具，未改TF或运动链。零速bag断言第一次Docker未转发stdin，未执行任何断言、不计通过；改用-i后获得非空JSON，说明保留在 `zero-interface-initial-empty.log`。

当前源码包：`odom-ready-598c899.tar.gz`；SHA256 `16bfbc745185a05b7d210de8d4e38764c0ea71a90c83610216a7ca3f536e08c9`。

## 还需要现场完成

当前三链已运行，无需重开；以上真实输入与零速检查均已完成。中途用户换电重启，软件安装与串口规则均保留；助手恢复 `ugv-sensors` 与 `ugv-navigation` 两个既有Launch，用户重新启动D。`post-reboot-restore.json`、`post-reboot-ready/` 和 `post-reboot-getplan-only.log` 保留新boot的恢复、全输入及22点规划证据，不把旧boot采样当当前状态。

按[一页操作单](ODOM_FIELD_CHECK.md)从第2节进入离地低速测试，保留D终端与后台定位/导航。没有新增开机服务、自动重启或管理框架；两个后台容器只承载原有ROS Launch。

通过这些检查后，非零速度、release、取消/hold及真实运动由现场用户执行。合并固件 `956eca3` 尚无本次非零验收；先离地前进/纯转/停止，再做地面短直线与导航。0x10低目标不限制MCU PWM；0.5s LIO超时是软件出口条件，不是物理停稳时间。真实距离和停车余程仍需同一车体固定点测量，不能用Action成功或轮速零自证。
