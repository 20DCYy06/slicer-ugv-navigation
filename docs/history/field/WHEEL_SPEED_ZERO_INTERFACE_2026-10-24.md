# wheel_speed 无运动 ROS 接口验收（2026-10-24）

A阶段的配置与零速接口检查已完成。用户启动和停止真实driver；助手只做隔离构建、只读订阅与分析，没有发送release、hold、非零速度/PWM、标定或导航目标。车辆全程保持用户确认的架空、Motor OFF、12V电机动力线物理断开条件。未验证带动力闭环、物理符号、Guard避障或导航效果。

## 接线与执行位置

本次只运行用户启动的既有Mux和独立driver：`/slicer_velocity_mux → /cmd_vel → /slicer_c30d_driver → C30D`，Mux保持锁定，最终Twist持续零。没有任何运动命令源；定位、规划器与Guard不在本次测试范围。后续非零仍须接`Guard raw → Guard → Mux → 唯一driver → Watchdog`，不能用本次零速接线作为绕过Guard的运动接口。

控制板仍为USB `1a86:55d4 / 5CF0016377`，路径`/dev/ttyACM1`。按设备major/minor而非Docker设备节点inode检查，在当前UID可读`/proc`范围仅观察到本driver一个FD；独占传输使用已审`TIOCEXCL`。另有9个PID拒读，`sudo -n`需要现场密码，未索取密码或扩大权限。因此本记录不是全主机所有权限范围的排他扫描证明。用户已确认其他驱动和串口程序停止，ROS图只见一个driver、一个最终Twist发布者。

没有覆盖车端当前源码或`install/`。Mux继续使用首个隔离安装；新版D使用第二个测试根：

`/home/potato/slicer_ugv_ros1/.test-results/wheel-speed-config-arm64-20261024/source`

该目录自己的`tools/slicer_shell.sh`将它挂为`/workspace`，加载自己的`install/`；队友`/home/potato/Slicer_Workspace`只读挂载。driver provenance已核对`mock=false`、`control_mode=wheel_speed`、实际模块路径及hash。

## 第一次记录及最小诊断修复

第一次真实零速运行约102.862秒，记录1943次成功0x10四轮零发送、2578帧推进遥测、无fault、软件duty全零，匹配到MCU接受零命令。预先订阅的bag仍缺最前18个诊断事件，没有捕获0x12/配置ACK，故当时没有将完整A标为通过。原bag及不完整状态均保留。

原bag含两次Mux启动，两者之间34.91秒观测空档在driver启动前；driver窗口内仅后一个实例。不能把整个bag称为无间断运行。临时在线观察器在JSON序列化时仍有回调追加，顶层事件数量与数组数量有差异；原文件保留，正式统计均从关闭后的静态bag重新计算。

修复提交`11f92178960a39b49fa6760affe989eb6b581a7b`（隔离开发提交`ced7e86`，ROS源码树逐字相同）只在原driver配置成功后增加13行诊断逻辑：将实际会话nonce、ACK、发送配置符号、configured及接收tick写回已有`~provenance/mcu_configuration`，同时发诊断事件。未更改控制、串口顺序/重试、默认模式、固件或门槛。没有该快照，晚订阅会失去一次性配置应答，无法事后核对请求/接受关系。

快照的nonce和tick为十进制uint32字符串，避免ROS1 XML-RPC的int32限制；读取时用`int(...)`。ACK只证明接受配置，不是物理方向读回。配置失败不产生成功快照；参数保存或诊断失败不阻止零命令与关闭。

实际RED为42项中4个预期失败，GREEN及主代理独立复核42/42通过，含高uint32、晚订阅、错误/丢失ACK、fresh configured缺失及诊断/参数失败路径。更新版ARM64完整无设备、`--network none`测试与安装通过：104个JUnit条目、0 errors/failures/skipped；run为`run-Nb7gjdj9`。69份输入构建后hash均与清单一致。实际容器`aarch64`，镜像ID与[前次ARM记录](WHEEL_SPEED_ARM64_2026-10-24.md)相同。

| 项目 | SHA256 |
| --- | --- |
| 新driver源码/实际安装 | `1e46d9bf263ca5e785851276f968099c396a9ff45f034dcb0ef073ed393e214c` |
| wheel_speed参考配置 | `d9d0ff6bc99eb614c2700444d58ee9355acc5b890b80d4fc58e03dd21f370f04` |
| 更新版69文件源码tar | `4f8bdb111766dc2b627cf3cfbb070e03555da7a2158166d3abd3b4154dd0cf0e` |
| 第一次原bag，6586087字节 | `57028e510e09747bf9ad7c3772e3bfa41dbb2f353ba4593c81539d2740712975` |
| 第二次原bag，8195244字节 | `1c1e1b479776d1d2d410cb0026ff4b7691dcaba81e687834fe098adcceb8beb7` |

## 第二次实际配置与零速证据

本次driver实例为`30be3dd3f039407ebea5abbff3316b41`，与第一次不同。实际回读快照：

```json
{
  "request_nonce": "2894679481",
  "ack_nonce": "2894679481",
  "accepted": 1,
  "pwm_signs_abcd": [-1, 1, 1, 1],
  "encoder_signs_abcd": [-1, 1, -1, 1],
  "configured": true,
  "mcu_tick_us": "1628000000"
}
```

这是已审driver在实际`configure()`成功后保留的本实例主机证据；与随后配置有效、推进遥测交叉核对，不是假定MCU回显了所有符号。该nonce实际超过INT32_MAX，证明现场ROS参数写入也保留了完整值。

第二bag仍缺前19个非锁存事件，原始ACK事件没有留在bag中。快照解决了本实例的可回读证据，但没有追补第一次的缺失记录，也不能将第二bag说成包含完整启动期的原始串口抓包。

| 已观测条目 | 实际结果 |
| --- | --- |
| driver诊断窗口 | 136.044秒，11247事件，事件20至11266中间无缺口 |
| 0x10 | 2573次成功发送，四轮目标全零；观察窗口仅见kind16，无0x11/PWM回退 |
| MCU遥测 | 3407帧tick推进，无重复/倒退tick；有匹配已接受零命令和新鲜遥测证据 |
| MCU状态 | flags始终10：configured、未置fault/EN/动力；软件duty及测得轮速全零 |
| Mux/最终Twist | 4533条状态均held、teleop_fresh=false、selected_source=zero；4533条Twist六分量全零 |
| ready | 2574条全false，符合本次Motor OFF/断动力条件 |
| 电压 | 板端读值4.254–4.423V；不是新动力电池实测电压 |

用户按Ctrl-C停止D后，节点及可见串口owner已退出，最终Twist只由原Mux发布且没有driver订阅；Mux仍startup held，未执行release。bag未保留driver关闭时十次零包的完整诊断，因此不以它证明关闭包数量或地面停车距离。

## 证据与后续

本地第一次原始证据：`.test-results/wheel-speed-arm64-20261024/interface-a-20261024-01/`。第二次及更新版ARM证据：`.test-results/wheel-speed-config-arm64-20261024/`；其中`interface-a-20261024-02/`保留原bag、provenance、静态分析、实际owner范围、最终状态和未扩大权限的结果；`config-snapshot-tests/`保留RED/GREEN。第二bag车端原件仍在首个隔离树的`.test-results/interface-a-20261024-02/`，没有移动或覆盖它。

固件三处时基修复、实际C离线测试、烧写回读、固件零速检查、新driver ARM软件验证及本次A接口检查分别记录，不能合并为“实车闭环已通过”。driver通用提示尚不能自行鉴定固件hash；此前实际ROM回读已证明板上镜像，见[固件记录](../firmware/C30D_TIMEBASE_2026-10-24.md)。

B仍待现场确认ABCD实际位置、PWM/编码器方向、轮端计数和mm/s单位、起转/维持/回零/换向/掉命令停止及地面负载。候选符号和几何值不是实测标定；0x10低目标不保证低PWM，软件duty不是CCR/转矩，一帧零不是停稳证明。B通过后才执行C短距、纯转、取消/hold、停后误差和基础障碍验证。本次没有RTK、主机PID、自动恢复或运行管理器。
