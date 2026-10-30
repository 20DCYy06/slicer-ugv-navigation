# 当前已安装版本（2026-10-29）

以队友3849abee版本为基线，保留其转向修复。新BIN为0852f4b43c5bfb21b34dec5ee727a7cbb72d4e4db982fe925b0d40a7f60a962a，10488字节。
只写sector0；完整512KiB读回和选项字节核对通过，详情见本目录last_verified_install.json及对应备份目录。
闭环PWM硬上限800‰；上位机config/teleop_params.yaml的motor_output_limit_percent为0..100，100对应最高80%PWM。
导航仍为0x16的线速度/角速度，STM编码器/IMU闭环、PID、速度上限0.8m/s和既有停机保护保留。0x18仅初始化配置输出上限。
传统0x11手动测试上限保持750‰；不把PWM上限当作电机电流/温度保护。
刷后实机零速验收通过：100%上限回读1280/1600，39帧PWM及全部CCR输出均为0，Motor/动力状态关闭、无fault。记录位于备份目录post_flash_zero.json；没有实车速度、温升或制动验收。

以下为历史说明，旧版本的600‰描述不再代表当前固件。

---

# C30D 车身线速度 / IMU角速度双闭环

工程路径：`/home/potato/Slicer_Workspace/firmware/c30d_body_closed_loop/`。

## 最新安装：PWM与连续曲线修正版

已安装2026-10-25修正版：闭环电桥时序与已验证开环一致；四轮统一比例限幅、积分抗饱和及换向复位；`body_speed`导航取消旧开环起转抬升和相关二次包络检查，保留原避障、平滑及15cm停车线。IMU实际角速度外环继续控制直行与转弯。

闭环PWM上限保持600‰，目标速度上限保持0.8m/s。实际地速与曲线跟踪仍须实测，不把静止验收视为达到0.8m/s。

当前BIN为9892字节，SHA-256 `b8a5b305c4015ced10bd961762da240834870273bdcfca391c550c5931bd12e2`；完整Flash回读SHA-256 `ece4fa455b510aa73cbf9419cebcabd0b77ee0e8f60485f56a5dee7b13789653`。只修改sector0，其他扇区和选项字节未改；IMU及真实驱动60次全零发送检查通过。备份与验收记录位于 `/home/potato/Backups/Slicer/2026-10-25/c30d_curve_pwm_E6qR5O/`。

修改说明与复测命令见 [CURVE_CONTROL.md](CURVE_CONTROL.md)。以下为此前版本记录。

本工程以现场已连通的 `c30d_vet6` SLC2 固件为基础，保留 UART1 PA9/PA10、115200 8N1、CRC、握手、编码器/PWM接线、四轮轮速PI、旧 `0x10/0x11/0x90` 接口。没有用厂家整车工程替换现场工程。Slicer ROS 驱动现已通过 `control_mode: body_speed` 接入新增 `0x16` 车身闭环，详见 [NAV_INTEGRATION.md](NAV_INTEGRATION.md)。

## 上一版本：导航 0.8m/s 配套版

- 车身/单轮目标上限均为800mm/s；PWM上限仍为600‰，没有提高。0.8m/s是配置目标，尚未实测达到该地速。
- 修正IMU启动时的I2C释放与上电等待；在启动初始化阶段分段喂狗，避免尚未进入主循环就触发现有看门狗。运行期500ms看门狗不变。
- 当前BIN：9572字节，SHA-256 `5ed04c2d22b6f151cf65b9741fd4476e95c3916a5739ca7e982fedbb84e4b3a1`。
- 完整512KiB回读SHA-256 `35a61ab71f097acc5a9a69f3697f97390ee40d278a917773eafbc26c88c8b698`；仅sector0变化，其他扇区及选项字节不变。
- 已通过固件测试、`$NAV check`、实际ROS驱动的假串口0.8m/s路由验证，以及真实串口60次全零指令检查。实际IMU WHO=0xEA、flags=7、数据年龄4ms，四轮PWM全零。没有进行地面运动测试。
- 源码及刷前双份Flash备份、刷写日志、静止遥测位于 `/home/potato/Backups/Slicer/2026-10-25/c30d_body_nav_800_g8bzSL/`。

## 2026-10-25 首次烧录与静止验收记录（旧0.6m/s版本，已被上述版本替代）

- 板卡 USB 序列号 `5CF0016377`；ROM 芯片 ID `0x413`。用户确认 12V 物理断开、Motor OFF，仅 USB 供电。
- 两份刷前完整 512 KiB 备份一致，保存于 `/home/potato/Backups/Slicer/2026-10-25/c30d_before_body_flash_OSJMIq/current_flash/`，本机另存一份。
- 实板启动时发现硬浮点 ABI 的 `main()` 序言先执行 `vpush`，早于原 `hardware()` 的 FPU 使能。已在 `startup.S` 的 `Reset_Handler` 中、进入任何 C 函数之前使能 FPU。
- 最终 BIN 为 9512 字节，SHA-256：`9c08380b6e9d94d06ddfddafdbc109b2fa4a8a4115871eacee2e4a9071bca794`。
- 仅擦写 sector 0；整片 512 KiB 回读一致，其他扇区及选项字节不变。完整 Flash SHA-256：`23bc929250577eb34cf3c03a52de470caa5cdfce759414634c002482e564af37`。
- 6 秒静止采样 151 帧，IMU WHO=`0xEA`、flags=`7`、最后样本年龄 4ms，四轮 PWM 和目标速度全零，无故障标志。未发送非零运动指令，未做带电运动验收。
- 详细记录：备份目录内 `body_install_journal.json`、`post_flash_zero_telemetry.json`。刷写前的底盘驱动容器已停止，未自动恢复；现有轮速上位机仍使用 `0x10`，不会自动切换成 `0x16` 车身闭环。

厂家工程的 ICM20948 寄存器序列和 PB10(SCL)/PB11(SDA) 软件I2C接线已适配到现有裸机工程；不依赖FreeRTOS、OLED、PS2、雷达或磁力计。源文件参考：厂家 `HARDWARE/ICM20948/ICM20948.c`、`HARDWARE/I2C.*` 和四驱 `Drive_Motor()`。

## 文件与控制链路

| 文件 | 职责 |
|---|---|
| `main.c` | 保留已有UART外层、四轮内环、引脚初始化；新增源仲裁与遥测调用 |
| `BALANCE/balance.c/.h` | 编码器线速度外环、IMU角速度外环、几何前馈、抗积分饱和 |
| `HARDWARE/ICM20948/ICM20948.c/.h` | WHO_AM_I、量程初始化、12字节突发采样、零偏与一阶低通 |
| `HARDWARE/usartx.c/.h` | 新车身指令校验/解码，固定64字节遥测小端序打包 |
| `config.yaml` | 机械尺寸、编码器CPR、轮位极性、PI/PID和IMU参数 |
| `build.py`、`runtime.c` | 仅生成宏、离线测试和ARM编译；没有刷写代码 |
| `tools/body_link.py`、`tools/monitor.py` | 上位机SLC2扩展解析与未来验收工具 |
| `tests/` | 实际C控制器/协议/IMU换算的无硬件测试 |

车身控制周期100Hz，旧遥测和新增遥测均25Hz。串口回传改为非阻塞缓冲发送，避免新增遥测拖慢控制周期。

```text
vx,wz -> 0x16 -> 车身外环 -> 四轮目标轮速 -> 原轮速PI -> PWM
                     ^            ^
        编码器四轮均速+IMU wz     各轮编码器

0x11手柄非零PWM ------------------------------> PWM（优先）
```

车身前进反馈 `vx_measured = mean(wheel_speed_ABCD)`，采用已配置极性后的有符号轮速。它是轮式速度估计，无法在四轮同时打滑时准确测量真实地面速度。角速度反馈直接取板载陀螺仪，不用轮速差假定车身已经转动。

外环计算：

```text
forward = vx_target + PID_v(vx_target - vx_measured)
turn    = wz_target * (track_width + wheelbase)/2
          + PID_w(wz_target - wz_imu)
left_target  = forward - turn
right_target = forward + turn
```

各轮按 `wheel_side` 分配。轮速超上限时统一比例缩放；外环在补偿或轮速饱和时停止累积相应积分。车身目标vx/wz都为0或控制源切换时清空积分。PI/PID默认值只是待验收初值，尚未经过实车调参。

## 配置与IMU

所有几何参数来自本工程的 `config.yaml`，编译生成 `build/generated.h`：轮径0.085m、轮距0.15m、轴距0.17m、CPR60000。CPR沿用现场1m测试使用值，不宣称已精确标定。

ABCD为右后、左后、右前、左前。闭环 `pwm_signs=[-1,1,1,1]`，`encoder_signs=[-1,1,-1,1]`，也保留 `0x12` 运行时极性配置。开环 `0x11` 的电气值仍由原手柄上位机按它自己的开环极性发送，不能混用两组符号。

IMU量程±500deg/s、±2g：陀螺仪每65.5计数为1deg/s，再乘pi/180转rad/s；加速度每16384计数为1g。启动后积累200帧静止样本估计所选偏航轴的零偏，检测到轮子或陀螺明显运动则重新积累。校准前输出遥测，但车身闭环暂不驱动；开环手柄仍可用。保持板卡静止约2秒再发第一条导航指令。

`imu.yaw_axis` 默认2（Z轴）；`gyro_signs`、`accel_signs` 调整安装方向。默认正负号**尚未在实板上验证**：必须先确认从上方看逆时针转动底盘时 `wz_rad_s` 为正。偏航轴负号错误会造成正反馈，不能用加大增益去掩盖。

IMU数据超过配置的100ms未更新时，车身控制停止并清外环积分，不锁存新的全局故障，IMU恢复后在新鲜导航指令下恢复。原电机使能、欠压、编码器异常、串口故障及500ms命令看门狗沿用现场固件。堵转/反向时间使用已有宽松配置3000ms/1000ms，不新增其他运动拦截。

## 串口协议

外层不变：`A5 5A | version=1 | type | payload_len | seq:u16LE | payload | CRC16:u16LE`。CRC16-CCITT初值0xffff，多项式0x1021，覆盖version到payload。载荷最大64字节。

| 类型 | 方向 | 载荷 |
|---|---|---|
| `0x01/0x81` | 握手 | 保留原nonce和SLC2回复 |
| `0x10` | 下行 | 原四路目标轮速i16 mm/s；仍是原轮速模式，不隐式切换车身闭环 |
| `0x11` | 下行 | 原nonce:u32 + 四路开环PWM:i16；非零时手柄优先 |
| `0x12` | 下行 | 原nonce+四路PWM极性+四路编码器极性 |
| `0x13/0x83` | 能力查询 | 保留原开环能力接口 |
| `0x14/0x91`、`0x15/0x92` | 诊断 | 保留原故障和寄存器只读接口 |
| `0x16` | 下行 | **nonce:u32、vx:i16(mm/s)、wz:i16(mrad/s)**，合计8字节 |
| `0x17/0x84` | 扩展查询 | 请求nonce:u32；回复nonce:u32、版本u8=1、WHO u8、IMU状态u8、控制源u8 |
| `0x90` | 上行 | 原61字节编码器/电机状态，载荷布局不变 |
| `0x93` | 上行 | 新增64字节编码器+IMU+车身状态 |

车身模式先完成握手和 `0x12` 配置，再每20～50ms发送 `0x16`。例如vx=0.2m/s、wz=0.5rad/s的载荷使用 `struct.pack('<Ihh', nonce, 200, 500)`。

`0x93` 的全部数值小端、有符号量为二进制补码：

| 字节偏移 | 字段 | 类型/单位 |
|---:|---|---|
| 0 | version | u8=1 |
| 1 | imu_flags | bit0识别、bit1零偏完成、bit2新鲜可用于控制、bit3最近读/初始化失败 |
| 2 | WHO_AM_I | u8，ICM20948应为0xEA |
| 3 | control_source | 0停止、1旧轮速、2手柄PWM、3车身、4原单轮校准 |
| 4 | mcu_us | u32，微秒计数，约71.6分钟回绕 |
| 8 | encoder_delta_ABCD | 4×i16，最近一个控制周期的原始脉冲增量 |
| 16 | wheel_speed_ABCD | 4×i16，极性换算后的mm/s |
| 24 | gyro_raw_XYZ | 3×i16，未减零偏的原始计数 |
| 30 | accel_raw_XYZ | 3×i16，原始计数 |
| 36 | gyro_XYZ | 3×i16，mrad/s，安装符号已应用、所选偏航轴已去零偏 |
| 42 | accel_XYZ | 3×i16，mm/s²，保留重力 |
| 48 | vx_measured | i16，mm/s，四轮均速 |
| 50 | wz_measured_filtered | i16，mrad/s，零偏扣除并低通后的控制反馈 |
| 52 | vx_target | i16，最后收到的车身目标mm/s |
| 54 | wz_target | i16，最后收到的车身目标mrad/s |
| 56 | battery_mv | u16，mV |
| 58 | status | u16：bit0使能、bit1动力电、bit2极性已配置、bit3正在执行；高8位原故障字 |
| 60 | imu_age_ms | u16，饱和至65535；无样本为65535 |
| 62 | ack_seq | u16，最后接受的控制帧序号 |

`encoder_delta` 是10ms最新采样，25Hz上报会跳过中间采样，**不要直接累加它算里程**。累计原始编码器仍使用旧 `0x90` 的 `raw_ABCD` 或其窗口增量；时间差需要按u32处理回绕。

## 手柄仲裁与上位机适配边界

非零 `0x11` 最近500ms内有效时覆盖导航。其全零帧表示释放手柄；否则超时也释放。释放后仅接管最近500ms内收到的 `0x16` 或旧 `0x10` 指令，没有新鲜导航指令就停车。发送全零 `0x10` 为兼容原 `car.close()` 的总停止，会同时清空手柄与导航请求。使能断开/故障同样清请求，不会恢复旧目标突然行驶。

原WASD/手柄的PWM帧及电气方向不变，可以继续独立使用。**现场手柄程序和ROS驱动目前各自独占同一个串口，不能同时打开。** 真正的同时仲裁需要一个上位机串口所有者，将摇杆有效输入发送为 `0x11`，将导航Twist发送为 `0x16`，共用一个握手nonce。当前ROS驱动已接入 `0x16`；独立WASD/手柄的串口独占运行方式保持不变。

## 离线编译（当前允许执行）

```bash
cd /home/potato/Slicer_Workspace
/usr/bin/python3 -s firmware/c30d_body_closed_loop/build.py
```

依赖Jetson已有的python3-yaml、gcc和arm-none-eabi-gcc。脚本先执行C原生控制测试和Python载荷测试，再生成：

```text
build/c30d_body.bin
build/c30d_body.hex
build/c30d_body.elf
build/c30d_body.map
build/generated.h
build/manifest.json
```

脚本没有设备打开、复位、擦除或烧录操作。产物限制在原sector0的16KiB容量以内，但这里只编译，不提供自动刷写入口。测试替换物理I2C和寄存器，不能证明实板接线、IMU轴向、实时调度或机械负载已验收。

## 运动验收步骤（尚未执行）

1. 保持Motor OFF，板卡静止约2秒。运行下方默认监视命令；它只握手/查询，不发送非零运动帧。确认WHO=234(0xEA)、imu_flags低3位为7，静止wz接近0，加速度模长约9.81m/s²。
2. Motor OFF时手推整车向前：原始delta期望A负、B正、C负、D正，四路wheel_mm_s和vx_measured都应为正。反向手推符号相反。若只手转单轮，其他轮应保持近零。
3. Motor OFF、零偏完成后，从上方看缓慢逆时针转整车：wz应为正；顺时针应为负。方向不符先改 `imu.gyro_signs` 并重新编译，不能直接开始角速度闭环试转。
4. 首次运动验证用四轮稳定架空且可缓慢转动底盘的支架，先给2秒低速纯转向命令。正wz应使左轮后退、右轮前进；结束/中断后全部停止。车身被支架固定不转时，IMU为0，补偿会升至配置上限，这是正常误差响应，不能由此判断地面性能。
5. 确认方向正确后，在同一低速命令期间缓慢正向转动可旋转架台：IMU正转速升高时左右轮差速应减小；反向转动时差速应增大。仅操作支架，勿接触旋转车轮。若趋势相反立即停机，核对IMU与轮位极性。
6. 验证释放手柄或断开输入后500ms停车；再由唯一串口进程持续发送导航帧，检查手柄覆盖和释放接管。最后再做地面低速调参。

```bash
# 默认监视，不发非零运动命令
/usr/bin/python3 -s firmware/c30d_body_closed_loop/tools/monitor.py --duration 15

# 仅在将来的架空验收条件满足后手动执行；本次未执行
/usr/bin/python3 -s firmware/c30d_body_closed_loop/tools/monitor.py --duration 2 --command 0.0 0.2
```

监视器收到旧固件时会报扩展接口超时，不能据此认定IMU损坏。当前实板仅通过静止验收，运动命令须等待现场架空和急停条件确认，并先检查IMU轴向及轮位极性。
