# C30D 闭环电桥输出候选：离线验证和构建记录

状态更新：用户随后明确批准了该镜像，已完成新鲜双读备份、烧写完整回读及CONFIG/四零核验；实际结果见[烧写记录](C30D_BRIDGE_FLASH_2026-10-24.md)。带动力闭环仍未通过，本文保留烧写前的构建与审查依据。

本候选独立于已完成烧写的三处 dt 修复。现场 [B3/B4 对照](../field/WHEEL_SPEED_POWERED_RESPONSE_2026-10-24.md) 表明：0x10 请求经过 Guard/Mux 进入 MCU，真实 CCR 非零，四轮仍不动并触发失速保护；同一现场链路使用既有 0x11 PWM 模式则产生用户观察到的前进及四轮编码器变化。后者在首个非零发送后约 0.38 秒已有四轮明显计数变化，不能仅用“开环运行时间更长”解释差异。两种模式的输出轨迹也不同，因此桥表达仍是待实物验证的单变量假设，尚不是已确认的唯一根因。

## 唯一生产逻辑变化

候选 patch（原归档未随仓库收录：`../../firmware/c30d_vet6/candidates/closed_loop_slow_decay.patch`） 仅在非零目标的闭环控制末尾替换一次调用：

```diff
-  pwm(i,duty[i]);
+  /* Preserve pwm() physical polarity: test_pwm uses opposite signs on A/B/D. */
+  test_pwm(i, i==2 ? duty[i] : -duty[i]);
```

复用同一固件中已用于 0x11 的 GPIO/CCR writer，不发送 0x11，不改变 direct_mode，不引入主机 PID。A/B/D 的输入符号需要翻转，C 不变，才能保持原 pwm() 的物理方向语义。两个 writer 的差别是驱动间隙使用 00 或 11；本候选改变电桥表达，实际电流、转矩和 PI 响应仍需现场验证。

PI/前馈、符号配置、dt 修复、固定 0.5 滤波、目标上限、保护条件和计时均保留。闭环 duty 先被限制到 ±960 ticks，既有 helper 的 1200 限幅不会介入；允许一侧 CCR=1600 恒高，有效差分仍不超过 960/1600=60%。这不等于实际电流或转矩上限。零目标仍走原 pwm(i,0) 的 00，fault/Watchdog/stop 路径保持；0x20 标定和原 0x11 路径均未改。

工作区 main.c 仍为已烧入的 dt 版本。候选只在临时测试副本和独立构建目录应用；旧 timebase.patch、原测试及 fresh_flash.py 均保持原样，不能把新镜像交给旧脚本冒充三处 dt 镜像。

## 实际执行的离线检查

无设备、network none 容器编译和执行真实 C 控制路径，模拟寄存器、时钟与 CNT；没有电机模型、跟踪成功或实车效果推断。

- RED：原 96 项全部通过，26 个新增桥波形断言按预期失败，合计 99/125；原始失败完整保留。
- GREEN：125/125 通过，包含原 96 项，以及经真实 0x12/0x10 的当前独立 PWM/编码器符号、四轮各自正负 19/100/600mm/s、混合零目标和差分限幅检查。
- 两组各 100 周期轨迹：积分、滤波、duty、实际速度、保护累计和故障状态逐字段一致；各 67 帧 CCR 按预期改变，不能宣称硬件输出与旧版相同。实际 writer 参考匹配 100/100，独立常量通道表另行核对，避免只用 helper 自证。
- 独立只读代码审查未发现实质问题。首次容器临时目录不可执行导致启动失败；改为明确 exec 的临时挂载后完成上述 RED/GREEN，执行说明保留在 summary.json。

证据：`.test-results/c30d-bridge-candidate-20261024/{red,green}/`，含实际被测源码、原始 JSONL、编译日志、运行结果和哈希。原 runner 仍只允许三处 dt 变更；新 runner 不放宽它。

## MCU 镜像构建

在 Jetson ARM64 主机使用已有 `arm-none-eabi-gcc 10.3.1 20210621` 交叉构建 STM32 镜像，无串口访问。独立目录为 `/home/potato/slicer_ugv_ros1/.test-results/c30d-bridge-candidate-20261024/offline-build/`，未改已部署文件或队友树。

同一套输入先重建当前 dt 源码，BIN 字节哈希与已烧入镜像完全一致，再构建候选。两个构建均 exit=0，generated.h 逐字节相同。候选 BIN 5944 字节，小于原流程扇区 0 的 16KiB；大小检查不是已执行烧写。

| 输入或产物 | SHA256 |
|---|---|
| 当前 dt main.c | `694ef6fdb9d713891be99ba235d9db8635889de44d87166b33095623452da79c` |
| 候选 patch | `103a26228ad84a4d551af859923b83d8b70548f1dc48bfd8b702952d4700fb69` |
| 候选实际 main.c | `d4b32cefc24a664dcae52b1f49d25f579fa5b5b255113ed3fe27a5887b000b88` |
| 原版重建 BIN，5872B | `b01c1c1ba96dba492fd49172b2d216a39b0d030fa87c53134cba29c2eeae48f3` |
| 候选 BIN，5944B | `fb0fee03a4e652593c75be6e2f9f3c495bf2685b0c3265552554e2ead04f3e2d` |
| 匹配的 slicer_config.yaml | `f52b4a975f999fa500a81fa18dd205f019267ee4c97fafb6d27c2aa57c3fa12d` |
| vendor_original.zip 构建输入 | `2f279e7dc64adb7a00081011ee2732a856fb4cbd20e8eb7888230e79d51d7b2c` |
| 两版 generated.h | `fadcf7a9f681577b5de20ab9f595e8333f9c7295efbe7ab07323c882c09c4996` |
| 取回的 build-artifacts.tar | `16f9b500a3cdc464d035b9e8f671a766e34d419740a4c9ccb1d3f453af66a186` |

关键生成参数未变：CONTROL_MS=10、WATCHDOG_MS=500、COUNTS_REV=60000、MAX_MM_S=600、ENCODER_DELTA_LIMIT=2247、PWM_PERIOD=1600、PWM_CAP=960、TEST_PWM_CAP=1200、KP=1、KI=4、FEEDFORWARD=3。

本地构建证据位于 `.test-results/c30d-bridge-candidate-20261024/arm-build/`，含两套源码、配置、generated.h、ELF/BIN/MAP、manifest、编译器版本和日志。构建输入包和远端构建脚本另存同级；所有取回产物逐项实算核对 manifest。

## 烧写前的授权边界与后续验收

准备阶段完成的是候选源码差异、实际 C 离线验证和 MCU 镜像构建，当时未烧写。随后按用户对新候选的明确授权完成了烧写回读与无运动核验；这些新证据单独记录，没有套用原dt固件的验收。带动力闭环和新固件下ROS整链仍待现场验证。

用户原烧写授权限定三处dt修改；新镜像经单独决定，用户明确回复“同意烧写，所列物理条件全部已满足”。实际操作重新建立当前dt镜像的新鲜完整双读备份，只写扇区0、完整回读，不改选项字节；旧工具保持，另用固定新镜像指纹的副本。物理条件由现场确认，没有从Motor OFF猜测12V断开。

若新镜像烧写回读和无运动接口通过，才由用户执行同链路 100mm/s 对照，随后验证回零、反向、命令中断停止和负载响应；不自动 release 或启用导航。它不保证 19mm/s 起转，也不解决历史停车余动、里程计误差或 NO PATH 的全部问题。进入导航 C 前还保留既有 CLI Ctrl-C 偶发失败的独立待办，详见现场记录。
