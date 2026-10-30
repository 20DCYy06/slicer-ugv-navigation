# 厂商资料补充：MCU低速闭环、转向与停车

日期：2026-10-24。资料来源：D:/Industrial_Cup/WHEELTEC L150Pro 智能小车资料_V2.1_2026.04.20。

**用户现场确认：C30D V2.0，轮胎外径约85 mm。** 本次只读核验厂家源码、芯片手册和原理图，补充实施任务；未改运行源码、未刷写、未连接串口或发送运动。参考副本保存在本地，原路径及SHA256见[证据清单](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_manifest.json>)。

结论：资料有直接价值。此前“当前停零是不是滑行”已有芯片真值表和V2.0接线依据；60000计数有新版GMR源码依据；厂家四驱转向模型与现配置不同，但轮胎65 mm不符合用户报告，不能照抄。仍应先调通19 mm/s低速闭环，随后验收直行和停车，外围修复不提前。

## 1. 最重要的新证据：当前零输出是滑行，厂家零输出不同

[AT8236手册](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_docs/AT8236.pdf>)第5页的真值表已经视觉核验：

| IN1 / IN2 | OUT1 / OUT2 | 芯片功能 |
|---|---|---|
| 0 / 0 | 高阻 / 高阻 | 滑行、休眠 |
| 1 / 1 | 低 / 低 | 低侧短接刹车 |
| PWM / 0，或0 / PWM | 驱动与高阻切换 | 快衰减 |
| 1 / PWM，或PWM / 1 | 驱动与刹车切换 | 慢衰减 |

[V2.0原理图](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_docs/C30D_V2.0_schematic.pdf>)第3页明确四路为AT8236，接线与实际固件控制引脚吻合。因此现在可把以下内容列为**确定的软件行为差异**：

- 当前已刷[main.c:50](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/review_20261024_915d503/DEPLOYED_FIRMWARE_main.c:50>)的stop()将所有CCR写0；[main.c:308](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/review_20261024_915d503/DEPLOYED_FIRMWARE_main.c:308>)零轮速也写双零。按V2.0图纸对应的是滑行，不是主动电气刹车。现场实际停车距离还需测量。
- 当前非零轮速[main.c:319](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/review_20261024_915d503/DEPLOYED_FIRMWARE_main.c:319>)已经进入test_pwm的慢衰减路径；这项桥写法修改确实部署，不能把B7的新桥失败再次归因于“还没接入”。
- 厂家[balance.c:304](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/balance.c:304>)的Set_Pwm(0)写双16799；其ARR=16799，相当于几乎全周期双高，保留一计数的共同低电平。它与当前全周期双低的停车行为不同，不能只因为传入数值都是0就认为等价。
- 厂家常规目标速度为0时仍进入增量PI，并非直接清积分停桥；而Turn_Off先写零后，主循环else又调用Set_Pwm(0)，覆盖为上述近双高。厂家代码本身还有PWMC1重复清零等问题，**不能整段当成已验证停车实现复制**。[balance.c:268](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/balance.c:268>)、[381](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/balance.c:381>)、[434](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/balance.c:434>)。

**给Codex的直接行动：** 在低速闭环可用后，把“普通0x10零速度时滑行与低侧刹车的短段对照”列为停车修复的第一项，不再只调导航容差或PI。候选必须覆盖命令处理、control()和stop()完整路径，防止只把接收零指令时写成双高，下一周期又被!have_command分支改回双低。正常停车与fault/禁用/失联如何退出要逐项验证，不能直接把共享stop()全部改双高。先离地验证电平与轮停，再在既有低速下对照余程、电流/电压；双高刹车也不等于机械抱闸或位置锁定。

现有duty遥测为0不能区分00与11。使用现有寄存器查询核对停后CCR，并保留主机接收时间；必要时测实际引脚。无需先扩建全套诊断协议。

## 2. 电桥硬件边界现在能具体到V2.0图纸

[V2.0资源表](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_docs/C30D_V2.0_resources.pdf>)和原理图对应如下：

| 端口 | 芯片IN1 | 芯片IN2 | 编码器定时器 |
|---|---|---|---|
| A | PB9 / TIM11_CH1 | PB8 / TIM10_CH1 | TIM2 |
| B | PE5 / TIM9_CH1 | PE6 / TIM9_CH2 | TIM3 |
| C | PE11 / TIM1_CH2 | PE9 / TIM1_CH1 | TIM4 |
| D | PE14 / TIM1_CH4 | PE13 / TIM1_CH3 | TIM5 |

A/B/D芯片OUT1与板上xOUT2网名相接，C接xOUT1。结合两种writer的实际参数，当前319行A/B/D反号、C不反号保持原极性，有硬件资料支持。**这不改变本车已实测的A右后、B左后、C右前、D左前关系。** 厂家四驱源码把A/B视为一侧、C/D视为另一侧，不能连同符号表一起照抄到本车。

其他有用边界：

- V2.0第2页SW3经电阻接PD3，是给MCU读取的使能开关；AT8236没有独立EN/nSLEEP输入。SW3不直接切断电机VM。主电源开关SW4在第3页另一路。因此现有程序响应PD3有真实作用，不属于凭空增加的限制。
- 第3页VREF接3.3 V，ISEN电阻标R250。按AT8236第6页公式，**名义驱动斩波阈值约3.3/(10×0.25)=1.32 A**。这是图纸推导，需实物器件/工作波形证实；不是已测电流、不是刹车电流已受同等限制，也不是板卡持续带载能力证明。不能拿芯片首页“6 A峰值”当本车能力，更不能说600‰已有硬件保证。

- [C30D随货手册](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_docs/C30D_shipping_manual.pdf>)PDF第2页（印刷页2–3）另写明：电机四路总持续电流≤5 A、单路≤1.3 A。这与图纸推导的约1.32 A数量级吻合；手册未注明精确板卡修订号，不能据此免除带载测量，也不证明B7已经触发硬件限流。

- AT8236第6页给出双低持续超过1 ms进入休眠；任一输入高至少5 μs，约1 ms后恢复。旧10 kHz快衰减PWM低于5%时，高脉冲不足5 μs，只能列为不满足文档保证唤醒条件的风险，不能断言必不醒。**当前新桥非零一路恒高，不能用这个旧路径风险解释B7约450 ms后失败。**
- 旧PWM/0也是芯片明确支持的快衰减模式，并非非法真值表。此前若说旧桥“原理上一定错误”，结论过强；可验证的是衰减方式和低占空响应差异。

这些信息用于缩小测量位置，不应派生新的运行管理系统。

## 3. 编码器与几何参数：保留85 mm，分清资料版本

当前GMR工程的[robot_select_init.h:56](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/robot_select_init.h:56>)、[66](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/robot_select_init.h:66>)、[101](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/robot_select_init.h:101>)与[robot_select_init.c:80](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/robot_select_init.c:80>)给出：减速比30、编码器500线、四倍频，即**60000计数/轮圈**。这为现配置提供了型号相关依据；仍需确认本车电机/编码器确为对应GMR版本，并用多圈计数核验。

旧[L150Pro使用手册](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_docs/L150Pro_manual_2024.pdf>)PDF第1页是2024版，第19页写旧霍尔13×30×4=1560。资料目录的2026日期不能使旧手册自动变成新版；**不要改成1560，也不要把1560与60000混用**。源码目录中的旧control.c和system.c.orig也不等于当前执行入口。

厂家FourWheel配置为65 mm轮径、187 mm轮距、173 mm轴距。[robot_select_init.c:30](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/robot_select_init.c:30>)、[robot_select_init.h:40](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/robot_select_init.h:40>)、[50](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/robot_select_init.h:50>)、[88](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/robot_select_init.h:88>)。

**用户确认轮胎外径约85 mm，因此本车不能按厂家65 mm修改。** 现有85 mm有现场依据；精确受载轮周长可在闭环验收时校准，不把测量轮径升级成长时间前置工程。资料也不证明四个实物轮的有效周长完全相同。

转向则有一个需要实测收敛的差异：厂家[balance.c:155](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/balance.c:155>)四驱逆解采用

    左右轮速 = Vx ± wz × (轮距 + 轴距) / 2

按它的标称尺寸，转向臂为0.18 m，wz=0.25对应±45 mm/s；现配置以0.15/2计算，对应±18.75 mm/s。**这是模型计算对照，不是应直接改成45的指令。** 先解决现有19低速在离地状态都不能起转的问题，再量本车轮距/轴距，并用外部实际转角拟合四轮滑移转向的有效参数。不同地面滑移会影响有效转向系数，厂家“轮距+轴距”也不是通用精确物理定律。

此差异可能导致同一wz请求给出过小轮速，但不能单独解释B7离地闭环故障。不能用提高角速掩盖低速控制未调通。

## 4. 厂家确有MCU轮速闭环，但不能直接抄PI参数

[WHEELTEC.uvprojx:615](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/USER/WHEELTEC.uvprojx:615>)至625对应system.c、balance.c、robot_select_init.c；当前项目不编译旧control.c。实际循环[balance.c:193](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/balance.c:193>)以100 Hz运行，反馈[1095](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/balance.c:1095>)以m/s计，PI输出以约16800周期计数。当前[system.c:77](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/BALANCE/system.c:77>)默认Kp=Ki=700，[usartx.c:286](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/reference_source/HARDWARE/usartx.c:286>)允许APP更新，不能保证现场厂家固件曾用这个运行值。

厂商增量式PI为：

    u[k] = u[k-1] + 700×(e[k]-e[k-1]) + 700×e[k]

仅换算到我方1600周期、mm/s和连续时间积分单位，约对应Kp=0.06667、Ki=6.6667/s；我方为Kp=1、Ki=4/s、FF=3。**700不等于“比我方增益大700倍”**；归一化后，我方Kp约为其15倍、Ki为其0.6倍，且另有前馈、滤波、限幅、同向输出限制和失速逻辑，整套响应不同。

厂商与我方名义PWM频率都是10 kHz，不能只比较16799和1600就认为频率差十倍。厂商源码没有证明±19 mm/s必能可靠起转，也没有提供可直接用于本车的分轮启动/维持PWM表。因此原任务中的实测前馈、必要启动补偿及失速判据配合仍是低速修复主线，不必移植厂商RTOS或控制框架。

## 5. 对现有Codex任务的最小增补

按此顺序落实，仍不把外围修复提前：

1. 固定本车V2.0、约85 mm和现用端口映射。60000按GMR依据保留并做一次多圈核验；不要改65或1560。
2. 继续直接0x11/0x10对照，测起转与维持，按数据修前馈/PI/启动补偿/失速判据。把厂商PI作为量纲校核参考，不作为无需验证的参数答案。
3. 低速闭环能可靠运动后，给出普通停车00滑行与11刹车的最小候选差异。离线核验每个后续控制周期不会覆盖停车意图、零指令不会留下单边驱动，超时/fault/禁用路径符合设计；先离地再低速地面测余程。不要把厂家Turn_Off和Set_Pwm整段搬过来。
4. 直行与停车采样保留外部标记、编码器、串口时序、duty及停后CCR。普通停车余程改善也不能替代到点容差与定位误差核验。
5. 最后测转向系数。只在本车尺寸/实际转角支持后修改几何参数，保留±19作为低速能力验收点，避免用新的大轮速绕过它。

软件之外还缺的关键观测已经缩小：本车编码器每圈实际计数、轮距/轴距和实际转角、起转/维持输出、零速后的真实余程；如出现驱动输出与运动不符，再测对应电桥/电机端波形及带载电流。现有资料仍不能证明电机没有机械问题、板上元件与图纸完全相同、历史超调全部来自滑行，或任何候选已经实车通过。

[换算记录](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/vendor_evidence_20261024/derived_values.json>)只记录上述算术推导；[主任务单](<C:/Users/LoveM/Documents/ChatGPT/中航工业杯 2/给Codex_915d503最小修复任务.md>)继续规定实施与验收顺序。
