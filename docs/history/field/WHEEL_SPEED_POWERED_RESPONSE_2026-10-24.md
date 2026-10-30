# Slicer wheel_speed 带动力响应记录

最新进展与条件更正：B5前进和B6纯转由用户确认均在地面；最后B7由用户随后明确确认“本轮是离地”。B5的100mm/s前进已响应；B6、B7的±19mm/s纯转均未形成持续起转并锁故障，地面摩擦不能单独解释B7。现场测试已结束，完整交接见[离场审查交接](FINAL_REVIEW_HANDOFF_2026-10-24.md)。未宣布整个B或导航C通过。

2026-10-24，用户执行Guard raw → Guard → Mux →唯一driver→MCU Watchdog链的速度测试。助手只订阅、保存和离线分析，没有发送实车release、hold或非零指令。B1至B4阶段未改固件/PI；B5、B6使用后来明确授权烧入的桥候选。早期架空是操作指引，不能据此证明各次实际负载；B5、B6按用户后续明确答复记为地面。

配置与零速接口A检查已完成，见[零速记录](WHEEL_SPEED_ZERO_INTERFACE_2026-10-24.md)。本次B检查未通过，不能进入导航验收C。

## 实际结果

| 项目 | 50mm/s | 100mm/s对照 |
|---|---|---|
| 用户执行 | 2秒发布进程命令执行两次 | 5秒发布进程命令执行一次 |
| 成功发送的非零0x10 | 第二次请求记录到11帧 | 11帧，序号814–824 |
| 同次遥测匹配的非零命令ACK | 3886–3895 | 814–823 |
| 目标窗口四轮反馈速度 | 全零 | 全零 |
| 目标窗口raw计数 | 固定[560,-249,315,47] | 固定[560,-249,315,50] |
| 遥测软件duty绝对最大值 | 四轮均279 ticks | 四轮均520 ticks |
| fault诊断 | mask4，目标[50,50,50,50]，delta全零 | mask4，目标[100,100,100,100]，delta全零 |
| fault软件duty快照 | [-281,279,279,279] | [-524,520,520,520] |

两次fault来源均为`closed_loop_stall_or_direction`。源码在`abs(actual)<3 && abs(duty)>64`时累计失速时间，超过450ms锁故障；反向条件共用同一fault标志。目标窗口零反馈更支持失速路径，但快照不含触发轮号和累计计时，不能仅凭fault名称判断方向错误。

100mm/s窗口电压遥测11.497–11.523V，EN及power位有效；Guard的hold_reason在运动请求期间为空，结束后为`command missing/stale`。目标已通过ROS链进入MCU，不是本次被Mux保持或Guard拒绝。driver识别fault后发送0x10零并退出，没有PWM回退。记录到其首个后续零帧825；用户命令最后的hold被接受，Mux generation=3、selected_source=zero，最终Twist持续为零。

50mm/s第一次raw请求未观察到下游非零，其具体原因仍未知；不能把第二次的MCU接受证据套用到第一次。50mm/s bag在第二次请求过程中结束，未覆盖fault与最后hold；这些只能由同次driver日志及后续状态补充。试验前出现的编码器变化不得解释为目标响应。

100mm/s bag覆盖启动、故障、用户hold和随后零输出。50mm/s用户报告未动；100mm/s本次答复提供了终端输出，尚无独立四轮实际转动观察。

## 已缩小的问题范围

本次请求已进入MCU轮速闭环，但没有观察到相应编码器反馈，随后保护停机。尚不能区分实际PWM/电桥未输出、起转响应不足或反馈链问题。

`0x90 duty[]`和fault中的`applied_pwm_ticks_abcd`都是软件参考。fault还可能发生在当前轮写CCR之前；它们不是实际CCR读回、电桥波形、电流或转矩。EN/ADC有效也不能证明各路电桥供电输出正常。主机FATAL时间包含driver关闭流程，不能用主机日志间隔推算MCU故障延迟。

已有`0x15`及`Chassis.read_motor_registers()`能读取部分输出寄存器，但当前driver没有ROS调用入口，本次未采集故障前CCR。fault停止后再读全零不能重建此前输出。下一步应补足实际输出端证据，不继续靠提高速度或修改PI猜测；不需要新管理框架，也不能另开串口争抢driver。

## Motor OFF的实际含义

固件HELLO始终先停止输出；仅当Motor输入为OFF时执行`clear_fault()`（`main.c`的HELLO分支）。单发零速、重新配置或只拨OFF都不能清除既有fault；需要OFF状态下重新HELLO。**这是故障恢复条件，不是无fault时每次正常启动的硬要求。** 物理断12V属于烧写/接线及前次零速核验条件，不应套用到正常运行。

## 证据位置

本次原始证据保存在新工程`.test-results/wheel-speed-config-arm64-20261024/`内，不在队友树，也没有扫描历史日志。

| 文件 | SHA256 |
|---|---|
| `wheel-speed-b-20261024-01/response.bag` | `2beba4e1ba2e8ad8327e914d54ed8562483e6c1f6fcea4d7e197f80b55e2f377` |
| `wheel-speed-b-20261024-02/response.bag` | `29b744604bd55ae177d2d244ccb5ef30fd8d72260c865f665e5d676a07a9de09` |
| `wheel-speed-b-20261024-02/guard-reasons.bag` | `08bcb003330e789fcdbbeb5c080cdf41248a1c10ec103db4c3dc874fcd64f16c` |

第二次目录含`current-driver-log.log`、`motion-analysis.json`及`response-summary.json`。两个bag以只读方式在`--network none`、无设备Noetic容器内分析；这是实测记录分析，不是软件模型成功结果。固件三处dt修复、烧写回读及A阶段已通过的结论不因此扩大为带动力闭环通过。

## 后续寄存器观察改动（4cf06c0）

上述两次失败发生时driver未请求寄存器。现已给同一driver补上可选`observe_motor_registers:=true`：正常非零命令发送后最多5Hz发送已有0x15，不等待回复；poll收到0x92后通过现有diagnostics发布`motor_registers`事件。默认关闭；没有新节点、service、运动门控或固件修改。底层串口写仍有原0.2秒超时，MCU每次69字节同步回复约6ms；不能称严格无阻塞或零时序影响。

新增8项driver测试，50/50通过；独立审查通过。ARM64隔离构建、完整112项JUnit及安装通过，69个实际源文件哈希一致。WSL首次完整回归为112项、1失败：既有`GoalProcessTest.test_cli_success_timeout_interrupt_and_explicit_cancel`在SIGINT触发的actionlib/Condition等待中出现`cannot release un-acquired lock`。同源码单项复查通过；首次失败XML/日志均保留，未修改导航客户端，不能宣布其Ctrl-C问题已修复。进入C导航验收前需要处理这个独立问题。

构建输入归档SHA256为`d13dbf51500866f0902aeac0a74a680f829575ec39f9069461351b0269efa307`；安装driver为`6e468f5b7699600d9fef59680dc7269765d940c055a8b44ed01a3c8a3d644f78`，link为`e3a8d02a75bd0e0e8120afc791dd6d4cf375d96d1834840e8bdf8997a9ff7835`。证据位于`.test-results/output-register-observation-20261024/`，包括RED/GREEN、首次失败及复查、ARM64原始JUnit和9个运行源/安装文件的替换清单。

只更新了用户刚才使用的独立`wheel-speed-config-arm64-20261024/source`测试目录，保存了替换前9文件；未修改主项目默认控制模式、队友树或固件，也未自动启动driver。用户随后执行了同一100mm/s比较，结果如下。0x92没有MCU时间、nonce或请求序号回显，且未覆盖TIM9/10/11控制寄存器及GPIOE AFR[0]，不能把少量寄存器快照扩大为全部引脚、电流或轮子运动验收。

## 实际寄存器对照结果：输出写入已确认，轮子仍不动

实例`dded8b85b0924e8881b06f7f5799917f`，用户明确观察到四轮完全不动。10帧非零0x10期间，raw计数始终为`[560,-250,315,50]`，四轮反馈全零，ADC电压11.293–11.311V。故障前取得3份实际0x92快照：

| MCU帧序号 | A TIM10_CH1 | B TIM9_CH1 | C TIM1_CH2 | D TIM1_CH4 |
|---|---:|---:|---:|---:|
| 40698 | 40 | 40 | 40 | 40 |
| 40705 | 235 | 235 | 235 | 235 |
| 40711 | 421 | 421 | 421 | 421 |

每份快照中，配对的TIM11_CH1、TIM9_CH2、TIM1_CH1、TIM1_CH3均为0，符合当前符号经过`pwm()`后的输出映射。TIM1 CR1=`0x81`（CEN/ARPE开启），CCER=`0x1111`（四通道使能，极性位0），BDTR=`0x8000`（MOE开启）；PB AFR高=`0x33`（PB8/9 AF3），PE AFR高=`0x01101010`（PE9/11/13/14 AF1），PD3=1。

现在可排除“只有软件duty增长、CCR始终未写入”及快照时TIM1 CEN/CCxE/MOE未开。仍没有CNT、CCMR、GPIO MODER、RCC时钟或实际引脚波形、电桥输出、电流证据。与能运动的0x11相比，0x10采用drive/00而0x11采用drive/11；两种写法在参考真值表下都存在驱动区间，不能将衰减模式区别直接定为根因，也不能盲换函数而忽略A/B/D符号会翻转。

本次仍为mask4、delta全零、目标`[100,100,100,100]`，软件fault快照duty=`[-510,506,506,506]`、dt=10。这些duty并非上述CCR的同周期读回。用户hold接受后Mux generation=4、selected_source=zero；driver故障退出，未继续发其他试转。软件诊断补丁已采到所需寄存器信息，带动力响应B仍未通过。

原始记录`.test-results/output-register-observation-20261024/field-registers-01/response.bag`为4,879,176字节，SHA256=`bef1d5ecef07fc2c52c41ecfbd200b61a7dbd674ab9d2a9527569dc282850264`，与车端封口原件一致；同目录保留实例日志、离线分析脚本及`register-summary.json`。现场随后未找到万用表，未取得板端电压测量；用户改走下述同链路PWM对照，助手继续被动记录。

## B4：同链路PWM对照能动，轮速闭环仍未通过

用户在同一Guard raw → Guard → Mux →唯一driver链路、沿用现场接线，切回既有`pwm_reference`并启用寄存器观察，没有新增控制代码或修改固件。输入`Twist.linear.x=0.125`、`angular.z=0`的参考请求，0x11实际发送峰值为ABCD `[142,-142,142,-142]‰`。这是开环PWM映射，不能将输入参考当成实测0.125m/s。用户明确报告车辆向前运动；本项未独立核验架空/负载条件，也未验证速度跟踪。

实例`36093ef02755477195912e19b895d7d0`中，首个非零0x11前约0.5秒的13份raw样本均为`[560,-250,315,50]`。随后出现以下变化，累计量均相对此基线：

| 相对首个非零0x11发送事件的bag接收时间 | ABCD累计raw变化 |
|---|---|
| 约0.161秒，首次变化 | `[-5,0,-2,1]` |
| 约0.381秒，四轮均有明显变化 | `[-118,108,-130,318]` |
| 约0.489秒 | `[-505,518,-579,725]` |

这些是记录事件的主机接收时间差，包含传输、采样和队列延迟，不能当成精确物理起转延迟。对照在前0.5秒内已出现四轮编码器响应。原始编码器变化和用户前进观察支持本次PWM路径实际驱动成功，但不标定CPR、实际m/s、电流或转矩。

共收到21份实际0x92。稳定输出快照中，TIM10_CH1、TIM9_CH1、TIM1_CH2、TIM1_CH4为1600；配对TIM11_CH1、TIM9_CH2、TIM1_CH1、TIM1_CH3为1373，符合0x11的drive/11表达。它们仍不是引脚波形或电桥电压测量，0x92覆盖缺口与前节相同。本次没有driver fault事件。用户随后hold，Mux generation=5、manual_hold=true、selected_source=zero；最终遥测四轮速度、delta及软件duty均零，用户已停止该driver。

与B3的0x10对照，现在已确认同一现场链路能通过既有0x11路径产生用户观察到的前进和编码器变化。两模式的桥表达、输出幅值/时间轨迹及保护分支不同：0x11为开环，不经过0x10的PI失速保护。现有证据支持继续验证闭环末端桥表达，尚不能将drive/00与drive/11区别认定为唯一根因，更不能据此宣布轮速闭环通过。

启动日志中的`Could not store driver provenance parameter: cannot marshal None unless allow_none is enabled`是XML-RPC元数据参数写入警告；同一实例随后正常发送0x11并取得响应，该警告没有阻断本次已观察到的PWM运动。参数服务器的`~provenance`可能保留旧值，本次模式和实例身份以同次startup诊断、日志及实际串口帧为依据。

证据目录为`.test-results/output-register-observation-20261024/field-pwm-01/`，含`pwm-summary.json`、本次driver日志与原始`response.bag`。bag为10,993,512字节，SHA256=`9f183581c4f3a3c3a8ae88c3aa0574df3925755e28fd696786a19f8fea4d80f3`。

后续桥表达候选仅做离线准备与实际C验证；原`firmware/c30d_vet6/main.c`仍保持已烧入版本，未烧写候选。候选属于原三处dt修复之外的新改动，需要单独的新镜像构建、审查及验证；具体构建标识另记。离线通过不替代带动力闭环验收。

## B5：新固件100mm/s地面前进响应

用户单独批准候选后，已完成新鲜双读、仅扇区0烧写、完整回读及CONFIG/四零检查，见[实际烧写记录](../firmware/C30D_BRIDGE_FLASH_2026-10-24.md)。实际新镜像5944B，SHA=`fb0fee03a4e652593c75be6e2f9f3c495bf2685b0c3265552554e2ead04f3e2d`；完整回读=`6cbf7993c9b38359d99d49eae2791cf5933610605e36e962a24c83e65631691b`。B4之后的“尚未烧写”描述属于候选准备时点。

用户重新启动定位、A和D，driver实例`25f8e8bc66a74b619a8e537d653538fe`，同一wheel_speed配置及独立符号，CONFIG请求与ACK nonce均为3431832335。静止采样Mux为startup hold、最终Twist零；本次仍经raw→Guard→Mux→唯一driver。助手仅订阅、记录和分析，release、原始Twist及hold均由用户执行。

用户执行与刷前相同的`linear.x=0.10`、5秒发布进程命令，并明确确认“四轮都朝前、hold后自行停住、无干预”。记录到85个非零0x10，全部有对应遥测ACK；目标随Guard加速度约束从10递增到四轮100mm/s，非零发送持续约4.53秒，最大发送间隔56.3ms。没有0x11或标定命令，没有fault事件/故障位；D仍保持运行。

| 轮 | 起步reported峰值mm/s | 最后约1秒均值 | 最后约1秒范围 |
|---|---:|---:|---:|
| A | 156 | 100.40 | 95–105 |
| B | 155 | 100.68 | 98–105 |
| C | 156 | 100.96 | 97–105 |
| D | 153 | 100.72 | 96–106 |

这里是固件基于暂定60000 counts/rev计算的轮速，不是外部标定值。存在明显起步超调；首次四轮全100目标后约1.085秒起，四轮采样均保持90–110直到零命令，末段未见持续大幅振荡。本次不改PI/前馈。全程raw累计为`[-106996,+107297,-107711,+107299]`，结合当前编码器符号得到四轮正向，且与用户观察一致。用户确认本次在地面，但未量测该次实际位移，不能据轮端计数宣称实际地速或距离已标定。

114个活动遥测的软件duty绝对峰为`[282,281,280,281]` ticks；22份0x92中21份符合高/高表达，1份全零；实际差分峰为`[259,256,250,253]`，均未达到960。采样窗口不同，所以两组峰值不要求一致。电压遥测约11.745–11.869V。这里确认了新固件下实际输出、编码器响应和反馈收敛的对应关系，仍不代替电流或转矩测量。

停机顺序由原始记录确定：raw发布结束，Guard进入`command missing/stale`并输出零，MCU接收四零，之后首次hold确认。首个0x10零的接收记录比hold状态确认早约1.33秒，因此本次主要验证raw指令超时停机及随后人工保持，不能描述为运动中hold首先打断。约52ms后遥测确认零ACK与软件duty零；从该ACK帧起按MCU tick计约0.24秒，四轮reported速度持续零。其后A/B/C各仅再变1个raw计数，D不变，不能将最晚1计数延续至3.52秒解释成持续滑行或精确物理停车时长。停止之后没有新的0x92，不声称取得了该时段的实际CCR快照。

最终Mux为manual_takeover hold、generation=1、selected_source=zero，最终Twist持续零；约75秒尾段软件duty和reported速度持续零。root分析中严格要求delta也全零的指标为false，原因是上述微小计数尾段，原结果保留；没有把它改成假通过。driver ready=true表示使能、供电、通信条件有效，并不表示正在运动。

本次原始bag：`.test-results/c30d-bridge-candidate-20261024/field-response-01/response.bag`，25,958,802B，SHA=`2811e44b74108101561fbda2e664a026895f596bb6c8552d00a962b4bbbbb3ca`，与车端封口文件一致。同目录含本次driver日志、preflight、用户观察、两份独立离线分析脚本和JSON。分析均在无设备、network none容器中读取真实bag；一项容器分析首次自动审批超时未执行，按返回许可收窄挂载重试一次后完成，未将超时解释为安全拒绝或试验失败。

结论仅为本次100mm/s地面前进起转、reported轮速收敛、raw超时停机及人工保持得到实物支持。前次0x10不转已出现明确改善；低轮速与每轮反向、地面转向负载、CPR和实际mm/s标定、导航到点及障碍仍未验收。随后执行了B6纯转，失败情况另记；不能继续沿用本节试验后“D正常、无需清故障”的状态。

## B6：地面0.25rad/s纯转失败与试验条件更正

用户按同一Guard/Mux链执行`linear.x=0, angular.z=0.25`，随后hold接受、Mux generation=2。D报告mask4并退出，用户观察车辆没有移动。用户随后明确答复B5和B6“**两次都在地上**”；两次均不得再归类为架空试验。原始bag不变，新增`surface-correction.json`记录条件更正；此前错误物理条件的派生摘要保留副本，定量数值不因条件更正而改写。

按当前track_width=0.15m，纯转参考对应轮速幅值`0.25*0.15/2*1000=18.75mm/s`，取整为ABCD `[19,-19,19,-19]`。本次有11个非零0x10，依次约`[4,-4,4,-4]`、`[8,-8,8,-8]`、`[16,-16,16,-16]`、`[19,-19,19,-19]`；软件没有请求100mm/s，也没有把正向/反向低速转换成额外起动输出。

取得3份0x92：1份全零，后两份有效差分约`[85,85,85,84]`及`[100,101,101,100]` ticks。A/C保持B5的方向，B/D的实际CCR对翻转符合候选写法；仍无证据表明B/D已经产生可用地面反向驱动。软件reported范围仅A=0..2、B=-1..0、C=0..2、D=-3..0mm/s，raw有微小变化，不能写成编码器完全零。

实际fault快照：MCU ms=1658413、dt=10、delta=`[-2,0,0,0]`、target=`[19,-19,19,-19]`、软件duty=`[-110,-110,110,-110]`，EN及power均true。电压11.523–11.532V；首条非零发送后接收记录约0.603秒出现fault4。输出远未到960限幅；无有效起转、低反馈与该快照支持失速路径，不能把共用的`PID_STALL_OR_DIRECTION`名称直接解释成方向错误。故障前raw累计`[-62,-33,-34,-75]`，配置后符号与请求一致，属于微量响应，仍不足验证地面持续反向驱动。

代码核算只用于解释数值：在恒定19、零反馈、积分从零、dt=10ms条件下，KP=1、KI=4、FF=3使首周期effort约76.76 ticks；积分每10ms增约0.76，约450ms时输出幅值110。因实际速度<3且duty>64，严格`stalled>450`会在下一周期触发。实际还有目标爬升和微量反馈，这不是对实车时刻的精确模型；它与快照110吻合，也说明不能等价为“已经施加600‰还转不动”。软件duty不等于实际电流或转矩。

地面四轮滑移转向有额外轮胎横向摩擦和转向负载，因此B5的较高轮速前进成功不能保证19mm/s地面纯转成功。[CMU滑移转向研究](https://publications.ri.cmu.edu/experimental-comparison-of-skid-steering-vs-explicit-steering-for-a-wheeled-mobile-robot)支持随转弯半径减小所需功率/转矩上升这一一般机制，不提供本车的起转阈值。现有证据支持“低速可用输出与地面转向负载不匹配”的假设，尚未证明它是唯一因素；B5微小计数的首次变化也不能当统一PWM门槛。

原始bag位于`.test-results/c30d-bridge-candidate-20261024/field-turn-01/response.bag`，32,988,736B，SHA=`386091cd17235cb94e0d0c621d353889d7e6d9dc80c26843facff1240b52e322`；车端封口归档与本地5项文件哈希均已核对，D日志、失败输出及`analyze_turn.py/json/log`保留。故障后发送11条零命令，未收到零ACK；不能把发送成功当作执行确认。最终Guard/nav和Mux输出持续零，raw停止发布且没有显式零样本。D已退出，Mux处于manual hold。本轮没有追加固件修改、调PI、提高角速或放宽失速保护。

最小对照是同一纯转指令、同一固件下真正四轮离地，改变地面接触这一变量。若离地能起转，则进一步处理地面转向负载下的低速能力；若离地仍起不来，则先处理底盘自身的低速起转/方向问题。该对照只用于定位，地面纯转仍是最终必须通过的功能。

## B7：离地对照仍失败，结束现场测试

上述对照已由用户执行；用户后来明确补充“本轮是离地”。新driver实例`cda3eac99b5e4f10a56c235e951ec408`只有一次fault，MCU ms=2487207、dt=10、目标`[19,-19,19,-19]`、软件duty=`[-113,-113,113,-112]`、delta=`[-1,0,0,0]`、EN及power有效。用户观察仍不动。地面摩擦不能作为本轮失败的唯一解释；低速持续反向能力和整个B仍未通过，不直接由此认定PI是唯一原因。

raw只有一组持续0.25rad/s发布；Guard/nav和Mux中间出现短暂回零，driver收到的非零0x10被显式零分成6段。间歇时Guard记录`swept footprint occupied/unknown/outside grid`，对应driver停止原因为`zero_command`；不能把从第一条非零至fault的约3.5秒当连续失速时间。最后持续段的低速执行失败仍须解释；Guard的独立阻停也要单列审查，不能绕过它。

B终端的粘贴拼接产生`holdrosrun`参数错误；最终独立hold接受，Mux generation=3、selected_source=zero。日志中另一条fault属于上一轮B6，不是本轮第二次故障。最终D退出，Guard和Mux输出零，助手停止自己的记录器并封存日志，没有追加试转。

原bag为38,775,109B，SHA=`ecdaa1e0310d7cb98738ff8a38dcd4742a80e0aae1553dfcd90e8e9ff8f8de61`，位于`.test-results/c30d-bridge-candidate-20261024/field-turn-final-01/`。同目录含原始D日志、用户命令/观察、场地确认、离线分析及此前场地未确认版本的副本。物理条件确认仅更正解释，不改原始bag或数值。
