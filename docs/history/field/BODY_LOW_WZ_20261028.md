# 0x16小角速度定点排查（仅离线）

已找回真正的车体闭环源快照，未连接运动硬件、部署或刷写；EKF、DWA、Guard、MCU生产代码与感知参数均未改。

## 源码与版本

源在 c30d_body_closed_loop（原归档未随仓库收录：`../../review_packages/slicer_field_handoff_20261025/teammate_snapshot/firmware/c30d_body_closed_loop/main.c`），不是 `c30d_vet6`。前者包含0x16、车体外环、四轮PI和0x93；10月28日材料包误收的是后者，不能用于解释本次车体控制。

23份文件与10月25日清单逐项相符；配置、生成头、构建清单和9892字节BIN相互一致，BIN SHA256为 `b8a5b305c4015ced10bd961762da240834870273bdcfca391c550c5931bd12e2`。生成头已独立重生成并逐字节比较。

**这证明找回了有效0x16源码基线，不证明它与10m试验时MCU镜像完全一致。** 现有握手只报告协议能力，没有固件哈希/构建ID；缺少连接这份BIN与该次在装镜像的烧写/回读凭据。未进行ARM重建。详细证据：source-identity.json（原归档未随仓库收录：`../../.test-results/body016-targeted-20261028/source-identity.json`）。

基线参数：轮PI Kp=1、Ki=4、FF=3；车体yaw外环Kp=0.10、Ki=0.20，修正量上限0.25m/s；几何差速前馈系数为 `(0.15+0.17)/2=0.16m`。四轮共同比例饱和至960 timer ticks。PWM极性ABCD为 `[-1,+1,+1,+1]`，编码器为 `[-1,+1,-1,+1]`。0x16主机不另算四轮目标。

## 从10m实测确认的两种现象

- 3534帧0x93全部匹配已接受0x16目标，且与同MCU tick的四轮遥测一一对应。最后20秒Guard持续clear，DWA非空局部路径200条，净位移约1.12cm。
- **持续同向也欠响应：** bag定位129.446～131.552秒，同号约+0.05～0.06rad/s跨53帧、MCU时间2.12秒；目标积分0.12408rad，gyro积分0.00424rad，96.23%的采样时间gyro绝对值≤0.01rad/s。软件duty持续爬升，不能解释成每条命令都被回零。
- **换向另有延迟：** bag定位约138.702秒目标变负后，11帧、MCU首末间隔400ms仍保留旧软件duty方向。它不是车轮实际方向的独立测量，内部轮目标/积分未录到。

0x93 delta是最新单周期原始计数；0x90 delta是约40ms内乘过encoder符号的累计，不能直接比较或再翻一次符号。积分/采样占比用MCU时间，bag接收时间仅作定位；边界至少有一帧不确定性。软件duty不是电桥电压或转矩。

实际时序图（原归档未随仓库收录：`../../.test-results/body016-targeted-20261028/body016-evidence.png`） · 统计与缺失项（原归档未随仓库收录：`../../.test-results/body016-targeted-20261028/body016-evidence-digest.json`） · 逐帧CSV（原归档未随仓库收录：`../../.test-results/body016-targeted-20261028/body016-timeseries.csv`）

源码中车体yaw积分跨请求换向保留，而轮PI在**内部轮目标**变号时才清积分。这是换向迟滞的具体候选机制；不能单独解释上述持续同向欠响应。未据此提高min_w/PWM、修改增益或延长oscillation超时。

## 实际C隔离对照

无网络、无设备容器中直接编译该基线的main、balance、IMU和协议代码，仅替换寄存器/I²C输入。100Hz控制、20Hz命令、固定零gyro和零编码器：+0.08保持2秒后改为−0.08，原版仍给旧方向轮目标63周期（630ms），直到760ms后首次产生新方向轮目标。**这是受控零运动夹具，不是对实车100Hz内部状态的精确重放。**

独立测试副本仅在相邻非零yaw请求反号时清yaw PID：一行诊断候选（原归档未随仓库收录：`../../.test-results/body016-targeted-20261028/c-control/minimal.patch`）。它使该夹具首周期即换向；原始C/协议回归通过，同向200周期、完整零命令及切源路径逐字段不变。夹具故意始终不给编码器运动，候选在3.01秒触发原有stall锁存并归零，保护未放宽。实际输出和CCR均保存在CSV。

**候选不作为完整修复交付或烧写。** 它尚不能解决同向起转不足；行进时`+w→精确0→−w`还会漏掉该规则，小幅请求符号抖动也可能频繁清积分。未扩大成启动补偿或改增益。证据见 C对照结果（原归档未随仓库收录：`../../.test-results/body016-targeted-20261028/c-control/result.json`）。

感知只核对：本轮静态manifest实际参数为min_height=0.04m、blind=0.3m、point_filter_num=3。稍后打包的磁盘scan.yaml为0.02m，不能覆盖运行快照结论；来源差异尚未定位，本轮不改。

## 下次最短复测

以下工具**仅在WSL准备，尚未部署或实车验证**。下次同步后，沿用原定位、导航/Guard/Mux、唯一body_speed driver。先cancel目标并hold，保留move_base给Guard提供实时Costmap；同一场地、负载、电池状态，两组之间不改配置。每组由现场人员确认转向空间、人工接管及急停可用后执行，并拍摄四轮标记和车体。

采集终端：每组用新目录，等 `RECORDING STARTED` 后再操作。

```bash
python3 tools/collect_odom_test.py --navigation --duration 45 --output evidence/body016-steady-01
```

操作终端（现有ROS shell，先取消目标）：

```bash
rosrun ugv_navigation goal.py cancel
rosrun ugv_slicer_control control.py hold
```

第一组：+0.08rad/s同向12秒，前后各2秒零。

```bash
rosrun ugv_slicer_control control.py release && python3 tools/probe_body_turn.py steady; rosrun ugv_slicer_control control.py hold
```

第二组：另开 `evidence/body016-alternate-01` 采集；幅值仍0.08，每2秒反向，共12秒，前后各2秒零。

```bash
rosrun ugv_slicer_control control.py release && python3 tools/probe_body_turn.py alternate; rosrun ugv_slicer_control control.py hold
```

两组均经raw→Guard→Mux→driver→Watchdog，不绕过保护。工具只发raw，不自行release/清锁；driver ready失效、其它raw源实际发指令或Ctrl-C时结束并尝试发零。遇明显单轮卡住、持续大输出不动、异常运动或停车失败，现场立即接管/断动力，结束本段，不反复重试。低角速目标仍可能使MCU提高duty，并不限制转矩。

工具离线验证：9项单元测试通过；tools整组100项、1项原有跳过；无设备私有ROS master的8个用例通过，覆盖两种完整16秒序列、Ctrl-C、ready丢失/停更、静默已注册规划器、真实竞争消息和缺外部订阅。均不算实车通过。记录：ROS结果（原归档未随仓库收录：`../../.test-results/body016-targeted-20261028/probe-validation/ros-final-reviewed/result.json`）。

记录是否从静止起转、起转延迟、换向延迟、四轮真实转停、是否触发fault及是否人工干预；保留原始bag和视频。现有bag缺内部四轮目标、yaw/轮积分和实际电桥测量，不能称已完成精确100Hz控制器重放或实车修复。

回退：本轮生产固件/配置未变，无需刷回。现场停止探针、hold并取消目标即可回到原控制链；离线候选仅供验证，不能直接烧入。
