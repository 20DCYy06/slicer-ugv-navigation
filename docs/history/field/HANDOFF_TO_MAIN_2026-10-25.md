# Slicer UGV：2026-10-25 实车试验与IMU闭环交接

**阶段结论：真实输入规划、直线odom自主到点已得到实车证据，按当前15cm位置容差工作。停止重复同类直线录像；本阶段剩余基础障碍响应。右偏、负19mm/s低速间歇以及更紧到点精度仍待分别处理。**

**版本更正：队友已采用下位机IMU车体闭环方案，队友配置默认 `body_speed`；用户确认新固件在本轮六次抬轮录像之前已烧入。我们本轮的采集却全部使用我方 `wheel_speed/0x10`，没有启用或验收新增 `0x16` 外环。不要把“已刷IMU固件”“队友采用该模式”和“本轮验证了IMU闭环”混为一谈。**

## 本轮实验与结论

| 阶段 | 输入及实测 | 结论与限制 |
|---|---|---|
| 六次抬轮：50正、19正、19负，各两次 | 画面左=A右后，右=C右前；六次开始前已排除电池擦A。50后段A/C反馈约50mm/s、duty均约80ticks。负19两次在持续非零目标下A近停、C仍有计数。 | 旧“A在50需要约1.8倍输出”未复现，支持额外机械摩擦的解释；负19并未解决。Guard把六次分别切成3/6/5/3/6/3段，不能把所有停顿都归给电机或合成连续稳态调参。 |
| 地面50mm/s直行两次 | 人工各约39cm；轮程39.17/38.65cm，LIO39.20/38.82cm；2秒后各轮平均反馈49.2–49.9mm/s。 | 连续行驶、视频末端基本停稳，未见Guard中途拦停或fault；起步各轮峰71–76mm/s，超调仍在。停车余程未单独实测。 |
| odom导航：0.5m、0.5m、1m | 人工39/40/90cm，三个终端结果均SUCCEEDED，用户观察均偏右。 | 首轮bag确认：剩14.53cm时导航开始发零，成功时剩10.85cm，符合生效的15cm容差。不能据此改轮径或里程计比例。另两次只有终端、视频和实测，未在bag内。 |

“19mm直行/逆行”是保留的视频文件名，实际为 `v=0,w=±0.25rad/s`，当前主机按轮距0.15m换算到四轮约±19mm/s，并非车体19mm/s直行。duty单位是软件timer ticks（周期1600），不是‰、电流或CCR读回。

视频的作用已经完成：核对轮子真实转停，结合日志识别Guard中断，观察地面连续运动及停车尾段，排除明显外力/冲过再退。手持画面不能给出毫米级偏移或精确停车余程。三次右偏保留为用户现场观察，不因影像难以定量而否定。

## 找到的队友IMU程序

车端根目录：`/home/potato/Slicer_Workspace/firmware/c30d_body_closed_loop/`。

| 文件 | 职责 |
|---|---|
| `main.c` | 旧0x10轮速入口、新0x16车体入口、来源选择、轮速内环和遥测。 |
| `BALANCE/balance.c`、`balance.h` | 车体外环：编码器均速反馈vx，IMU实际偏航角速度反馈wz；输出四轮目标，交给轮速内环。 |
| `HARDWARE/ICM20948/ICM20948.c`、`.h` | 板载ICM20948采样、单位换算、静止零偏估计、角速度滤波及新鲜度。 |
| `HARDWARE/usartx.c`、`.h` | 车体协议、消息类型和扩展遥测。 |
| `tools/body_link.py`、`monitor.py` | BodyChassis主机协议适配、扩展/IMU就绪检查和观察。 |
| `config.yaml`、`build.py`、`build/generated.h` | 实际配置、构建入口和生成参数；附现存 `build/c30d_body.bin` 与构建清单。 |

已只读取回源码/配置/测试及现存BIN。BIN为9892字节，SHA256为 `b8a5b305c4015ced10bd961762da240834870273bdcfca391c550c5931bd12e2`；它是车端现存构建产物，本轮没有刷写或MCU回读，不能把此哈希称为本轮硬件身份验证。

新增接口为 `0x16 + <Ihh>`：nonce、vx毫米/秒、wz毫弧度/秒；`0x17`查询扩展，`0x93`输出车体/IMU反馈。旧 `0x10 + <4h>` 仍是ABCD毫米/秒。**诊断JSON的十进制 `kind=16` 是0x10，0x16的十进制是22。**

新外环前馈使用 `(轮距+轴距)/2`，配置为0.15+0.17m；同样0.25rad/s名义差速约±40mm/s，再叠加反馈补偿。它可能改变转向能力，但不是证明19mm/s轮速内环已平顺。

## 补充取回的其他修改

与上一份 `slicer_odom_takeaway_20261025` 的队友控制包快照相比，确认以下四个文件变化：

- `Slicer_wasd_control/config/teleop_params.yaml`：默认加入 `body_speed`，最大轮速声明600→800mm/s。
- `scripts/c30d_driver_node.py`：新增BodyChassis、IMU就绪检查和0x16发送分支；保留wheel_speed分支。
- `scripts/c30d_link.py`：允许的最大轮速配置范围扩到800mm/s。
- `tests/test_control.py`：相应更新轮速限幅期望。

新固件也不仅是旧合并 `956eca3` 的三个dt替换：已增加外环/IMU/协议/串口缓冲，并改变轮速控制的部分实现。旧153项C回归的结论只属于旧受测源码，不可套给此新版本。补充包保存完整相关源码、逐文件SHA、上述差异和旧main→新main对比；本次未运行新固件回归。

## 我方当前状态与主窗口接续点

- 我方工程HEAD仍为 `123adfd`；本窗口新增采集材料、分析与交接文档，没有继续改固件或导航/控制逻辑。旧项目与队友源码保持只读。
- 我方driver仅支持 `pwm_reference/wheel_speed`，本轮用后者。若要使用0x16，需要明确接入BodyChassis/IMU状态与反馈，不能仅把配置名改成body_speed。
- 队友teleop配置切到body_speed曾使我方Mux报 `Unknown control_mode: body_speed`。现场已用A启动参数 `control_config:=/workspace/install/share/ugv_bringup/config/slicer_driver_wheel_speed_reference.yaml` 选定独立配置；launch默认值尚未修改。
- 原先大幅越过目标未在本轮复现；少走约10cm先对应当前容差。若目标是5cm级到点，最小候选是单独收紧现有xy容差后一次对照，尚未修改，也未承诺5cm精度。
- 直线复验停止；下一项是基础障碍减速/阻停与取消/hold。右偏、新0x16转向及负19另列，不扩RTK、自动恢复或新导航框架。

## 数据覆盖与材料入口

三组本轮采集均已完整取回并校验：

- `evidence/wheel-follow-20261025-042659`：六次抬轮；六视频在 `evidence/wheel_follow/`。
- `evidence/ground50-20261025-051045`：两次地面直线；两视频在 `evidence/ground50/`。
- `evidence/nav05-20261025-053203`：120秒，首个目标在约105秒后才发出，只覆盖首轮运动/result，未捕获goal消息；目标取自终端三位小数。三视频在 `evidence/nav05/`。后两次不要求补包重跑。

总补充材料目录：`review_packages/slicer_field_handoff_20261025/`。包含本报告、当前队友源码/构建产物、变化清单和diff、三组原始bag/manifest、分析数值/曲线/关键帧、我方HEAD源码归档及旧固件基线。11个原视频现保留在 `evidence/` 下的同名分类目录，补充包附原路径、SHA和选取帧，避免重复打包大视频；包中旧 `tests/` 媒体路径现对应 `evidence/`。详细文件说明和新旧协议注意项见包内 `SOURCE_NOTES.md`。
