# 版本与证据索引

[返回首页](../README.md)

状态截至2026-10-30。前轮已核对源码、启动配置、两包一致性与归档测试结果。本次将我方src/tools提升到仓库根，并重新检查复制身份和文档路径；没有重新执行实车试验、刷写、串口操作或431项测试。

## 输入身份与本轮核验

| 输入 | SHA256 |
|---|---|
| slicer_current_sources_20261030.zip | `1dff9dfcaf9c4ea8b2fa1597b3566ca73acfc14b2821932e9fe17fe60412840a` |
| slicer_gnss_arm64_20261030.zip | `68d9a79b24b111857ed497064af97927d0196a43ded6db5e5a81fbfdcfa046a2` |

本轮逐项校验原源码包SHA256SUMS中的625个条目，全部匹配。源码包总626文件，校验清单自身不列入其自身哈希。ARM交付包内source.tar.gz的234个普通源码文件与当前候选逐字节一致。

原包结果：[独立包核验](../evidence/independent_package_checks.json)。本次目录整理结果见 [packaging_validation.json](../evidence/packaging_validation.json)，文件映射见 [repository_files.json](../reference/provenance/repository_files.json)。原始文件路径与来源见 [FILE_MANIFEST.csv](../reference/provenance/original_bundle_FILE_MANIFEST.csv)。

## 开发、部署与固件

| 身份 | 证据 | 可下的结论 |
|---|---|---|
| 候选27488d3… | [候选版本](../reference/provenance/our_candidate_version.json)、ARM归档源码一致 | 当前四包软件基线 |
| Jetson既有部署快照 | [差异清单](../reference/provenance/jetson_vs_candidate.json) | 区分相同核心文件与新增候选；不证明采集时节点运行 |
| 3849abee…，10208字节 | 前轮实测归档身份核对与原始bag独立解码 | ±0.08交替方向功能通过，低速准确跟随待验 |
| 0852f4b4…，10488字节 | [最新安装记录](../reference/Slicer_Workspace/firmware/c30d_body_closed_loop/last_verified_install.json)、[差异](../reference/provenance/firmware_3849_to_jetson0852.patch) | 有安装/现有BIN身份记录；本轮无板卡读回、独立重建或新运动结果 |

3849与0852的balance.c相同；共同文件有七项变化，包括0x18、动态输出上限与编译cap 600→800‰。版本变化不能只用“最新”二字概括。

## ARM软件验证

包内记录为tools 153项 + Catkin汇总278项 = **431项**，errors/failures/skips为0。本轮交叉查看归档日志和28份JUnit XML，计数一致。Catkin汇总含rostest包装/rosunit口径，不能写成431个独立实车场景。

- [ARM结果摘要](../evidence/archive_20261030/ARM64_RESULT.json)
- [隔离条件](../evidence/archive_20261030/test-container-isolation.json)
- [ARM日志](../evidence/arm64/noetic-arm64.log)
- [JUnit目录](../evidence/arm64/junit)
- [PTY集成结果](../evidence/archive_20261030/pty_result.json)

PTY测试覆盖q4/q5/q0、过期、CRC、断续恢复及单串口所有者，属于合成GNSS/NTRIP链路。结果明确 `hardware_acceptance=false`；未打开真实接收机或连接真实NTRIP。它不能证明真实RTK精度、在线建图、深度绕障或空地端到端任务。

[选定源码检查](../evidence/archive_20261030/selected_sources_final.txt)只到包发现/launch解析，节点均未启动，也没有重新构建传感器二进制。它与431软件验证的范围不同。

## 3849现场复核的关键量

[新旧同量程响应图](../evidence/field_3849/alternate_old_new_mcu.png)来自前轮原始bag独立复算。本轮只是引用已经完成的审查，不重新冒充采集或读板。

| 指标 | 结果与限制 |
|---|---|
| 新版六段反馈/请求角积分比 | 39.0%、44.0%、30.2%、42.4%、35.4%、34.6% |
| 三个反向段持续起转延迟 | 约1.40、1.20、1.40s；统计口径为同向角速超过0.01rad/s且至少连续0.20s |
| 六腿四轮同时近零采样占比 | 约38%–49%，速度阈值abs(v)≤2mm/s |
| Guard/Mux/ACK | 交替期间持续放行、nav、目标ACK一致，无fault |
| 运行结束 | 首零目标帧PWM全零；四轮首次精确零约0.12s，后续目标/PWM持续零 |
| 独立zero包 | 处于禁能/缺动力状态，不能替代有动力零速验收 |

角积分来自MCU角速度，不是外部量角。上表支持方向恢复与迟滞仍在；不提供地面停车距离、轮vx尺度或导航到点验收。

原交替包SHA256为 `1efa0e366bb990defb4284aa9877bbed8df78e256d8155cf91984e302de03cdb`。本草稿只附比较图与摘要，没有附原始bag；重算运动曲线需另取该证据包。

## 关键源码锚点

| 结论 | 直接入口 |
|---|---|
| LIO投影、首帧局部原点 | [lio_projector.py](../reference/Slicer_Workspace/catkin_ws/src/slicer_bringup/scripts/lio_projector.py#L13) |
| 融合掩码 | [fusion.yaml](../reference/Slicer_Workspace/.generated/fusion.yaml#L9) |
| 轮里程计实际协方差 | [driver](../src/ugv_slicer_control/scripts/c30d_driver_node.py#L257) |
| 在线默认和TF分工 | [slicer_online.launch](../src/ugv_bringup/launch/slicer_online.launch#L4) |
| 纯转换向积分修复 | [balance.c](../reference/Slicer_Workspace/firmware/c30d_body_closed_loop/BALANCE/balance.c#L26) |
| 新输出上限接口 | [main.c](../reference/Slicer_Workspace/firmware/c30d_body_closed_loop/main.c#L167) |
| 航点仅落盘 | [uav_waypoint_receiver.py](../reference/Slicer_Workspace/catkin_ws/src/slicer_navigation/scripts/uav_waypoint_receiver.py#L27) |
| GNSS质量要求 | [quality.py](../src/cup_gnss/src/cup_gnss/quality.py#L17) |
| 未知场地配置 | [gnss_site.yaml](../src/ugv_bringup/config/gnss_site.yaml#L3) |

源码能说明行为和配置意图，现场包能说明特定版本/条件下的观测；两者共同构成结论，不能互相替代。
