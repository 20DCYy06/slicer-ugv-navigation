# 第三方来源与许可说明

本仓库保留来自上游、队友Slicer工程及我方适配的代码。目录重排不改变原作者与许可，不为全仓新增统一许可证。请结合每个组件的许可原文、版权头和来源记录使用。

| 组件 | 保留的来源/许可入口 | 需要区分的事实 |
|---|---|---|
| FAST-LIO | [LICENSE](reference/Slicer_Workspace/catkin_ws/src/fast_lio/LICENSE)、[package.xml](reference/Slicer_Workspace/catkin_ws/src/fast_lio/package.xml) | 包声明BSD，随附LICENSE为GPLv2；此差异尚待向来源核清 |
| Livox ROS driver2 | [LICENSE.txt](reference/Slicer_Workspace/catkin_ws/src/livox_ros_driver2/LICENSE.txt) | 保留原文本及源码版权头；SDK2不是本仓库完整交付物 |
| NMEA解析 | [来源版本](src/cup_gnss/src/cup_gnss/NMEA_UPSTREAM.json)、[_nmea_0_6_1_parser.py](src/cup_gnss/src/cup_gnss/_nmea_0_6_1_parser.py)、[checksum](src/cup_gnss/src/cup_gnss/_nmea_0_6_1_checksum.py) | 原BSD版权/许可在文件头中 |
| NTRIP | [ntrip_client许可](src/cup_gnss/LICENSES/ntrip_client-1.4.1-LICENSE.txt) | 复用成熟后端及项目内适配，保留归属 |
| Slicer控制适配 | [SOURCES.json](src/ugv_slicer_control/SOURCES.json)、[迁移说明](docs/history/MIGRATION.md) | 既有Guard/Mux/协议基础和后续适配不是同一次原始创作 |
| 固件 | [最新记录源码说明](reference/Slicer_Workspace/firmware/c30d_body_closed_loop/README.md) | 保留厂家头文件、队友基础与后续补丁来源；不代表最新实车验收 |

标准ROS系统包通过开发环境依赖提供，不在本仓库重新声明其版权。package.xml中的license字段是原包元数据，不能单独用于推断整仓所有文件的许可。

队友仓库：[potato-ugv-online-navigation-radio](https://github.com/20DCYy06/potato-ugv-online-navigation-radio)。当前记录与版本关系见[整理来源](reference/provenance/README.md)。
