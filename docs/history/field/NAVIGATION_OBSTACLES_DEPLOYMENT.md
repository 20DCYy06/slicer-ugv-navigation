# 导航/小障碍工具部署记录（2026-10-26）

状态：**Jetson 独立工程已同步，ARM64 构建安装与无设备验证通过。** 本次由用户明确授权部署；没有启动驱动、导航或定位，没有发控制权或运动命令。DWA 仍为默认，TEB 严格兼容失败和感知实车待验状态不因部署改变。

目标目录 `/home/potato/slicer_ugv_ros1`；本次证据目录 `.test-results/nav-obstacles-deploy-20261026/`，已同时保存在车端与 WSL。队友工程和旧项目保持只读。

实际结果：

- 21个明确文件同步：本轮配置、RViz、采集/TEST_ONLY工具及文档；未整体覆盖工作区，未同步日志、bag、历史归档、缓存或旧 dirty driver/control 文件。
- 车端架构 `aarch64`；镜像 `industrial-cup/nav-slicer:20261023-dwa`，ARM64，ID `sha256:bd1daedc41721933507b0017b2acae3a8079b583489498db96707abad069feca`。构建容器仅 loopback，无设备映射。
- `catkin_make --force-cmake install -j2` 成功；16项采集器单元检查通过；7个安装资源逐字节/hash核对通过。
- 默认 DWA、显式 TEB、12m全局/4m局部配置、Guard raw接线、footprint及 `.125m/s/.25rad/s` 限速已通过参数解析。TEB覆盖权重确认 `1.5/.2`；诊断投影和新采集选项可解析。仅解析 launch，不启动其节点。
- 20个原有源码/安装文件哈希不变，涵盖实际driver、Guard、Mux核心文件以及原DWA、地图、footprint、body配置；部署前后ROS节点集合完全一致。
- 车端具备 pointcloud_to_laserscan 1.4.1、RViz 1.14.26、TEB 0.9.1、DWA 1.17.3；未新装系统包。

覆盖前源码保存在本次证据目录的 `source-before.tar.gz`，SHA256为 `2aa395dc65b43b37e7934e5c480e9b93f4c00943bdccf065861e404e00f1b91c`，配套 `source-before.json` 标明原来不存在的新文件。构建/验证见 `arm64-build-install.log`、`arm64-result.json`；初始传输清单为 `payload-files.json`，完成部署后补记文档的最终清单另存 `final-files.json`。

接下来按[现场操作单](NAVIGATION_OBSTACLES_FIELD.md)由用户启动，先做静止感知，不先发运动目标。改回 `local_planner:=dwa` 可退出TEB实验；关闭诊断旁路仍保留原scan。部署不等于实车漏检、二维跟踪或绕障已经解决。
