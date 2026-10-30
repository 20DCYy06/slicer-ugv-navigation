# odom 现场准备：离线验证交付

**这是准备阶段的历史快照。** 后续用户已授权并完成Jetson同步/ARM64安装，也批准应用LIO补丁；当前状态见[车端交付](ODOM_READY_20261025.md)和[现场操作单](ODOM_FIELD_CHECK.md)。下文原版失败与当时边界保留，不作为当前部署状态。

2026-10-25。范围是 WSL 主树 `915d503` 上的采集、分析、独立测试和文档；没有合并固件、修改生产控制逻辑、部署、SSH 接车或打开串口。本次 ROS 测试均在已有 amd64 镜像中，以 `--pull never --network none`、无设备映射运行；软件输入标注 TEST_ONLY，不代表实车结果。

## 交付与用法

- [一页现场操作单](ODOM_FIELD_CHECK.md)：启动前检查 → 离地低速 → 地面短直线 → 导出。地面自主阶段的 LIO 前置问题见下文。
- [实际启动链](ODOM_STARTUP_CHAIN.md)：入口、overlay、Topic、默认值及现场有效配置核对；没有另起 fusion 或导航框架。
- `tools/collect_odom_test.py`：只订阅、读取所选参数并运行标准 `rosbag record`。保存 `motion.bag`、`manifest.json` 和测量模板；Ctrl-C 收尾。包括 wheel/LIO/EKF odom、scan、TF、diagnostics、已有 MCU 反馈、速度链与 Action 状态；不录点云/图像/costmap，不订阅 manual_intent。
- `tools/analyze_odom_test.py`：与同目录已有 `analyze_motion_bag.py` 一起使用；输出时间戳、频率、速度、位移增量、公共时间窗内的 LIO/EKF 差异与 MCU 摘要。人工测量单列，未知保留 `null`，不据零速反馈断言物理停稳。

在已启动 ROS 的 shell 内：

```bash
python3 tools/collect_odom_test.py --duration 60 --output evidence/odom-trial-01
```

采集完成后复制 `measurements.template.json` 为 `measurements.json` 并填写同一车体固定点的实测值；位移单位米，时间窗口字段使用 bag_record_s。再运行：

```bash
python3 tools/analyze_odom_test.py evidence/odom-trial-01 --measurements evidence/odom-trial-01/measurements.json
```

输出目录必须是新名称。采集开始提示只证明 recorder 已订阅当前可见的白名单输入，不构成运动授权或输入健康结论；结束后看 `CHECK` 和 manifest。观察者 shell 的 rospack 路径也不证明已运行进程选用同一代码。

## 实际验证结果

证据根目录为 `.test-results/odom-preflight`（原归档未随仓库收录：`../../.test-results/odom-preflight/`），保留所有本次失败记录。

| 检查 | 实际结果 | 证据 |
|---|---|---|
| 采集6项、新分析17项、已有分析20项 | **43/43 通过**；包含真实 rosbag API/CLI、小包重映射、独立原点、冻结/回退/未来时间戳、测量未知及停止尾段检查 | `tools-final.log` |
| 采集器与独立 ROS master 贯通 | 定时结束与 Ctrl-C 两例通过，bag 均完整、含 tf_static；无采集器/recorder 的速度发布者，无 manual_intent 订阅 | `collector/TEST_ONLY-03/result.json`、各 case 的 manifest/bag/log |
| 刚采集 bag → 分析 | 两个实际生成的 TEST_ONLY bag 均可分析；LIO 与 EKF 人为相差100m原点，位移差仍为零；人工位移与物理停稳保持 unknown | `collector/TEST_ONLY-03/{complete,interrupt}/report.json` |
| 当前 driver 原有回归 | **50/50 通过**；未搬入合并固件工作树的其他测试或生产逻辑 | `install-check/driver-tests.log` |
| 临时 Catkin install 与实际 Launch 解析 | 安装完成；默认导航、接链导航、默认 driver、显式 wheel_speed 四种解析通过；安装脚本及 helper 导入与源内容匹配 | `install-check/result.json`、`execution.json`、各 dump YAML |
| 默认本地运行镜像的 ROS 包 | 导航/融合标准包存在；Slicer/Livox/FAST-LIO 仍依赖外部 overlay；未启动这些节点或验证 Jetson 部署 | `runtime-image-dependencies.json` |
| 原版 Guard 的独立 LIO 新鲜度 | **三项失败已复现**：LIO停止、header冻结、旧header递增时，其余信号仍新鲜，Guard仍输出非零；raw/TF/grid停止三个对照正常归零 | `lio-stale/baseline-red-final/result.json` |
| 只在临时副本应用候选补丁 | **6/6 情景通过、11项原 Guard 传输回归通过**；不等于生产链已修复 | `lio-stale/proposal-green-final/result.json`、`proposal-regression-summary.json` |

未隐藏的环境/开发失败：采集初次容器缺 `tf2_ros`，随后一次 Docker socket 权限错误，日志分别保留为 `collector/integration-01.log`、`integration-02.log`；改用已存在的运行镜像并按正常工具权限运行后，`integration-03.log` 通过。分析开发过程的红灯测试、候选 Guard 回归最初两次 rostest 资源路径错误也保留。这些环境错误与原版 Guard 的真实安全行为失败分开记录。

## 已知缺口与待确认补丁

LIO 新鲜度发现（原归档未随仓库收录：`../../proposals/LIO_FRESHNESS_FINDINGS.md`）与最小补丁（原归档未随仓库收录：`../../proposals/lio_freshness.patch`）已交付，**未应用**。只增加 Guard 对独立 `/lio/odom` 的接收时间、源时间戳新鲜度及推进检查；没有新节点或状态机。候选超时0.5s尚需现场采样核对。临时副本在 LIO 停止后约0.5s出口归零，是软件出口观测，不是物理停车时间。

为什么需要这项检查：EKF可在轮速仍有效时继续输出 odom/TF；当前 Guard 检查的地图、TF、raw 新鲜，不能证明 LIO 仍有效。**请先确认是否采用此最小补丁；本次没有擅自改生产链。**

仍待现场：当前 Jetson 实际 driver/overlay/configured ACK 与遥测、合并固件非零验收、轮速比例与地面低速起转/转向、真实位移及停车距离。离地通过不证明地面负载通过；0x10低速目标不保证低占空比。当前 DWA xy容差0.15m，Action成功不等于实际走足目标距离或精确停车。

## 保持不变的源码与复核

生产 launch、参数、driver/Guard/Mux 源码无本轮差异；未触碰 MCU 工作树。`git apply --check proposals/lio_freshness.patch` 通过，但没有 apply。

| 文件 | SHA256 |
|---|---|
| 当前生产 `arc_velocity_guard.py` | `865875d92845f824c51cacb4eb976a174576df929421c80e1d2003ad8036a3cd` |
| `collect_odom_test.py` | `ffa4340c98af2801f609a3fbb23b1e0fd20d8ddf91ebf461143380b3e4f1c802` |
| `analyze_odom_test.py` | `2d02dbf36c97192da6aff58bb7b5b0cf594ff16c12c66cbd368980f5f36f1ad2` |
| 复用的 `analyze_motion_bag.py` | `b1f7fc8f89b6a7fd2876837645dfd465a0161b03b886a16061a864b35bd7e6ac` |

工具测试复现命令（无 master、无设备；测试自己创建并删除微型 bag）：

```bash
docker run --pull never --rm --network none --cap-drop ALL --security-opt no-new-privileges --user "$(id -u):$(id -g)" -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONPATH=/workspace/tools --mount "type=bind,src=$PWD,dst=/workspace,readonly" --workdir /workspace --entrypoint /bin/bash industrial-cup/nav-slicer:20261023-dwa -c 'source /opt/ros/noetic/setup.bash; python3 -m unittest -v test_collect_odom_test test_analyze_odom_test test_analyze_motion_bag'
```

ROS采集集成 fixture 为 `tools/check_collect_odom_ros.py`，只允许在断网容器中运行；原版 Guard 复现命令见新鲜度发现文档。没有把这些测试加到生产 launch。
