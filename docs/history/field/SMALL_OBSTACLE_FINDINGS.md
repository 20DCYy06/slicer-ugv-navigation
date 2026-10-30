# 小障碍分层观测准备（2026-10-26）

当前证据支持追查“哪一层丢失了障碍回波”，尚不能确定小障碍漏检根因。未调整导航分辨率、实际高度阈值、LIO、EKF、MCU 或 Guard；未连接实车。透明、低反射、遮挡目标可能没有有效雷达返回，软件旁路投影无法补出不存在的回波，也不能保证检测透明瓶。

## 已读证据与缺口

最新队友只读源码根目录：
`deliverables/slicer_odom_review_20261026/external_source/Slicer_Workspace/`。

主要文件：`config/slicer_config.yaml`、`catkin_ws/src/slicer_bringup/launch/sensors.launch`、`sensor_geometry.py`、`catkin_ws/src/slicer_navigation/scripts/arm_cloud_filter.py`、`self_scan_filter.py`。我方下游使用 `src/ugv_bringup/config/costmap_common.yaml`。

本次快照没有队友运行 `.generated/scan.yaml`、`lio.yaml`、`sensor_launch.yaml`、`MID360.json`，也没有 FAST-LIO/Livox 的完整版本源码或本次小障碍点云/bag。因此，源码配置值不能替代参数服务器快照；Livox 实际发布格式、FAST-LIO 盲区、内部点筛选及原投影 `use_inf` 均须从本次现场资料确认。

采集器会保存投影节点的实际参数，以及 arm/body filter 和 geometry 的 `config_file` 路径；后三个节点从 YAML 直接读取配置，未把掩码和安装值全部发布为 ROS 参数。因此 manifest 不能独立还原其运行加载值。现场另保留本次明确使用的 `slicer_config.yaml` 中 `sensors`、`frames`、`chassis`、`navigation` 相关节与文件 SHA256，且记录是否启动后改过文件；不要把读取时文件等同于已加载值。无需让采集器自动沿参数路径读取文件。

## 实际链与可观察边界

| 环节 | 源码接线 / 参数 | 对判断的限制 |
|---|---|---|
| MID360 → FAST-LIO | `sensors.launch` 启动 `livox_ros_driver2_node`、`fastlio_mapping`；加载队友生成文件 | `/livox/lidar` 可能是 Livox CustomMsg；先核对实际 Topic 类型。RViz 的 PointCloud2 display 不能直接显示 CustomMsg。 |
| LIO 输出 | 导航订阅 `/cloud_registered_body` | **这是 LIO 输出，不是原始雷达回波。** 此处无点不能证明 MID360 没收到回波。 |
| LIO 筛点配置 | launch 显式设 `point_filter_num=3`、`filter_size_surf=0.15`、`filter_size_map=0.25` | 不能据此断言 body cloud 经 15 cm 体素化。body 发布使用何种中间点集须核对车端 FAST-LIO 版本；世界点云/建图降采样与 body 输出不能混为一谈。 |
| 安装 TF | 配置 `base_link→laser_link=(0.07,0,0.18)m`，无旋转；Livox 内参 `p_IMU=R*p_LIDAR+t`，`t=(-0.011,-0.02329,0.04412)m` | `sensor_geometry.py` 得到 `base_link→body=(0.081,0.02329,0.13588)m`。LiDAR 内参不是安装高度。以上均为配置值，不能代替现场安装测量。 |
| arm cloud filter | `/cloud_registered_body` → `/cloud_nav_filtered`；先变换到 `base_link`，仅屏蔽 `x∈[-0.10,0.17]m`、`|y|≤0.11m` 且 `z≥0.23m` 的点；被屏蔽 XYZ 写 NaN | 同一 XY 包络内的低障碍按源码应保留。TF/格式错误时该节点停止输出，不能把无输出解释为无障碍。 |
| 标准 3D→2D 投影 | `/cloud_nav_filtered` → `pointcloud_to_laserscan` → `/scan_raw`；配置入口高度 `[0.04,0.50]m`、距离 `[0.05,20]m`、角步长约 `0.5°` | 高度应在 `base_link` 中判断。低于下限、超出上限、距离窗外或未命中的点不会成为有限 scan 距离；同角度格只保留最近返回。实际生成参数缺失，须现场核对。 |
| 车身 scan filter | `/scan_raw` → `/scan`；`|x|≤0.125m` 且 `|y|≤0.10m` 的有限回波写 **NaN**；另发布 160° 的 `/scan_front` | `/scan` 本身保留 360°，不是 160° 前视限制。NaN 用于避免沿被车身遮住的方向清空障碍。 |
| 我方 obstacle layer | LaserScan marking+clearing；`obstacle_range=3m`、`raytrace_range=4m`、高度 `[-0.05,0.50]m`、`inf_is_valid=true`、`observation_persistence=0`；足迹清除开启 | 消费的是投影后的平面 scan，不能恢复前面丢失的 3D 高度。若上游输出 +inf，沿有效射线可清空旧障碍；NaN 不等价于 +inf。足迹内障碍也受 footprint clearing 影响。 |
| 栅格 | 我方共享配置 `resolution=0.05m` | 仅凭物体尺寸小于格宽，不能判定漏检由分辨率造成。先确认有限 scan 返回能否到达 marking 范围内，再看是否标格、随后被清除。 |

## 已准备的观察工具

现有队友 `slicer_bringup sensors.launch` 已能独立启动原感知链（内含原 EKF，不含底盘 driver）；本次没有复制或修改该启动链。现场是否已运行由用户确认，避免重复启动。仅看静态链时无需新 launch：

```bash
rosrun rviz rviz -d $(rospack find ugv_bringup)/rviz/small_obstacle.rviz
```

该 preset 以 `base_link` 为固定坐标，显示 LIO body cloud（橙色）、arm 过滤后 cloud（青色）、`/scan_raw`（绿色）、`/scan`（白色）、两张 costmap、足迹和 DWA/TEB local plan（绿色/紫色）。逐项切换 display 可比较同一物体在过滤前后的变化。没有发目标、定位或速度的工具；不启动任何传感器、导航或控制节点。原始 Livox CustomMsg 不在这些 cloud display 中。

只有需要在**不重启原感知、不改队友 `.generated`** 的情况下比较一项投影参数，才使用新增旁路：

```bash
roslaunch ugv_bringup small_obstacle_projection.launch
```

它仅将已有 `/cloud_nav_filtered` 投影到 `/ugv/perception_test/scan`，没有接入 move_base。在 RViz 手动启用 “Optional diagnostic projection” display 后才有订阅；标准投影节点会按订阅需求接收点云。参数文件 `config/small_obstacle_projection.yaml` 取最新源码中的高度、距离、角步长作为**诊断基线**，其余值显式列出便于复现，不声称等于车端运行参数。其中 `transform_tolerance=0.10s` 是本工具的诊断选择，**不是复制原投影节点此前的 `0.05s`**；本次实车值仍以参数快照为准，不将该差异写成漏检原因。没有该 launch，原链各边界仍可观察；其具体用途是独立比较投影参数，原链的 scan 无法在保持原配置运行的同时承担这个对照。

进行单变量对照时，先复制我方基线 YAML 并仅改一个有现场证据支持的参数，通过 `projection_config:=绝对路径` 加载。先保存实际原节点参数，明确两份配置差异；不要把扩大高度窗造成的地面点回流误当检测改善，也不要把诊断 topic 改成导航 scan。

## 离线验证与结论边界

`tools/check_small_obstacle_ros.py` 只允许无设备、`--network none` 的 Docker 容器，启动独立 loopback master。它运行快照原 `sensor_geometry.py`、`arm_cloud_filter.py`、`self_scan_filter.py` 以及实际标准 `pointcloud_to_laserscan_node`，输入合成的 body 点云，不运行 LIO、EKF、底盘、move_base 或目标/速度发送器。

2026-10-26 已在本地 amd64 镜像 `industrial-cup/nav-slicer:20261023-dwa`、无设备且 `--network none` 的独立 master 中实跑：

- 过滤与投影结果（原归档未随仓库收录：`../../.test-results/navigation-obstacles-20261026/small-obstacle-projection/result.json`）：**11/11 通过**。0.5/1/2 m、z=0.06m 的返回经过原过滤节点后保留；z=0.041/0.499m 保留，z=0.039/0.501m 被投影窗剔除；高 arm 点变 NaN，低点保留；车身内 scan 回波变 NaN。另覆盖距离窗、同角度最近返回、移除目标后仅地面返回产生空 scan 格。结果包含输入坐标、有限距离、屏蔽点数、实际诊断节点参数与只读源码 SHA256。
- 既有 DWA / costmap 回归结果（原归档未随仓库收录：`../../.test-results/navigation-obstacles-20261026/dwa-rolling-regression/result.json`）：**5/5 通过**，其中 `scan_wall_mark_and_ray_clear` 在全局和局部地图中标出 lethal=100，移除合成墙并继续发送新 scan 后，检查区域全为 0。此测试直接输入理想 LaserScan，未经过前一项的点云链；两项测试不能拼成端到端实车检测结论。

这些结果分别证明给定配置下的过滤/投影行为与合成 scan 的标记/清除行为。**空 scan 格的 +inf 仅表示该次投影没有有限返回，不能作为物理空间无障碍的证据。** 透明瓶、遮挡、低于高度窗的目标都可能产生相同表象；本次测试不是实车小障碍检测率，也不证明静止水瓶可以稳定识别。

现场后续依据同一静止障碍、同一时间窗，按以下顺序判断：

1. 原始 Livox 返回与 LIO body cloud 是否都有目标对应点。缺一层证据就保留该层未知；尤其不能用 body cloud 代替原始返回。
2. body cloud 有点时，arm 前后点是否变化，正确 `base_link` 高度是否落在运行投影窗内，是否命中车身掩码。
3. `/scan_raw`、`/scan` 是否在对应角度有有限距离；同时核对实时时戳、发布率和 TF 可用性。
4. scan 有有限返回时，再看 3 m marking 范围、实际 costmap lethal 格，以及目标移开后是否按射线清除；结合参数快照区别“从未标出”与“标出后被清除”。

水瓶静止期间若点云、scan 或障碍标格时断时续，判为**静态检查不通过**；不以间歇出现、全 +inf 或之后地图变空认定检测可靠。记录首次消失的环节，保留失败资料，暂不进入绕障运动。

未取得本次现场数据前，不把透明瓶、距离稀疏、遮挡、安装误差、LIO 筛点、高度窗、自过滤或栅格分辨率中的任意一项写成既定根因。
