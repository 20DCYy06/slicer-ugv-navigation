# GNSS/RTK 接入与最短接收检查

本轮为离线软件交付，未连接车辆、串口或真实差分服务，未部署。工作分支 `offline/rtk-reuse-20261030`，基线 `c8abe39e74792b13e2c376b0bad47fbf3f0b0340`。

## 迁移内容

来源为只读旧工程 `/home/rescue/industrial_cup_ros1` 的 HEAD `65308aa68ddf640cb3f0012393e6d02b3cae1947`，现用包 `catkin_ws/src/cup_gnss`；包级最近提交 `8281c709b49ef6af97ee8dde9604bf7eea6fee51`。迁移的是默认 mature 后端，不是冻结 legacy。

| 旧文件（相对旧包） | 新位置与处理 |
| --- | --- |
| `scripts/gnss_node.py`、`scripts/gnss_status.py` | `src/cup_gnss/scripts/`，逐字复用 |
| `src/cup_gnss/{core,sources,quality,ntrip,upstream_ntrip,upstream_nmea,enu}.py`、初始化文件、NMEA 锁定副本及 manifest | 同包结构，逐字复用；不重写接收、解析、NTRIP 或 RTCM 回注 |
| `setup.py`、公开 service JSON、NTRIP 许可证、`tools/rtk_evidence.py` | 原样复用；evidence 工具仅为已有数学/历元测试依赖，不加入运行链 |
| 11 个单元测试及 ROS/PTY drill | 复用；一个测试增加原有 ACK 等待，drill 改为本地测试 helper，不依赖旧导航管理器 |
| `CMakeLists.txt`、`package.xml` | 精简安装项和依赖，注册必要测试；未迁移旧双 master、浏览器、回退管理器或全局 EKF |

接收运行源码及 ENU 数学均未改。上游版本为 `ntrip_client 1.4.1`、`nmea_navsat_driver 0.6.1`，后者两份 vendored 文件保留原 BSD 头和 SHA256 manifest。逐文件旧路径、新路径、哈希及实际改动见交付包 `evidence/migration-source-final.json`。

现有单串口线程负责 NMEA 读取和 RTCM 写入；NTRIP 仍通过它的有界请求/ACK 回注。断线清会话与旧样本，保留原重连、帧校验、质量与新鲜度规则。没有第二个 GNSS serial node。

## 后续现场：三个入口

使用前须确认 ARM64 安装检查通过，镜像含上述两个 ROS 依赖与 `python3-serial`。构建/安装检查不打开接收机；以下是**后续现场接收命令**，需要接收机到场后执行。

在车端宿主机进入已安装的独立工程。`ROS_MASTER_URI` 必须与当前导航完全相同，`ROS_IP` 是车端可回连地址。例如导航使用电台地址时：

```bash
cd /home/potato/slicer_ugv_ros1
export ROS_MASTER_URI=http://192.168.10.33:11311 ROS_IP=192.168.10.33
UGV_GNSS_DEVICE=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5185028488-if00 ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_gnss.launch
```

这是一个独立接收节点，NTRIP 默认关闭。示例 by-id、VID/PID `1a86:55d3`、序列号 `5185028488` 来自旧接收机，现场必须核对；不能用 C30D 的设备代替。容器将指定设备映射为 `/dev/gnss`，不启动底盘。配置不是该接收机时使用 `config_file:=/workspace/src/ugv_bringup/config/你的公开配置.yaml`，不能用空身份绕过检查。

另开同环境的 ROS shell 查看状态：

```bash
./tools/slicer_shell.sh
rosrun cup_gnss gnss_status.py --once --json
```

不要启动旧 11321 master 或桥接管理包装；若当前导航走其他 IP，改为同一 `ROS_MASTER_URI`/车端 `ROS_IP`，而不是再开一个 master。普通本地导航不需要 GNSS；`slicer_online.launch` 不包含它。独立 GNSS 节点退出也不终止组合中的本地导航。

ENU 预览在本地工程根运行即可；下面只有**已知合成变换**，东 10m 经 `map <- ENU=(2m,1m,π/2)` 得到 map 约 `(2,11)m`：

```bash
python3 tools/preview_enu_goal.py --east 10 --north 0 --map-from-enu 2 1 1.5707963267948966
```

真实经纬度入口、显式公共原点、椭球高及可复跑合成示例见 [GNSS_ENU_PREVIEW_20261030.md](GNSS_ENU_PREVIEW_20261030.md)。未实测 ENU↔map 配准前只做预览。省略 yaw 的预览不含 yaw；现有 `goal.py` 默认 `--yaw 0`，不得把预览机械拼成目标命令并声称“无需朝向”。本轮不自动发目标，也不修改该策略。

## 配置与接口含义

- `src/ugv_bringup/config/slicer_gnss.yaml`：串口、身份、115200 波特率、原有龄期、NTRIP 参数。旧公开 CORS 端口 8003 声明 CGCS2000，但未核实，NTRIP 保持关闭。确认有效服务、挂载点、基准后才启用；不从历史公开声明推定账号仍有效。
- 如需差分，使用现有 `ntrip_credentials_file` 文件接口。宿主机已有凭据可经 `UGV_NTRIP_CREDENTIALS=/实际受保护文件` 只读挂载至 `/run/secrets/slicer_ntrip.json`；加载器要求当前 UID、regular mode 0600。账号/密码不放 launch、参数或命令行，本包没有真实凭据。
- `src/ugv_bringup/config/gnss_site.yaml`：日常只填这份有中文注释的场地配置，包含基准、公共原点、`map_from_enu` 和天线安装位置；未知值留 `null`。原点为经纬度（度）与椭球高（米）；配准为 `[tx米, ty米, yaw弧度]`，天线位置为车体前/左/上（米），目前只记录、不自动补偿。旧 `gnss_enu_origin.json` 保留给显式 `--origin-file` 调用兼容，不必两份重复填写。本工具只做 WGS84 数学，不把 CGCS2000 改标签当作转换。
- `/gnss/nmea`：原始句 `std_msgs/String`；`/gnss/fix`：`sensor_msgs/NavSatFix`；`/gnss/diagnostics`：`DiagnosticArray`。原始 q4/q5 在诊断保留，标准 `NavSatStatus` 不能区分 Fixed/Float。判 RTK 要看 `fresh_rtk_usable/reason`、龄期及当前会话完整 RTCM 写入，不能只看 q4。
- `gnss_link` 表示天线位置，没有未知杆臂补偿或静态 TF。NavSatFix stamp 是主机接收/发布适配时刻，不是接收机测量时刻；无 PPS 同步保证。高程为有效 MSL+geoid，缺项 NaN；协方差 UNKNOWN，不能当作已测精度。
- Gmapping 保持 `map→odom`，现有 LIO+wheel EKF 保持 `odom→base_link`；GNSS 不发布 TF、不覆盖 `/odom`，也不把未配准 ENU 填入局部 EKF。Guard/Mux/driver/0x16 与现有规划配置未变。

## 实际软件验证与回跑

已在本机 amd64 Noetic 镜像 `ugv-online-offline:20261030`（ID `88f218098c63…`）、无设备、`--network none` 中完成 Catkin 构建/安装：**147 项 tools + 278 项 Catkin = 425 项通过，0 失败/跳过**。安装检查确认 GNSS 两脚本、辅助模块、锁定 NMEA manifest、外部包可发现及 launch 组合。已有源参考仅以 `/slicer` 只读挂载；不运行其脚本。

复用 ROS+PTY drill 实测：一个串口 owner；fake caster→RTCM→PTY **682 字节逐字相等**；真实发布 69 fix、61 NMEA、13 diagnostics，覆盖 q4/q5/q0、差分龄期、坏 CRC、断流恢复及清理。测试全部是合成数据；没有物理 RTK/导航通过结论。第一轮曾暴露测试线程读取字节早于 ACK 计数更新的竞态，只给测试增加原有有界等待，未改生产链或放宽断言。

```bash
UGV_NOETIC_IMAGE=ugv-online-offline:20261030 ./tools/test_noetic.sh
UGV_NOETIC_IMAGE=ugv-online-offline:20261030 ./tools/noetic.sh env CUP_NAV_ISOLATED_TEST=1 ROS_MASTER_URI=http://127.0.0.1:11331 python3 src/cup_gnss/tests/ros_mature_gnss_drill.py --output /workspace/.test-results/rtk-pty-rerun
```

drill 输出目录须是新目录。第二条的 11331 仅是 network-none 内临时独立 master，不是车端新增 master。现有 sensor 依赖的源码可发现不等于 FAST-LIO/Livox 等 ARM 设备链已在本机完整构建。

## 现场最短待办与回退

1. 不开运动，确认新接收机身份、唯一串口 owner、天线安装位置/无旧杆臂；以与导航同一个 master 启动 GNSS，先检查接收频率、q、有效性与掉线状态。
2. 在可接收卫星的环境核对账号、服务基准/历元、设备时间与主机时间。启用已核实 NTRIP 后检查 CRC、类型、写入字节/龄期、raw q 与 `fresh_rtk_usable`；写成功仍不等于接收机使用差分。
3. 留存参数与 `rosbag record -O gnss_receive /gnss/nmea /gnss/fix /gnss/diagnostics /tf /tf_static /odom /lio/odom /wheel/odom`；静止/短时断流后应失效，恢复须新鲜数据，不能重发旧缓存为有效定位。现场动作另行确认。
4. 实测公共原点、基准、天线到 `base_link` 外参、ENU↔map 关系与时间偏差；本轮不填假值。此后才评估真实坐标目标及持续纠偏。

持续 RTK 纠偏需单独决定全局职责：保持 Gmapping 独占 `map→odom`、GNSS 仅观测/配准；或经对照后交给全局融合独占该 TF，并关闭 Gmapping 的 TF 输出。两者不可并发争用；本轮均未新增实现。回退可停止 GNSS（Ctrl-C）并继续既有本地导航；软件迁移整体可回到基线 `c8abe39`，不要覆盖已有修改。
