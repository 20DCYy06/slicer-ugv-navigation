# GNSS 配置简化与 Jetson ARM64 验证（2026-10-30）

代码提交 `27488d3a75f3a827de432c9056e65aa34e428dac`，分支 `offline/rtk-reuse-20261030`；上一轮 GNSS 接入提交 `d1bba69c` 保留。

## 两份日常配置

- `src/ugv_bringup/config/slicer_gnss.yaml`：接收机序列号、115200波特率和NTRIP服务。各项已有中文注释；原运行参数值未改。凭据仍只通过受保护文件提供，不写进配置。
- `src/ugv_bringup/config/gnss_site.yaml`：基准、公共ENU原点、`map_from_enu: [tx米, ty米, yaw弧度]`、`antenna_in_base_link_m: [前米, 左米, 上米]`。未知值留null，不加额外状态开关。天线位置目前只记录，不自动补偿，不发布TF。

日常不必再同时填写旧 `gnss_enu_origin.json`；它仅保留给原 `--origin-file` 接口兼容。`preview_gnss_goal.py` 默认读取site配置，命令只需经纬度和椭球高；也能用 `--site-config` 指定另一份。旧命令行变换入口兼容，省略yaw仍不产生目标航向。

## 实际 ARM64 结果

Jetson为aarch64/Ubuntu22.04，使用Noetic/Focal ARM64容器。基础镜像 `industrial-cup/nav-slicer:20261023-dwa` 已具备同版本GNSS依赖，只补官方final快照的RealSense相机2.3.2、librealsense2 2.50.0和ddynamic_reconfigure 0.4.2；索引签名、大小和SHA256均核对，派生镜像在network-none中构建。

- 实际4包Catkin构建、安装完成。
- 153项tools +278项Catkin = **431项通过，0错误/失败/跳过**。
- 新配置及兼容接口另在本机Noetic执行47项针对性测试，通过；不与ARM431相加冒充同一套测试。
- ARM ROS+PTY集成PASS：1个串口owner，682字节RTCМ严格一致；发布74 fix、65 NMEA、14 diagnostics，q4/q5/q0、过期、CRC、重连与清理通过。这是合成数据，非真实RTK验收。
- 三份GNSS安装配置与源码逐字一致，已记录SHA256；安装脚本/模块/manifest与launch组合检查通过。
- 只读来源路径检查中编译传感器节点不可见，但按车端实际运行方式加载已有ARM `devel_navigation`、再加载我方install后，FAST-LIO/Livox和两滤波脚本均可发现；新GNSS解析到我方install。没有启动这些节点。
- 7个本轮读取的队友公开文件哈希未变，软件测试容器无残留。

## Jetson 上的安装位置

候选独立放在：
`/home/potato/slicer_ugv_ros1/.test-results/rtk-online-arm64-20261030/source`

两份配置的源码都在此目录的 `src/ugv_bringup/config/`，安装副本在 `install/share/ugv_bringup/config/`。修改源码配置后需要重新安装才能更新默认launch使用的副本，或者显式以 `config_file:=/workspace/src/ugv_bringup/config/slicer_gnss.yaml` 使用源码配置。

派生镜像：`ugv-online-rtk:arm64-20261030`，完整ID见 `jetson/ARM64_RESULT.json`。在候选目录将 `UGV_NOETIC_IMAGE` 设置成该镜像即可沿用现有shell；ROS_MASTER_URI/ROS_IP仍跟随当前导航，不引入旧11321 master。此刻只有Jetson，没有打开真实串口、接入真实差分或启动实车节点，也没有覆盖原部署目录。

最短复跑（Jetson宿主终端，仅软件测试）：

```bash
cd /home/potato/slicer_ugv_ros1/.test-results/rtk-online-arm64-20261030/source
UGV_NOETIC_IMAGE=ugv-online-rtk:arm64-20261030 UGV_SLICER_ROOT=/home/potato/Slicer_Workspace ./tools/test_noetic.sh
```

测试为无设备、network-none，队友目录只读。本轮不重建队友FAST-LIO/Livox，不替代真实传感器或实车验收。基础镜像仍保留，派生镜像可以单独选择/回退；原配置与提交均保留。

## 证据

`jetson/ARM64_RESULT.json` 含实际版本、结果、配置哈希；`jetson/test-container-isolation.json` 记录none网络、devices=[]、只读队友挂载。`noetic-arm64.log`、`jetson/.../junit/`、`pty-arm64.log` 是本轮输出。`site-config.patch` 为相对上一交付提交的增量；`source.tar.gz` 是本次真实构建的Git源码快照。依赖包及签名/索引见dependencies，不含真实凭据、历史bag或车辆日志。
