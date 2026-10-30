# Noetic 离线构建与安装验证（2026-10-30）

本轮在独立副本完成真实 Catkin 构建、测试和安装，不连接车辆，不部署/刷写，不发布实车目标。ROS 模型测试仅运行在无设备、`network none` 的本机容器及其独立 master。

## 实际结果

| 项目 | 本轮结果 |
|---|---|
| 候选输入 | ZIP MANIFEST 433/433 文件哈希一致 |
| 补丁接入 | `git apply --check` 退出0，随后应用；无冲突 |
| Catkin | 三个我方包的 `catkin_make -j2` 和 `catkin_make install -j2` 通过 |
| tools | 126项，通过126，0跳过 |
| Catkin nose/rostest | 官方汇总166项，0错误、0失败、0跳过 |
| 安装检查 | 3个包来自真实install；6入口可执行并可导入；7辅助模块导入来源正确；9份复用配置齐全 |
| 安装后launch | 默认、连接Guard、传感器、真实RealSense include、禁用深度5分支，官方XmlLoader全部展开；未启动这些节点 |
| 保护范围 | 15份既有控制/Guard/Mux/EKF/DWA相关文件与本轮独立副本基线字节一致 |

126项tools与166项Catkin是两组不同测试，共292项；安装检查的条目和5个解析分支不再重复加入测试总数。此前候选的228项结果属于上一阶段，不能与本轮相加。

## 本轮修正

1. `source devel/setup.bash` 会替换 `ROS_PACKAGE_PATH`，丢掉只读队友参照。测试入口在source后恢复该路径，不source队友ARM的devel/install。
2. 加入实际安装检查：用空白环境仅source Noetic与install，切到`/tmp`，核对脚本/辅助模块来源和外部包发现，不执行入口的main块。
3. 模拟share安装测试原先只改ROS路径，仍被真实`CMAKE_PREFIX_PATH`带回当前源码。仅在该测试内排除当前devel/install前缀，保留所有原断言；另有真实安装检查交叉验证。
4. 本地原镜像缺RealSense。仅从镜像原有ROS final官方快照下载3个amd64包，核对签名、索引和包SHA256后，用已有基础镜像在`--network none`下离线安装：相机2.3.2、librealsense2 2.50.0、ddynamic_reconfigure 0.4.2。

未修改底盘协议、MCU、0x16、Guard/Mux安全逻辑、EKF融合、DWA评分或速度参数。补丁应用在当前本地修改的副本上，保留了较新的bag目标残差分析代码，没有整目录覆盖。

## 最短复跑

本机已存在镜像`ugv-online-offline:20261030`。在本项目目录执行：

```bash
UGV_NOETIC_IMAGE=ugv-online-offline:20261030 ./tools/test_noetic.sh
```

相邻`../Slicer_Workspace`自动只读挂载；其他位置可显式设置`UGV_SLICER_ROOT`。每次结果保存在`.test-results/run-*/output.log`、`exit-code`和`junit/`。不运行队友`run.sh`。

交付包提供`dependencies/Dockerfile.offline`和本次3个包；已有基础镜像时，可在交付包根目录重建同类测试镜像：

```bash
docker build --network none --pull=false -f dependencies/Dockerfile.offline -t ugv-online-offline:20261030 dependencies
```

本次实际使用本机已有的Noetic基础镜像；未声称从空机器重建原始Dockerfile或全部传感器SDK。完整镜像不在交付包中，身份记录在`verification/build-environment.json`。

## 明确未通过/未覆盖

- 外部9个包名均可发现；只读源码参照不等于可运行的外部安装。`arm_cloud_filter.py`源文件不可作为可执行节点发现；FAST-LIO和Livox编译节点未在本轮x86参照中构建。它们的可用性单独输出，不计为安装通过。
- 未启动新在线导航完整传感器图；本轮rostest覆盖项目原有软件模型，新组合入口进行真实安装后的解析检查。
- 尚无新链同步D455深度/CameraInfo/hits/rays/地图bag。全无效深度、断流、低障碍标记与移除清除，不能由Python几何检查替代现场验证。
- ±60m初始地图的Jetson内存/CPU/延迟、唯一`map→odom`及`odom→base_link`发布者、地图更新和长距离规划仍需上车核对。
- 以后获授权部署前，确认ARM侧外部传感器节点/包装脚本可运行；再进行静止相机/TF/Costmap检查、短二维目标与静态绕障、取消/hold及停车验证。维持3849abee固件基线，不把低速专项作为此次接手前置。
- ENU工具仍只计算给定变换下的预览；飞机正式协议、原点/单位/时序与物理配准未补齐，也未发送任何目标。

软件通过不代表实车导航通过。原工作目录及其未提交改动保留，候选留在独立副本，未自动合并到原分支。
