# 源码来源与离线复现

[返回首页](../README.md)

本仓库根是我方Catkin工作区，开发源码位于src。队友仓库与本仓库并列，reference只保留明确来源的依赖与固件参考，不作为第二套待编译主线。

## 1 主线与来源

| 位置 | 用途 |
|---|---|
| [src/ugv_bringup](../src/ugv_bringup) | 定位/导航/GNSS launch与配置 |
| [src/ugv_navigation](../src/ugv_navigation) | 标准Action工具、深度适配及测试 |
| [src/ugv_slicer_control](../src/ugv_slicer_control) | 既有Slicer控制链的选定适配与独立driver |
| [src/cup_gnss](../src/cup_gnss) | GNSS成熟后端适配、质量与ENU工具 |
| [reference/Slicer_Workspace](../reference/Slicer_Workspace) | 选定传感器、通信与固件来源快照 |
| [reference/field_verified_3849abee](../reference/field_verified_3849abee) | 旧实测固件参考，不当作最新安装身份 |
| [docs/history](history) | 原候选历史文档，保留日期与当时状态 |
| [evidence](../evidence) | 已有软件结果和本次整理检查 |

基线为27488d3。队友方案参考 [potato-ugv-online-navigation-radio @643a704](https://github.com/20DCYy06/potato-ugv-online-navigation-radio/tree/643a704c5fa042b5e9f08d3c37ddad3a38e8b56a)，侧重Gmapping+B样条+TEB与电台通信。本仓库已有实测主线是GlobalPlanner+DWA，新增在线地图与GNSS候选。参考树来自此前交付包，不能默认与该GitHub提交逐字节相同。

## 2 三种环境用途

| 用途 | 依赖来源 | 当前能验证什么 |
|---|---|---|
| 阅读源码 | 本仓库src与reference | 接口、配置、控制逻辑、来源 |
| 我方四包离线检查 | 依赖齐全的Noetic镜像＋reference源码 | 构建/测试/安装、lookup/import/launch解析 |
| 实机运行 | 已编译且架构匹配的Slicer overlay、SDK和设备配置 | 需在现场单独验收；本次未执行 |

reference下的slicer_navigation、slicer_wasd_control是裁剪源码，未收完整CMake构建内容；arm_cloud_filter在已有环境靠Catkin wrapper启动。FAST-LIO/Livox源码保留，但Livox SDK2完整源码和确定版本尚未收齐。

因此，四包软件检查成功不代表外部传感器可执行节点已构建。现有check_noetic_install.py会将缺少的外部节点标为UNAVAILABLE，而不把它们全当作该软件检查失败。

## 3 离线检查入口

在Linux、WSL Linux或相应开发机中，从仓库根执行。先准备与机器架构匹配、依赖齐全的Noetic环境镜像，再设置其实际名称：

```bash
export UGV_SLICER_ROOT="$PWD/reference/Slicer_Workspace"
export UGV_NOETIC_IMAGE="本机已有的依赖镜像名"
bash tools/test_noetic.sh
```

镜像名是待替换项。此前验证使用过的标签是本地环境记录，不代表公开镜像仓库一定可拉取。根Dockerfile列出我方所需系统包，可作为重建环境的入口；在新机器完整重建仍需验证依赖可取得和架构兼容性。

脚本使用无网络、无设备容器，将参考树只读挂载到/slicer，补充ROS_PACKAGE_PATH，不source收集的ARM构建产物。它构建src四包，执行tools与Catkin测试，安装并检查入口，在当前仓库生成build/devel/install/.test-results。

**必须显式设置UGV_SLICER_ROOT**：现有noetic.sh保持原样，默认寻找仓库同级的Slicer_Workspace；本次参考目录已移到reference中。

可以选择已验证的另一份依赖树，但要记录其来源版本。队友GitHub快照本身未携带完整FAST-LIO/Livox/SDK/overlay，不能仅把目录名改成同级便认为依赖齐全。

## 4 实机环境与离线参考的区别

tools/slicer_shell.sh保留原实机环境包装行为：使用已构建Slicer的catkin_ws/devel_navigation/setup.bash，以及本工程install/setup.bash；它还使用现场ROS地址和显式设备映射。

reference树没有devel_navigation。不要将上述离线UGV_SLICER_ROOT直接当作实机运行overlay。实际运行须使用已核对的外部安装环境，并只选择一套传感器、EKF、Gmapping、Guard/Mux和底盘串口所有者。

本次没有修改这些运行脚本、启动默认、控制参数或算法。入口行为见 [架构](01_architecture.md)。

## 5 Git与ZIP应保留哪些文件

- src和tools保持基线字节；选定依赖、各许可证与来源说明保留。
- 两处.generated配置是launch真实输入，必须跟踪；固件参考的build/generated.h和manifest也保留。
- reference/CATKIN_IGNORE与evidence/CATKIN_IGNORE防止根目录递归发现参考包，不影响通过指定catkin_ws/src作查找。
- 根.gitignore排除构建缓存、bag、二进制、凭据和运行备份，保留精选evidence。
- Livox原.gitignore忽略package.xml；本次在该参考副本中添加反向规则，确保上传不会漏包声明。原规则与这次差异有来源记录。
- ZIP保留原归档的可执行模式，并使用LF文本；.gitattributes明确脚本行尾。建议在Linux/WSL中解压后再加入Git，保留脚本执行位；Windows目录中的普通复制不能作为执行位已保留的证明。

历史原文引用的bag和部分.test-results没有随源码包附带。整理时把这些失效本地链接改成“原归档未收录”的文本说明，未伪造缺失文件。历史命令保留其原语境，不是当前推荐启动流程。

## 6 许可与身份

[THIRD_PARTY_NOTICES](../THIRD_PARTY_NOTICES.md)汇总许可文件。FAST-LIO的package.xml写BSD、LICENSE却为GPLv2文本，保留并标明这项差异，不能给全仓统一贴新许可。

控制包SOURCES.json保留历史来源，其中Guard的旧hash已不等于当前文件；本次不改历史记录，当前发布副本身份以 [repository_files.json](../reference/provenance/repository_files.json) 为准。

正式发布只需在确定目标GitHub仓库后提交这一目录；2026-10-30完成本地整理；独立发布目标为 `20DCYy06/slicer-ugv-navigation`，整理过程未更改现车工程。
