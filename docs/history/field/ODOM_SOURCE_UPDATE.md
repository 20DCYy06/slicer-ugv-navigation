# 源码重新安装参考（本次已由助手完成）

本次已由助手完成同步、Jetson ARM64安装和参数核对，无需再执行本页。当前请直接按[现场操作单](ODOM_FIELD_CHECK.md)启动；下列步骤仅用于以后重新安装。原问题是主工程src/install均为旧版，单独复制YAML不足以更新driver。

当前源码包基线为 `598c899`（ROS控制修复为f4bd9ee，另含TF采集修复），包含已批准的LIO新鲜度补丁。只包含我方 `src/CMakeLists.txt`、三个ROS包及环境/采集分析工具、相关测试和udev规则；不包含固件、队友源码、build/devel/install、日志或bag。

## 1．WSL复制准备好的包

```bash
cd /home/rescue/slicer_ugv_ros1
scp .test-results/jetson-odom-update-20261025/odom-ready-598c899.tar.gz cup-jetson:/home/potato/slicer_ugv_ros1/
```

包SHA256：`16bfbc745185a05b7d210de8d4e38764c0ea71a90c83610216a7ca3f536e08c9`。

## 2．Jetson宿主机解包

保持held，结束A与D的旧启动器；定位可继续。退出当前 `cup@...:/workspace` 容器，回到 `potato@...` 宿主机。以下只更新独立新工程，不进入队友树：

```bash
cd /home/potato/slicer_ugv_ros1
sha256sum odom-ready-598c899.tar.gz
```

与上面的哈希一致后执行。第一条tar只保存当前三个包和Shell入口，保留已有车端改动；不扫描其他目录：

```bash
mkdir -p evidence
tar --exclude='__pycache__' --exclude='*.pyc' --exclude='backup*' -czf "evidence/ros-source-before-update-$(date +%Y%m%d-%H%M%S).tar.gz" src/CMakeLists.txt src/ugv_bringup src/ugv_slicer_control src/ugv_navigation tools/slicer_shell.sh
tar -xzf odom-ready-598c899.tar.gz
```

## 3．无设备、断网构建安装并只解析Launch

仍在Jetson宿主机项目根执行。这是一次构建命令，不启动master、driver、导航、传感器或控制权客户端，也不映射串口。直接从 `/opt/ros/noetic` 开始，避免加载车端旧devel overlay；强制重新配置并明确写入 `/workspace/install`。

```bash
set -o pipefail
docker run --pull never --rm --network none --cap-drop ALL --security-opt no-new-privileges --user "$(id -u):$(id -g)" -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 -e ROS_HOME=/tmp/ugv-build-ros -e PYTHONDONTWRITEBYTECODE=1 --mount "type=bind,src=$PWD,dst=/workspace" --workdir /workspace --entrypoint /bin/bash industrial-cup/nav-slicer:20261023-dwa -c 'set -e; source /opt/ros/noetic/setup.bash; catkin_make --force-cmake install -j2 -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/workspace/install; source /workspace/install/setup.bash; roslaunch --dump-params ugv_bringup slicer_driver.launch config_file:=$(rospack find ugv_bringup)/config/slicer_driver_wheel_speed_reference.yaml control_mode:=wheel_speed publish_tf:=false' 2>&1 | tee evidence/ros-install-598c899.log
```

正常最后输出四项，且无错误退出：

```yaml
/slicer_c30d_driver/config_file: /workspace/install/share/ugv_bringup/config/slicer_driver_wheel_speed_reference.yaml
/slicer_c30d_driver/control_mode: wheel_speed
/slicer_c30d_driver/observe_motor_registers: false
/slicer_c30d_driver/publish_tf: false
```

构建或参数解析失败就保留 `evidence/ros-install-598c899.log`，不继续启动driver。没有切换到PWM兜底。

## 4．返回现场操作单

构建命令结束后仍在宿主机；按[现场操作单](ODOM_FIELD_CHECK.md)重新进入A/D/B的ROS环境。已有 `/dev/c30d` 可直接使用，无需重复安装udev规则。所有driver启动、release与运动仍由用户现场执行，先做只读预检；本包含独立LIO停更检查，仍不代表合并固件已通过非零验收。

本包已在Jetson现有arm64运行镜像中以无设备、断网方式构建安装并解析上述参数，日志为 `.test-results/jetson-odom-update-20261025/arm64-install.log`。完整实际结果见[车端交付记录](ODOM_READY_20261025.md)，不将安装或软件模型结果当实车运动结果。
