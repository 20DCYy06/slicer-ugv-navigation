# Windows/WSL实时RViz观测

本页只处理观测电脑，不改变车端ROS_IP、导航、控制或电台配置。WSL启动已简化为 `./tools/rviz_wsl.sh perception`（小障碍分层观察）或 `./tools/rviz_wsl.sh navigation`（导航路径观察），都没有发目标工具。

2026-10-26本次核对：WSL为 `192.168.3.208`，到 `192.168.10.33` 的路由经 `192.168.3.68`。脚本默认使用此路由的源地址作为ROS_IP；若已设置ROS_IP则尊重该值。下面 `172.20.10.*` 是此前热点观测的历史地址，切换网络后不要原样套用。脚本不修改路由，也不代替Docker Desktop的host networking设置。

**2026-10-26恢复结果：窗口绘制已由用户确认正常。** 本机WSL重启后重新打开Docker与RViz，用户回复“界面和地图都正常”。最终只读核验已收到真实scan、odom及静态TF；当时导航尚无costmap发布者，因此不能把可见网格／扫描等同于两层导航代价地图已恢复。确认记录见`.test-results/rviz-wsl-20261026/gui-confirmation.json`。

## 此前热点观测的地址（历史）

- Jetson热点：172.20.10.4；电台网卡：192.168.10.33。
- Windows WLAN：172.20.10.7，InterfaceIndex=9；WSL镜像网卡eth3同地址。
- ROS master及节点公布192.168.10.33；只连热点master能列出节点，不代表能收到topic数据。
- Docker daemon实际为Docker Desktop。其`--network host`不是共享WSL的路由表，单独给WSL加路由不足以让RViz容器联网。

地址、接口编号随网络变化，以上不是固定安装参数。ROS1通过节点公布的地址建立实际连接，并非把topic广播到电台。

## 1. Windows与Docker网络

在**Windows管理员PowerShell**添加本次临时主机路由：

```powershell
New-NetRoute -DestinationPrefix '192.168.10.33/32' -InterfaceIndex 9 -NextHop '172.20.10.4' -PolicyStore ActiveStore
```

Docker Desktop：`Settings → Resources → Network → Enable host networking`，设置发生变化时应用并重启Docker Desktop。它影响电脑上的容器，不重启Jetson。Docker Desktop从4.34开始支持这个显式启用的TCP/UDP功能，详见[Docker官方说明](https://docs.docker.com/engine/network/drivers/host/)。

WSL需要直接运行ROS查询时，在**WSL的rescue终端**使用：

```bash
sudo ip route replace 192.168.10.33/32 via 172.20.10.4 dev eth3
```

此命令不能放到Jetson的potato终端执行，Jetson没有WSL的eth3。上述两条路由只指定这台观测电脑到小车的一个地址，不改变默认路由或无人机与小车的电台通信。

## 2. WSLg打开窗口

在**WSL的rescue终端**执行；普通车端`./tools/slicer_shell.sh`未转发DISPLAY，`./tools/noetic.sh`又是断网环境，两者均不用于本机实时GUI。

```bash
cd /home/rescue/slicer_ugv_ros1
./tools/rviz_wsl.sh perception
```

`perception` 选择 `small_obstacle.rviz`，Fixed Frame为base_link，同时看过滤前后点云、scan、地图和footprint；不带参数也是这个模式。要看导航全局/局部路径，关闭该窗口后执行 `./tools/rviz_wsl.sh navigation`，选择Fixed Frame为odom的 `odom_navigation.rviz`。

脚本完成原长命令中的Docker环境设置，并在容器里执行 `rviz -d /rviz/所选配置.rviz`。本机无需 `rospack find`，因为本机源码目录已经只读挂载到容器的 `/rviz`。它不映射串口或运动设备，不启动driver，不release或发目标。关闭窗口即可退出观察，再按需打开另一个模式。

正常应能看到scan、odom、两级costmap，以及活动目标的全局路径、DWA局部轨迹、footprint和目标箭头。只出现“waiting for master”说明容器到master仍不通；有话题列表但没有画面时继续查实际节点端口和车端回连，不靠改变车端ROS地址修补。必要时针对实际失败端口检查Windows防火墙，不全局关闭防火墙或映射大段端口。

2026-10-25首次RViz容器未接通，已保留这个失败事实；用户完成Windows路由和Docker设置后，实时连接验证通过：RViz登记为`http://172.20.10.7:40283/`，scan、odom、TF和两张costmap均有持续实际数据，规划话题已连接，Jetson也能查询该RViz实际XMLRPC端口。当前没有活动目标时，current_goal及plan没有新消息是正常的。验证记录为`.test-results/rviz-wsl-20261025/live-check.json`；端口是本次运行动态值，不作为固定配置。

### 黑屏排查记录

Qt5实际读取`QT_XCB_NO_MITSHM`，旧命令的`QT_X11_NO_MITSHM`没有起到预期作用；已修正，但用户实测仍黑。普通Qt文字探针的内部绘制图有文字，实际窗口抓图黑；原生WSL的Qt窗口也长期未收到exposed事件，用户未看到任务栏里的探针窗口。不能据此继续盲改OpenGL、地图或导航参数。

已关闭黑屏RViz及临时探针；Windows仍残留无响应的RViz显示窗口，正常关闭请求无效，随后只结束了核对标题后的该显示客户端。Ogre能成功初始化llvmpipe／OpenGL3.1，完整日志在`.test-results/rviz-wsl-20261025/rviz-ogre-current.log`。修正Qt变量和清理单个显示客户端均没有完成恢复。

2026-10-26用户确认本机工作已保存后，执行了本机`wsl --shutdown`，随后重开Docker Desktop、Ubuntu-22.04及只读RViz；本轮没有启动Jetson底盘driver、release或发送目标。重启后新窗口得到正常映射，用户确认实际可见，故本轮恢复依赖WSLg显示会话重置，不能声称仅Qt变量就是唯一根因。导航和定位若依附关闭的WSL／SSH终端，其进程可能同时退出；恢复实车观察时应核对当前发布者，不把master仍在线视为整条导航链仍在运行。

若需要重启WSL恢复显示，须先保存电脑上其他WSL／Docker工作；`wsl --shutdown`会停止本机所有WSL发行版及相关Docker容器。它不会重启Jetson，但不能由助手在未确认本机工作已保存时擅自执行。恢复后重开Docker Desktop，再用本页命令核验菜单、面板及地图是否实际可见。

## 3. 退出热点观测

切换为直接电台连接或网络地址变化时，撤销本次主机路由，避免观测电脑继续向旧热点转送：

Windows管理员PowerShell：

```powershell
Remove-NetRoute -DestinationPrefix '192.168.10.33/32' -InterfaceIndex 9 -NextHop '172.20.10.4' -Confirm:$false
```

WSL（若仍有本次路由）：

```bash
sudo ip route del 192.168.10.33/32 via 172.20.10.4 dev eth3
```

这些删除操作也只作用于观测电脑。
