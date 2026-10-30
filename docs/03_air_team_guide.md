# 给无人机同学的接口指南

[返回首页](../README.md)

飞机侧最需要理解三件事：目标属于哪个坐标系、车端究竟接受到了哪一步、结果来自哪个任务。当前“跨机传输”“航点存盘”和“我方导航执行”还没有完全接成闭环。

通信来源可从队友的 [potato-ugv-online-navigation-radio（固定提交 643a704）](https://github.com/20DCYy06/potato-ugv-online-navigation-radio/tree/643a704c5fa042b5e9f08d3c37ddad3a38e8b56a) 学习：它保留 FKIE 与 Gmapping + B 样条 + TEB 在线导航。我方已有现场记录的是 odom + GlobalPlanner + DWA，并另有 online 候选。两边共享部分组件，但不能据此视为同版部署，也不要混编同名包或双启动两套导航、定位和底盘控制。

## 1 已有接口

| 接口 | 类型 | 已有行为 |
|---|---|---|
| UAV→UGV `/uav/mission/waypoints` | `std_msgs/String`，内容为JSON | 校验一批路点，默认备份并替换 `slicer_navigation/config/waypoints.yaml`；不发Action |
| UGV→UAV `/ugv/odom` | `nav_msgs/Odometry` | 转发本地/odom，最多5Hz；frame和时间戳原样保留 |
| UGV→UAV `/ugv/chassis_ready` | `std_msgs/Bool` | 变化时转发并latched；不是任务接受或在线心跳 |
| UGV→UAV `/ugv/pose` | `PoseWithCovarianceStamped` | 配置源为旧/amcl_pose；当前主线不保证有数据 |
| UGV→UAV `/ugv/mission/state` | `std_msgs/String` | 配置源为旧patrol；尚不是我方Action结果 |
| UGV本地 `/ugv/move_base` | `MoveBaseAction` | 单目标、反馈、结果和取消 |
| UGV本地 `/ugv/move_base/make_plan` | `nav_msgs/GetPlan` | 只查询路径，不执行目标 |
| UGV本地 `/gnss/fix` | `sensor_msgs/NavSatFix` | GNSS天线点观测；不是已补偿的base_link位姿 |
| UGV本地 `/gnss/diagnostics` | `DiagnosticArray` | 解质量、时效、差分链路等状态 |

现有FKIE白名单没有自动共享GNSS、Action、TF或速度链。表中的“UGV本地”接口不能假设飞机目前直接可见。状态relay只是转发，不做坐标转换；latched的最后值也不表示源仍在线。

源码：[接收器](../reference/Slicer_Workspace/catkin_ws/src/slicer_navigation/scripts/uav_waypoint_receiver.py)、[路点校验](../reference/Slicer_Workspace/catkin_ws/src/slicer_navigation/scripts/navigation_core.py)、[relay配置](../reference/Slicer_Workspace/communication/config/relay.yaml)、[同步白名单](../reference/Slicer_Workspace/communication/config/sync_topics.yaml)。

## 2 航点JSON的实际含义

下面只说明数据形状，示例坐标不是现场目标：

```json
{
  "frame_id": "map",
  "waypoints": [
    {"name": "point_001", "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0}
  ]
}
```

`frame_id`必须非空，`x/y`必须是有限数值，name须批内唯一；z可省略默认0，yaw可省略。可选的wait_s和容差字段只是被保存，不代表下游已执行。每批替换文件，不是追加队列；显式空批次可清空所存路点。

接收器不检查真实TF是否存在，也不自动做ENU或经纬度转换。当前没有强制任务ID、时效、去重或标准任务回执；`[UAV SAVED]`只说明落盘成功。**名字同为map，不代表飞机和小车的物理坐标已对齐。**

还有一处必须先明确的文件边界：队友 [receiver](https://github.com/20DCYy06/potato-ugv-online-navigation-radio/blob/643a704c5fa042b5e9f08d3c37ddad3a38e8b56a/workspace_snapshot/catkin_ws/src/slicer_navigation/scripts/uav_waypoint_receiver.py) 默认写 `slicer_navigation/config/waypoints.yaml`，其在线 [patrol.launch](https://github.com/20DCYy06/potato-ugv-online-navigation-radio/blob/643a704c5fa042b5e9f08d3c37ddad3a38e8b56a/workspace_snapshot/catkin_ws/src/slicer_navigation_without_mapping/launch/patrol.launch) 默认读 `slicer_navigation_without_mapping/config/waypoints.yaml`。这是两个不同文件；收点成功不会自动启动巡航，也不表示在线巡航读到了刚收到的批次。

队友原 [patrol 参数](https://github.com/20DCYy06/potato-ugv-online-navigation-radio/blob/643a704c5fa042b5e9f08d3c37ddad3a38e8b56a/workspace_snapshot/catkin_ws/src/slicer_navigation/config/patrol_params.yaml) 指向 `/move_base`，脚本还依赖原 TEB 的诊断话题。两套仓库当前都没有把这条收点链接到我方 `/ugv/move_base` Action；仅把文件名或 Action 名改成相同仍不足以证明任务闭环成立。实际集成需明确读取路径、启动时机、坐标和结果回传，后面的推进步骤属于待办。

## 3 坐标约定

| 坐标/量 | 本项目含义 |
|---|---|
| base_link | +x前、+y左、+z上；平面yaw逆时针为正 |
| odom | 小车本次局部连续坐标；定位重启可能重置原点 |
| map | 在线候选Gmapping地图；与odom通过map→odom联系 |
| ENU | 公共原点下东、北、上；x东、y北，单位米 |
| 经纬度 | 角度单位度；当前计算器要求已确认的WGS84 |
| 高度 | GNSS工具使用椭球高，不能直接把MSL海拔当同一量 |
| 目标yaw | 车辆目标朝向，单位rad；不是飞机姿态或默认北起航向 |

现有数学工具支持 `p_map = R(yaw) × p_enu + [tx, ty]`。它们只预览已提供的变换，不测量、不验证配准、不发布TF、不发导航目标。

现场 [gnss_site.yaml](../src/ugv_bringup/config/gnss_site.yaml) 中基准、公共原点、ENU→map及天线偏移仍为null。飞机与车各自拿首帧归零不是共同原点；GNSS点是天线点，杆臂目前只记录、不自动补偿。地图或odom重置后，旧配准不能直接沿用。

预览工具省略yaw意味着未给朝向；`goal.py`省略yaw却默认为0。正式接入时须明确最终朝向策略，不能静默把“无要求”变成“朝世界+x”。

## 4 GNSS已做到什么

`cup_gnss`复用 `nmea_navsat_driver 0.6.1` 和 `ntrip_client 1.4.1`，已有NMEA解析、串口单所有者、RTCM写入及质量诊断。默认配置NTRIP关闭；GNSS独立输出，不覆盖/odom、不发布定位TF、未进行持续全局纠偏。

需要正确解读观测：

- NavSatFix的时间戳是主机接收/适配时间，未完成PPS测量时刻授时。
- 协方差为UNKNOWN，零数组不表示零定位误差。
- status=GBAS_FIX不能单独区分q4 fixed与q5 float；应看原始质量和新鲜度。
- `fresh_rtk_usable`结合q4、时效、差分龄期和本节点近期RTCM写入等条件；写入串口成功仍不证明接收机实际采用了差分。
- 历史差分配置涉及CGCS2000口径，当前WGS84预览工具没有完成基准/历元转换。先确认双方实际数据基准，再转换。

入口：[GNSS节点](../src/cup_gnss/scripts/gnss_node.py)、[质量判断](../src/cup_gnss/src/cup_gnss/quality.py)、[ENU预览](../tools/preview_enu_goal.py)、[经纬度预览](../tools/preview_gnss_goal.py)。

## 5 最小空地协作推进方式

先完成一条**单目标**闭环，再扩展批量航点。沿用现有通信和标准Action，补一层薄适配即可，不需要新任务管理框架。

1. 先离线约定目标的frame、单位、公共原点/配准、物理参考点和yaw语义，用几个已知点双向核算。
2. 接收一个带请求编号的目标，给出明确接受/拒绝结果；先只预览坐标和请求make_plan。
3. 将获准目标送给现有Action，向飞机映射同一编号的进行中、成功、失败和取消结果。ready继续只表示底盘就绪。
4. 用假Action验证重复投递、取消、失联和重置后的行为，之后再做现场单目标试验。
5. 单点链成立后再决定是否需要串行多点、等待、替换等批次语义；已有文件字段不自动等于已实现策略。

飞机提供的地理/视觉目标可以独立推进坐标学习与通信联调；现阶段不应直接发布/cmd_vel来绕过车端导航和控制权。
