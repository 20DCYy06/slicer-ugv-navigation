# GNSS → 公共 ENU → map 离线预览

适用 Slicer 独立新工程；只计算坐标，不连接 ROS、设备或网络，不请求路径、不发送目标/速度。结果是 GNSS 天线点，未补偿未知天线到 `base_link` 的杆臂，不能直接声称是车体目标或现场精度验证。

`tools/preview_gnss_goal.py` 复用迁移的 `cup_gnss.enu.to_enu()` 与现有 `preview_enu_goal.py` 的变换和输出。没有新增定位器、全局 EKF、TF 发布者或导航控制链。

## 平时只改两份配置

接收机身份、串口与差分服务在 `src/ugv_bringup/config/slicer_gnss.yaml`；公共原点、ENU↔map 配准与天线安装位置集中在有中文注释的 `src/ugv_bringup/config/gnss_site.yaml`。后者只有四组数据：

| 字段 | 含义 |
| --- | --- |
| `datum` | 接收机输出和公共原点的真实基准，当前计算只支持 WGS84 |
| `origin` | 所有设备共用的 ENU 原点：纬度/经度为度，椭球高为米 |
| `map_from_enu` | `[tx米, ty米, yaw弧度]`，`p_map = R(yaw) * p_enu + [tx, ty]` |
| `antenna_in_base_link_m` | 天线相位中心相对车体原点的 `[前, 左, 上]` 米；仅记录，不自动补偿 |

未知值保留 `null`，不能用零或旧底盘值冒充。确认公共原点和配准后，日常预览只需输入目标经纬度/椭球高：

```bash
python3 tools/preview_gnss_goal.py --latitude 目标纬度 --longitude 目标经度 --altitude-ellipsoid 目标椭球高
```

没有命令行变换时默认读取 `gnss_site.yaml`，也可用 `--site-config PATH` 指定；解析 YAML 使用工程已有的 `python3-yaml`。配置里的安装位置只显示在输出 `antenna_in_base_link_m`，不会被悄悄应用到坐标；`physical_point` 仍为 `gnss_antenna`，`lever_arm_compensation_applied=false`。补偿到车体还需要对应时刻的车体姿态，不在本预览工具中猜测。

## 原有 JSON/命令行调用保持兼容

原 `gnss_enu_origin.json` 保留，显式使用 `--origin-file PATH` 时仍配合命令行变换，不依赖 PyYAML；也不允许与 `--site-config` 混用。显式命令行变换优先于 site 中的 `map_from_enu`。未指定 site、但显式给出变换的旧调用仍使用原 JSON 默认路径。

两份原点模板的出厂字段均为 `null`，会明确拒绝计算，不从首帧或输入目标自动建立原点。合成 JSON 示例仍可用于原来的无第三方依赖离线检查：

```json
{
  "datum": "WGS84",
  "origin": {
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude_ellipsoid_m": 0.0
  },
  "origin_id": "SYNTHETIC_TEST_ONLY"
}
```

上面的赤道零点只是合成示例，没有实测身份，不能当现场配置。`origin_id` 是可选文字备注，不替代原点坐标。

输入与原点都必须实际采用 WGS84：经纬度为度，高程为椭球米，不能用海拔 MSL 代替，也不能缺高程默认填零。旧公共 service 声明 CGCS2000，其与当前接收机输出的一致性尚未核实；本工具不做基准或参考历元转换，不接受将 CGCS2000 改标签当作 WGS84。来源未核实前保留 `datum: null`，先确认接收机/服务基准。

## 合成示例与输出

在临时文件保存上述合成 JSON 后，从新工程根运行：

```bash
python3 -I -S -B tools/preview_gnss_goal.py \
  --origin-file /tmp/synthetic-origin.json \
  --latitude 0 --longitude 0.000008983152841195214 --altitude-ellipsoid 0 \
  --map-from-enu 10 -2 1.5707963267948966
```

这只是手算示例：目标约在原点东侧 1 米，合成变换将其转至 `map` 约 `(10, -1)` 米。示例中的平移/旋转同样没有现场实测身份。工具不会估计或验证真实 `map <- enu` 配准，输出始终保留 `registration.verified=false`。

允许直接提供 `--map-from-enu TX TY YAW`，或同时提供 `--odom-from-enu TX TY YAW --map-from-odom TX TY YAW`；后一种按 ENU→odom→map 顺序应用，两个变换必须来自同一个评估时刻。变换约定为 `p_target = R(yaw) * p_source + translation`，平移单位米，旋转单位弧度。

ENU 的 x/y/z 为东/北/上，输出 `input.east/north/up` 为米；`map_target` 只含平面 x/y。可选 `--target-yaw` 是目标朝向：ENU 中从 East 逆时针计弧度，不代表 UAV 或车辆姿态。省略后 `map_target` 完全不包含 yaw，显式 `--target-yaw 0` 会按变换旋转。`--alignment-source` 可附调用者的配准来源备注。

配置文件缺失、JSON/YAML 不合法、未知基准、缺坐标/椭球高、非法数值或不完整变换都会返回退出码 2，标准输出为空；按错误检查配置或参数后重新运行，原文件不会被修改。

## 离线验证

```bash
python3 -B tools/test_preview_gnss_goal.py -v
python3 -B tools/test_preview_enu_goal.py -v
```

旧 JSON 测试以 `python -I -S` 运行真实 CLI；site 测试以 `python -I` 使用实际 PyYAML，均只创建临时合成配置，并禁止 ROS 导入、网络、设备、外部进程与文件写入。覆盖公共原点、轴向、变换与覆盖关系、未知安装位置、非法配置、yaw 省略/显式零；这些结果不等于实车 GNSS 或 map 配准验收。
