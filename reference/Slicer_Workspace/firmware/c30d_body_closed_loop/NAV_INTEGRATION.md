# 导航车身双闭环接入（2026-10-25）

适用工程：`/home/potato/Slicer_Workspace/`。独立工程 `slicer_ugv_ros1` 未修改。

## 控制链路与配置

```text
TEB → /cmd_vel/nav_raw → 原有Guard → /cmd_vel/nav → 原有Mux → /cmd_vel
    → Slicer_wasd_control/c30d_driver_node.py → 0x16(vx,wz)
    → MCU编码器均速/IMU角速度外环 → 四轮编码器PI → PWM
```

- `config/slicer_config.yaml`：`navigation.max_forward: 0.8`，生成的 TEB 上限也是 0.8m/s。
- `catkin_ws/src/Slicer_wasd_control/config/teleop_params.yaml`：`control_mode: body_speed`、`max_wheel_mm_s: 800`。
- `firmware/c30d_body_closed_loop/config.yaml`：`max_body_mm_s: 800`、`max_wheel_mm_s: 800`；闭环 PWM 上限仍为 600‰，未提高。
- ROS 驱动复用 `firmware/c30d_body_closed_loop/tools/body_link.py`，查询扩展和 IMU 就绪后发送 0x16；0x10 全零总停止与旧 wheel_speed 模式保留。
- 原定位、编码器里程计发布、话题名、避障、Mux 及 WASD 独立入口不变。导航由 Guard 平滑，车身模式不再叠加旧四轮目标斜坡或按旧轮距限制车身速度。

0.8m/s 是目标上限，不能视作已实测地速。此前旧接口受600mm/s轮速上限限制，转向还会等比缩小两侧速度；此外现有线加速度0.2m/s²使0→0.8至少需要4秒。雷达15cm停车线会按前方空间及制动距离继续减速。轮端CPR仍是现场暂定60000；负载、电池、侧滑及60% PWM饱和也会影响实际速度，未通过加大PWM来掩盖误差。

## 新接口直行1米测试

```bash
cd /home/potato/Slicer_Workspace
# 默认只显示计划，完全不打开串口
python3 -s tests/test_body_closed_loop_1m.py
# 仅握手、检查IMU及遥测，无非零运动指令
python3 -s tests/test_body_closed_loop_1m.py --check
# 用户现场执行；先停止导航、WASD及手柄串口驱动，留出前方空间并准备急停
python3 -s tests/test_body_closed_loop_1m.py --run
```

默认目标 `vx=0.6m/s,wz=0`。0x90累计四轮编码器估计距离达到1米后停车；正确处理uint32回绕。0x93显示实际编码器均速、IMU角速度、控制源及PWM参考；`source=3`表示MCU正在执行新车身模式。Ctrl+C/异常/完成均发送总停止。测试不带雷达避障；1米为编码器估计值，尚未承诺实际地面精度。

修改测试目标可使用 `--speed 0.6 --distance 1.0 --timeout 20`。正常导航仍使用原 `$NAV nav ...`、`$NAV patrol`，加载新驱动后启动日志应包含 `body_speed control via 0x16`。

## 验证与回退资料

- 固件原生控制器、仲裁、看门狗与协议测试；控制层现有5项单元测试；距离正负极性与uint32回绕检查。
- Docker `$NAV check` 通过，输出 `NAVIGATION_CHECK_PASS`。旧ROS测试的固定1.25秒自转等待改为按现有角速度/角加速度计算，不改变实车控制逻辑。
- 刷写前源文件、构建日志、Docker检查日志及完整Flash备份：`/home/potato/Backups/Slicer/2026-10-25/c30d_body_nav_800_g8bzSL/`。
- 刷写和静止验收的实际结果见该目录 `current_flash/body_install_journal.json` 与 `post_flash_zero_telemetry.json`；仅这些结果可以证明已安装版本。
- 本次实际烧入BIN SHA-256：`5ed04c2d22b6f151cf65b9741fd4476e95c3916a5739ca7e982fedbb84e4b3a1`，完整Flash回读 `35a61ab71f097acc5a9a69f3697f97390ee40d278a917773eafbc26c88c8b698`。仅sector0改动，选项字节不变。
- 真实串口已验证60次全零车身指令发送、IMU flags=7及四轮PWM全零；测试结束驱动已退出。Motor OFF时固件不会执行运动模式，因此这不是带载闭环性能验证，也没有执行1米运动。
