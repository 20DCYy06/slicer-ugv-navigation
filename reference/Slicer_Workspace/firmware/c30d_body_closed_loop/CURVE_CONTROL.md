# 2026-10-25 PWM与连续曲线修正

## 实车证据

直行测试目标0.6m/s、实际编码器约0.16m/s，1.006米用时6.41秒，用户确认地面距离也接近1米。四路回传PWM持续为`[-960,960,960,960]`，达到编译配置60%上限。

该结果证明当前输出已饱和，不能证明电机物理上无法达到0.8m/s。源码显示闭环与实车已验证的开环使用不同电桥调制，因此先统一调制，暂不提高输出上限。当前电机速度/PWM关系仍待实测，不把配置速度当作已达成的地速。

## 改动

1. `main.c:pwm()`：沿用已验证`test_pwm()`的高电平/互补PWM时序；保持闭环各端口原来的电气方向。逐个A/B/C/D比较前进、后退、停止时的八个CCR寄存器，闭环与开环完全相同。60%闭环上限、75%独立开环上限不变。
2. `main.c:wheel_control()`：四轮PI原始输出使用一个比例缩放因子进入允许区间，保留差速比例；饱和时冻结继续推高输出的积分，换向时清除上一方向积分。没有新增停机条件或提高现有故障阈值。
3. `arc_velocity_guard.py`：当底盘为`body_speed`时，取消旧开环起转抬升，不再把弱角速强抬至2.119rad/s，也不再对这种补偿追加扫掠包络校验。保留规划器足迹避障、原线/角加速度平滑、已有禁倒车设置与雷达15cm停车线。兼容旧轮速模式的原补偿分支。
4. 车身前进编码器外环、IMU实际角速度外环及四轮编码器PI继续工作。直行时目标`wz=0`，IMU反馈参与左右轮速修正；曲线时跟踪规划器的非零`wz`。未引入另一套上位机PID。
5. 一米测试额外显示电压和四轮反馈速度，便于实测分析，不改变动作流程。

`navigation.max_forward=0.8m/s`、下位机车身/轮速上限800mm/s均保持；现有线加速度0.2m/s²意味着从静止达到0.8m/s需要至少4秒。短路径、近障或供电/输出能力不足时不保证达到上限。

## 验证

固件原生C测试包括电桥CCR等价、饱和差速保留、积分换向、IMU反馈方向，以及原协议/仲裁/看门狗测试。Docker `$NAV check` 包含`BODY_CONTINUOUS_CURVE_NO_OPEN_LOOP_BOOST_PASS`、`FRONT_LASER_15CM_STOP_PASS`、`NAVIGATION_CHECK_PASS`。

源码、编译、Docker结果及安装记录目录：
`/home/potato/Backups/Slicer/2026-10-25/c30d_curve_pwm_E6qR5O/`。

是否已安装以该目录`current_flash/body_install_journal.json`中`verified`结果为准；静止验收不能替代地面速度和曲线性能测试。

已完成本次刷写，BIN SHA-256 `b8a5b305c4015ced10bd961762da240834870273bdcfca391c550c5931bd12e2`；完整Flash回读 `ece4fa455b510aa73cbf9419cebcabd0b77ee0e8f60485f56a5dee7b13789653`。整片回读、选项字节及IMU验收通过，实际ROS驱动60次全零指令检查通过，四轮PWM全程为0。未执行带电行驶，导航容器及验收驱动均保持停止。

```bash
cd /home/potato/Slicer_Workspace
# 默认只显示计划，--check仅检查通信和IMU，均不发送非零运动
python3 -s tests/test_body_closed_loop_1m.py --check
# 由用户在空旷场地、可急停且其他串口驱动已停止时执行
python3 -s tests/test_body_closed_loop_1m.py --run --speed 0.6
# 前一步确认直行和停车正常后，再测0.8m/s目标
python3 -s tests/test_body_closed_loop_1m.py --run --speed 0.8
```
