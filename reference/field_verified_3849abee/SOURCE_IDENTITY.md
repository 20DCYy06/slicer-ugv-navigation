# 3849abee 固件来源与收录建议

本页只核对用户指定 ZIP 和已有 2026-10-29 实测交付记录；未执行包内脚本、未连接硬件，也未重新构建。

## 身份和当前使用状态

- 原文件：`/home/rescue/slicer_ugv_ros1/firmware/c30d_yaw_fix_20261029_3849abee.zip`。
- ZIP 内真正候选源码根是 `c30d_yaw_fix_20261029/source/`，不是 `outputs/baseline/`。
- 候选 BIN 为 `outputs/candidate/c30d_body.bin`，10208 字节、SHA256 `3849abee0451ef94a6ac5bfd5bbe508c7ca9da016ae87afed70ee09ea18a17a4`。
- `outputs/baseline/c30d_body.bin` 是 10144 字节 `92e816859d94d94e20077998b4cd4b86bd0c52fdc590d5e21c0b7246048885da`，只是该次对照基线，不应混入当前固件目录。
- 2026-10-29 `deliverables/c30d_workfix_field_20261029/RUN_RESULT.json` 记录3849镜像烧写及完整回读核对、实际正反转及交替方向恢复；精确低速跟随仍未通过，存在脉冲和迟滞。
- 本次主代理从 Jetson 当前 `last_verified_install.json` 读到另一镜像 `0852f4b43c5bfb21b34dec5ee727a7cbb72d4e4db982fe925b0d40a7f60a962a`。因此3849只能标为**此前实测对照**，不能凭旧交付记录称为当前部署版。该安装记录也不等于本轮板卡回读。
- 已提供的远端关键哈希中，当前 `BALANCE/balance.c` 与3849相同；`main.c` 和 `config.yaml` 不同。当前源码须单独原样保存，由主包记录远端来源与哈希，不能用3849覆盖它。

## 最少收录内容

在 `firmware/field_verified_3849abee/` 收录指定 ZIP 的 `source/` 全部24个文件（含配置、链接脚本、启动文件、CMSIS/STM32头和假寄存器测试）；**`source/build/generated.h` 是构建输入身份，不能因build目录通用排除而漏掉**。

协议测试额外依赖 `build_tools/test_support/{c30d_link.py,chassis_core.py,control_core.py,serial_transport.py,provenance.json}`，保留与source相对的原布局。`source/tools/body_link.py` 被纯协议测试导入；不构造真实串口。

建议同时收录小型身份材料：`outputs/candidate/{manifest.json,build_validation.json}`、`outputs/final_identity.json`、`firmware.patch`；完整参数在config.yaml，不另复制一份调参表。已有实测交付 `README.md` 与 `RUN_RESULT.json` 可作来源证据，但文中旧相对bag路径不意味着本包已收bag。

不必带 `outputs/baseline/` 镜像、全套ELF/MAP、逐周期CSV、构建失败中间目录，或可SSH的车端构建包装脚本。可保留候选BIN作为身份参照，但这不是源码构建所需项，也不授权烧写。

## 离线构建依赖（仅静态核对）

`source/build.py`：Python3 + PyYAML，native GCC，`arm-none-eabi-gcc`/objcopy/size；编译 Cortex-M4 hard-float (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`)，`-Os`，STM32F407VET6，生成并写入自己副本的build目录。历史工具链为10.3.1 20210621。若将来重建，仅在独立副本运行：

```bash
cd firmware/field_verified_3849abee/source
PYTHONPATH=../build_tools/test_support python3 build.py
```

该命令在本次收集任务没有执行；不据此新增本轮构建通过声明。

## 哈希

| 材料 | SHA256 |
|---|---|
| `来源 ZIP` | `7835f69c3ea258e87356701fc6ea5e733d0395e749d1cffbb7871ee02fd261b7` |
| `source/main.c` | `2a918cf01b52074400917b2075747565447096029e7402c669e80fc4d13ae45e` |
| `source/BALANCE/balance.c` | `85bf21baf35bf66112acf68e1c448582e99f69496e349d4cc666077ce79d27ab` |
| `source/BALANCE/balance.h` | `e7d5e4de6069f02ab7f93483142d02adc848ad39c3752393ca19db42bc4c8977` |
| `source/config.yaml` | `4372931dc2026112270fcf659cf69b72ca70981809d54e4ca222c2cfdea3c5e4` |
| `source/build/generated.h` | `4c3a7ee1f7447b6a625ad5b182bfad6a0460cd8459211b1d0b03abf2c07a74b3` |
| `source/tests/test_firmware.c` | `e27cfd46ac76c86bba0d31cc8e3ddcd31f3cdd34dacacd5093976e5f16b16ccd` |
| `outputs/candidate/c30d_body.bin` | `3849abee0451ef94a6ac5bfd5bbe508c7ca9da016ae87afed70ee09ea18a17a4` |

## ZIP 中24份源码/构建输入

- `source/build.py`
- `source/config.yaml`
- `source/link.ld`
- `source/main.c`
- `source/runtime.c`
- `source/startup.S`
- `source/BALANCE/balance.c`
- `source/BALANCE/balance.h`
- `source/build/generated.h`
- `source/HARDWARE/usartx.c`
- `source/HARDWARE/usartx.h`
- `source/tests/stm32f4xx.h`
- `source/tests/test_firmware.c`
- `source/tests/test_protocol.py`
- `source/tools/body_link.py`
- `source/vendor_headers/arm_math.h`
- `source/vendor_headers/core_cm4.h`
- `source/vendor_headers/core_cm4_simd.h`
- `source/vendor_headers/core_cmFunc.h`
- `source/vendor_headers/core_cmInstr.h`
- `source/vendor_headers/stm32f4xx.h`
- `source/vendor_headers/system_stm32f4xx.h`
- `source/HARDWARE/ICM20948/ICM20948.c`
- `source/HARDWARE/ICM20948/ICM20948.h`
