#!/usr/bin/env python3
"""仅离线生成宏、运行测试并编译新固件；无串口、复位或烧录功能。"""
from pathlib import Path
import hashlib
import json
import math
import subprocess
import sys
import yaml

HERE = Path(__file__).resolve().parent


def generate():
    cfg = yaml.safe_load((HERE / 'config.yaml').read_text(encoding='utf-8'))
    h, w, b, imu = (cfg[n] for n in ('hardware', 'wheel_pi', 'body', 'imu'))
    assert h['part'] == 'STM32F407VET6' and h['baud'] == 115200
    assert h['control_ms'] == 10 and h['watchdog_ms'] == 500
    assert 20 <= h['telemetry_ms'] <= 50
    assert 1 <= h['counts_per_revolution'] <= 65535
    assert 0 < h['wheel_diameter_m'] < 1 and h['track_width_m'] > 0 and h['wheelbase_m'] > 0
    assert 0 < h['max_wheel_mm_s'] <= 800 and 0 < h['max_body_mm_s'] <= 800
    assert 0 < h['max_yaw_mrad_s'] <= 32767
    assert 1 <= h['pwm_limit_permille'] <= 800
    assert 1 <= h['pwm_test_limit_permille'] <= 750  # legacy direct-PWM interface unchanged
    assert imu['address_7bit'] in (0x68, 0x69) and imu['yaw_axis'] in (0, 1, 2)
    assert imu['gyro_fs_dps'] == 500 and imu['accel_fs_g'] == 2
    assert imu['startup_bias_samples'] > 0 and imu['filter_tau_s'] >= 0
    assert h['wheel_side'].count(1) == h['wheel_side'].count(-1) == 2
    for signs in (h['pwm_signs'], h['encoder_signs'], imu['gyro_signs'], imu['accel_signs']):
        assert len(signs) in (3, 4) and all(x in (-1, 1) for x in signs)
    assert len(h['pwm_signs']) == len(h['encoder_signs']) == 4
    assert len(imu['gyro_signs']) == len(imu['accel_signs']) == 3
    for section in (w, b):
        assert all(math.isfinite(v) and v >= 0 for v in section.values())
    assert b['linear_compensation_max_m_s'] > 0 and b['yaw_compensation_max_m_s'] > 0
    assert h['control_ms'] <= h['stall_timeout_ms'] <= 60000
    assert h['control_ms'] <= h['direction_timeout_ms'] <= 60000
    encoder_limit = math.ceil(h['encoder_max_speed_m_s'] * h['counts_per_revolution'] *
                             h['control_ms'] / 1000 / (math.pi * h['wheel_diameter_m']))
    assert 0 < encoder_limit * 3 < 32767
    ints = {
        'BAUD': h['baud'], 'COUNTS_REV': h['counts_per_revolution'],
        'CONTROL_MS': h['control_ms'], 'TELEMETRY_MS': h['telemetry_ms'],
        'WATCHDOG_MS': h['watchdog_ms'], 'MIN_BATTERY_MV': h['min_battery_mv'],
        'MAX_MM_S': h['max_wheel_mm_s'], 'MAX_BODY_MM_S': h['max_body_mm_s'],
        'MAX_YAW_MRAD_S': h['max_yaw_mrad_s'],
        'CAL_PERMILLE': h['calibration_permille'], 'CAL_MS': h['calibration_ms'],
        'PWM_PERIOD': 1600, 'PWM_CAP': 1600*h['pwm_limit_permille']//1000,
        'TEST_PWM_CAP': 1600*h['pwm_test_limit_permille']//1000,
        'TEST_MAX_MS': 0, 'ENCODER_DELTA_LIMIT': encoder_limit,
        'STALL_TIMEOUT_MS': h['stall_timeout_ms'], 'DIRECTION_TIMEOUT_MS': h['direction_timeout_ms'],
        'IMU_ADDRESS': imu['address_7bit'], 'IMU_YAW_AXIS': imu['yaw_axis'],
        'IMU_BIAS_SAMPLES': imu['startup_bias_samples'],
        'IMU_BIAS_WHEEL_MM_S': imu['bias_stationary_wheel_mm_s'],
        'IMU_TIMEOUT_MS': imu['sample_timeout_ms'],
    }
    floats = {
        'KP': w['kp'], 'KI': w['ki'], 'FEEDFORWARD': w['feedforward'],
        'MM_PER_COUNT': math.pi*h['wheel_diameter_m']*1000/h['counts_per_revolution'],
        'TRACK_WIDTH_M': h['track_width_m'], 'WHEELBASE_M': h['wheelbase_m'],
        'BODY_V_KP': b['linear_kp'], 'BODY_V_KI': b['linear_ki'], 'BODY_V_KD': b['linear_kd'],
        'BODY_V_CAP': b['linear_compensation_max_m_s'],
        'BODY_W_KP': b['yaw_kp'], 'BODY_W_KI': b['yaw_ki'], 'BODY_W_KD': b['yaw_kd'],
        'BODY_W_CAP': b['yaw_compensation_max_m_s'],
        'IMU_RAD_PER_COUNT': math.pi/180/65.5,
        'IMU_FILTER_TAU_S': imu['filter_tau_s'],
        'IMU_BIAS_MAX_RAD_S': imu['bias_stationary_gyro_rad_s'],
    }
    arrays = {'WHEEL_SIDE_VALUES': h['wheel_side'], 'DEFAULT_PWM_SIGNS': h['pwm_signs'],
              'DEFAULT_ENCODER_SIGNS': h['encoder_signs'],
              'IMU_GYRO_SIGNS': imu['gyro_signs'], 'IMU_ACCEL_SIGNS': imu['accel_signs']}
    text = '#ifndef BODY_GENERATED_H\n#define BODY_GENERATED_H\n'
    text += ''.join('#define %s %du\n' % x for x in ints.items())
    text += ''.join('#define %s %.12ff\n' % x for x in floats.items())
    text += ''.join('#define %s {%s}\n' % (k, ','.join(map(str, v))) for k, v in arrays.items())
    text += '#endif\n'
    out = HERE/'build'
    out.mkdir(exist_ok=True)
    (out/'generated.h').write_text(text, encoding='utf-8')
    return cfg, out


def main():
    cfg, out = generate()
    modules = [HERE/'BALANCE/balance.c', HERE/'HARDWARE/ICM20948/ICM20948.c', HERE/'HARDWARE/usartx.c']
    includes = ['-I'+str(out), '-I'+str(HERE), '-I'+str(HERE/'BALANCE'),
                '-I'+str(HERE/'HARDWARE/ICM20948')]
    common = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-misleading-indentation']
    native = out/'test_native'
    subprocess.run(['gcc', '-O2', *common, '-DSLC2_NATIVE_TEST',
                    '-I'+str(HERE/'tests'), *includes,
                    str(HERE/'tests/test_firmware.c'), *map(str, modules), '-o', str(native)], check=True)
    subprocess.run([str(native)], check=True)
    subprocess.run([sys.executable, str(HERE/'tests/test_protocol.py')], check=True)
    elf, binary = out/'c30d_body.elf', out/'c30d_body.bin'
    subprocess.run(['arm-none-eabi-gcc', '-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16',
                    '-mfloat-abi=hard', '-DSTM32F40_41xxx', '-DARM_MATH_CM4', '-Os', '-g3',
                    *common, '-ffreestanding', '-fno-builtin', '-ffunction-sections', '-fdata-sections',
                    '-I'+str(HERE/'vendor_headers'), *includes, '-nostdlib',
                    str(HERE/'startup.S'), str(HERE/'main.c'), str(HERE/'runtime.c'), *map(str, modules),
                    '-T'+str(HERE/'link.ld'), '-Wl,--gc-sections,-Map='+str(out/'c30d_body.map'),
                    '-lgcc', '-o', str(elf)], check=True)
    subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', str(elf), str(binary)], check=True)
    subprocess.run(['arm-none-eabi-objcopy', '-O', 'ihex', str(elf), str(out/'c30d_body.hex')], check=True)
    subprocess.run(['arm-none-eabi-size', str(elf)], check=True)
    data = binary.read_bytes()
    assert len(data) <= 16384, 'New image exceeds existing sector-0 deployment capacity'
    manifest = {'part': cfg['hardware']['part'], 'flash_base': '0x08000000',
                'protocol': 'SLC2', 'extension': 'body-imu-v1',
                'size': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                'sectors_changed': [0], 'configuration': cfg,
                'build_only': True, 'flashed': False}
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps(manifest, indent=2), flush=True)


if __name__ == '__main__':
    main()
