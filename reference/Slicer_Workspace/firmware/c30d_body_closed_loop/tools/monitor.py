#!/usr/bin/env python3
"""默认只看遥测；只有显式 --command VX WZ 才发送非零车身控制指令。"""
import argparse
import json
import time
from body_link import BodyChassis, load_config


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration', type=float, default=15.)
    p.add_argument('--command', nargs=2, type=float, metavar=('VX_M_S', 'WZ_RAD_S'))
    args = p.parse_args()
    car = BodyChassis(load_config())
    try:
        print('EXTENSION_INFO', car.extension_info(), flush=True)
        if args.command:
            car.configure(car.cfg['encoder_signs'])
            car.wait(lambda: car.body_telemetry is not None and car.body_telemetry['imu_flags'] & 7 == 7, 5.)
            if car.status['flags'] & 0x34 != 0x30:
                raise RuntimeError('Motor/power/fault state does not allow the requested command')
        started = time.monotonic()
        next_print = started
        while time.monotonic()-started < args.duration:
            if args.command:
                car.body_speed(*args.command)
            car.poll(.04)
            if time.monotonic()-car.last_rx > car.cfg['telemetry_timeout_s']:
                raise RuntimeError('Telemetry timeout')
            if car.status['flags'] & 4:
                raise RuntimeError('MCU fault: %s' % car.read_fault_diagnostic())
            if time.monotonic() >= next_print:
                print(json.dumps(car.body_telemetry, ensure_ascii=False), flush=True)
                next_print = time.monotonic()+.2
            time.sleep(.01)
    finally:
        car.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
