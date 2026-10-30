"""无ROS、无串口副作用的连续按键和差速控制。"""
import math
from pathlib import Path
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/teleop_params.yaml'


def load_config(path=DEFAULT_CONFIG):
    c = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    for k in ('wheel_diameter', 'track_width', 'turn_gain_permille_per_rad_s',
              'linear_speed', 'wasd_spin_angular', 'max_wheel_mm_s',
              'test_slew_permille_s', 'wheel_slew_mm_s2',
              'initial_scale', 'min_scale', 'max_scale', 'scale_step'):
        if not math.isfinite(float(c[k])) or c[k] <= 0:
            raise ValueError('Invalid positive setting: ' + k)
    if not 1 <= c['test_pwm_limit_permille'] <= 750:
        raise ValueError('PWM cap must be 1..750 permille')
    if not 1 <= c['ros_driver_pwm_limit_permille'] <= c['test_pwm_limit_permille']:
        raise ValueError('ROS driver PWM cap must be within the firmware PWM cap')
    for k in ('test_duty_permille',):
        if not 1 <= c[k] <= c['test_pwm_limit_permille']:
            raise ValueError(k + ' exceeds PWM cap')
    for k in ('test_pwm_signs', 'encoder_signs', 'pwm_signs'):
        if len(c[k]) != 4 or any(s not in (-1, 1) for s in c[k]):
            raise ValueError(k + ' requires four +/-1 signs in A/B/C/D order')
    if set(c['wheel_ports']) != {'left_front','right_front','left_rear','right_rear'} or set(c['wheel_ports'].values()) != set('ABCD'):
        raise ValueError('Wheel mapping must cover A/B/C/D exactly once')
    if not .2 <= c['key_timeout_s'] <= .5 or not .1 <= c['command_timeout_s'] <= .5:
        raise ValueError('Invalid command/key watchdog timeout')
    if not .1 <= c['telemetry_timeout_s'] <= .5 or not 10 <= c['publish_hz'] <= 50:
        raise ValueError('Invalid telemetry timeout/rate')
    if not c['min_scale'] <= c['initial_scale'] <= c['max_scale']:
        raise ValueError('Invalid speed scale range')
    return c


def wheel_pwm(v, w, c):
    """Twist到PWM参考映射，保持左右速度比；不声称是实测速率闭环。"""
    if not all(math.isfinite(a) for a in (v, w)):
        raise ValueError('Nonfinite velocity')
    left, right = v-w*c['track_width']/2, v+w*c['track_width']/2
    if abs(w) > 1e-9:
        gain = c['turn_gain_permille_per_rad_s']/(c['track_width']/2)
    else:
        gain = c['test_duty_permille']/c['linear_speed']
    pair = [left*gain, right*gain]
    scale = max(1., max(map(abs, pair))/c['test_pwm_limit_permille'])
    values = {port: pair[0 if wheel.startswith('left_') else 1]/scale
              for wheel, port in c['wheel_ports'].items()}
    return [values[p]*s for p,s in zip('ABCD', c['test_pwm_signs'])]


def wheel_speeds_mm_s(v, w, c):
    """差速车体速度 -> A/B/C/D 轮缘目标速度；符号由 MCU 闭环配置处理。"""
    if not all(math.isfinite(a) for a in (v, w)):
        raise ValueError('Nonfinite velocity')
    left, right = v-w*c['track_width']/2, v+w*c['track_width']/2
    values = {port: left if wheel.startswith('left_') else right
              for wheel, port in c['wheel_ports'].items()}
    scale = max(1., max(abs(a) for a in values.values()) * 1000 / c['max_wheel_mm_s'])
    return [round(values[p] * 1000 / scale) for p in 'ABCD']


def mix_pwm(throttle, steering, c, forward_pwm, turn_pwm):
    """开环差速：前进分量和转向分量独立叠加，再统一缩放限幅。

    throttle/steering 为 [-1,1]；正转向为左转。0x11 电气极性使用
    已实测的 test_pwm_signs，不使用 0x10 闭环的 pwm_signs。
    """
    base = throttle * forward_pwm
    turn = steering * turn_pwm
    left, right = base - turn, base + turn
    values = {port: left if wheel.startswith('left_') else right
              for wheel, port in c['wheel_ports'].items()}
    return limit_pwm([values[p] * sign for p, sign in zip('ABCD', c['test_pwm_signs'])],
                     c['test_pwm_limit_permille'])


def limit_pwm(values, limit):
    """Keep the requested four-wheel ratio while enforcing a lower ROS safety cap."""
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError('Four finite wheel PWM values required')
    if not 1 <= limit <= 750:
        raise ValueError('Invalid ROS PWM limit')
    scale = max(1.0, max(abs(value) for value in values) / limit)
    return [round(value / scale) for value in values]


class Slew:
    def __init__(self, c, rate_key='test_slew_permille_s'):
        self.c, self.output, self.rate = c, [0.]*4, c[rate_key]

    def step(self, desired, dt):
        if not any(desired):
            self.output = [0.]*4
        elif any(a*b < 0 for a,b in zip(self.output, desired)):
            # 方向切换先让所有轮归零，避免瞬间一侧反转。
            self.output = [0.]*4
        else:
            distance = max(abs(a-b) for a,b in zip(self.output, desired))
            factor = min(1., self.rate*min(max(dt,0.),.1)/distance) if distance else 1.
            self.output = [a+(b-a)*factor for a,b in zip(self.output, desired)]
        return [round(x) for x in self.output]


class Keys:
    def __init__(self, c):
        self.c, self.key, self.last = c, '', -math.inf
        self.scale = c['initial_scale']

    def feed(self, data, now):
        if b'\x1b' in data:
            self.key = ''
            return True
        for char in data.decode('ascii', errors='ignore').lower():
            if char in ('q','\x03','\x04'):
                self.key = ''
                return False
            if char in 'wasd':
                self.key, self.last = char, now
            elif char in '+=-':
                self.scale = max(self.c['min_scale'], min(self.c['max_scale'],
                    self.scale + (-1 if char == '-' else 1)*self.c['scale_step']))
            else:
                self.key = ''
        return True

    def velocity(self, now):
        if now-self.last >= self.c['key_timeout_s']:
            self.key = ''
        if self.key in ('w','s'):
            return (1 if self.key == 'w' else -1)*self.c['linear_speed']*self.scale, 0.
        if self.key in ('a','d'):
            # Pure spin setpoint independent of +/- straight-drive scale.
            # v=0 makes both sides counter-rotate around the chassis center.
            w = self.c['wasd_spin_angular']
            return 0., (1 if self.key == 'a' else -1)*w
        return 0., 0.
