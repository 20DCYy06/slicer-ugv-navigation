"""无ROS、无串口副作用的连续按键和差速控制。"""
import math
from pathlib import Path
import yaml
from slicer_manual import wheel_pwm as continuous_wheel_pwm

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/teleop_params.yaml'


def load_config(path=DEFAULT_CONFIG, control_mode=None):
    c = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if control_mode is not None and control_mode != '':
        c['control_mode'] = control_mode
    mode = c.get('control_mode', 'pwm_reference')
    if mode not in ('pwm_reference', 'wheel_speed', 'body_speed'):
        raise ValueError('Unknown control_mode: ' + str(mode))
    if mode in ('wheel_speed', 'body_speed'):
        for key in ('pwm_signs', 'encoder_signs'):
            signs = c.get(key)
            if not isinstance(signs, list) or len(signs) != 4 or any(type(s) is not int or s not in (-1, 1) for s in signs):
                raise ValueError(mode + ' requires explicit ' + key + ' in A/B/C/D order; test_pwm_signs are not a substitute')
        if type(c.get('max_wheel_mm_s')) is not int or not 1 <= c['max_wheel_mm_s'] <= 600:
            raise ValueError('max_wheel_mm_s must be an integer in 1..600')
        # 0x10 has no host PWM-cap field. Do not silently accept a lower PWM
        # cap that this mode cannot enforce. 600 is only the reference firmware
        # compiled cap; it does not establish the identity of the installed MCU.
        if c.get('ros_driver_pwm_limit_permille') != 600:
            raise ValueError(mode + ' cannot enforce ros_driver_pwm_limit_permille; only the reference value 600 is accepted')
    if mode == 'body_speed':
        c.setdefault('body_feedback_timeout_s', .30)
        c.setdefault('body_imu_timeout_ms', 100)
        age = float(c['body_feedback_timeout_s'])
        if not math.isfinite(age) or not 0 < age <= c['telemetry_timeout_s']:
            raise ValueError('body_feedback_timeout_s must be positive and within telemetry_timeout_s')
        if type(c['body_imu_timeout_ms']) is not int or not 1 <= c['body_imu_timeout_ms'] <= 100:
            raise ValueError('body_imu_timeout_ms must be an integer in 1..100 for the selected firmware')
    for k in ('wheel_diameter', 'track_width', 'turn_gain_permille_per_rad_s',
              'linear_speed', 'test_slew_permille_s',
              'initial_scale', 'min_scale', 'max_scale', 'scale_step'):
        if not math.isfinite(float(c[k])) or c[k] <= 0:
            raise ValueError('Invalid positive setting: ' + k)
    if not 1 <= c['test_pwm_limit_permille'] <= 750:
        raise ValueError('PWM cap must be 1..750 permille')
    if not 1 <= c['ros_driver_pwm_limit_permille'] <= c['test_pwm_limit_permille']:
        raise ValueError('ROS driver PWM cap must be within the firmware PWM cap')
    for k in ('test_duty_permille', 'wasd_spin_pwm_permille'):
        if not 1 <= c[k] <= c['test_pwm_limit_permille']:
            raise ValueError(k + ' exceeds PWM cap')
    if c['wasd_spin_pwm_permille'] > c['ros_driver_pwm_limit_permille']:
        raise ValueError('WASD spin PWM exceeds ROS driver cap')
    for k in ('test_pwm_signs', 'encoder_signs'):
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
    if c['track_width']/2 <= 0 or not math.isfinite(c['test_duty_permille']/c['linear_speed']):
        raise ValueError('Invalid derived PWM mapping settings')
    return c


def wheel_pwm(v, w, c):
    """连续开环PWM参考；保留直行/纯转端点及左右比，不声称实测速度闭环。"""
    return continuous_wheel_pwm(v, w, c)


def wheel_speed_mm_s(v, w, c):
    """SI differential target -> ABCD int16 mm/s; no PWM signs or host PID.

    Scale before multiplication to retain wheel ratios for extreme finite input.
    Saturation uses one factor for all wheels, then Python nearest-even rounding.
    """
    if not all(math.isfinite(x) for x in (v, w)):
        raise ValueError('Finite chassis velocity required')
    magnitude = max(abs(v), abs(w))
    if magnitude == 0:
        return [0]*4
    turn = (w / magnitude) * (c['track_width'] / 2)
    sides = dict(left=v / magnitude - turn, right=v / magnitude + turn)
    peak = max(abs(x) for x in sides.values())
    peak_mm_s = min(c['max_wheel_mm_s'], (magnitude * peak) * 1000)
    wheels = [0]*4
    for name, port in c['wheel_ports'].items():
        wheels[ord(port)-ord('A')] = round(sides[name.split('_')[0]] / peak * peak_mm_s)
    return wheels


def limit_pwm(values, limit):
    """Keep the requested four-wheel ratio while enforcing a lower ROS safety cap."""
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError('Four finite wheel PWM values required')
    if not 1 <= limit <= 750:
        raise ValueError('Invalid ROS PWM limit')
    scale = max(1.0, max(abs(value) for value in values) / limit)
    return [round(value / scale) for value in values]


class Slew:
    def __init__(self, c):
        self.c, self.output = c, [0.]*4

    def step(self, desired, dt):
        if not any(desired):
            self.output = [0.]*4
        elif any(a*b < 0 for a,b in zip(self.output, desired)):
            # 方向切换先让所有轮归零，避免瞬间一侧反转。
            self.output = [0.]*4
        else:
            distance = max(abs(a-b) for a,b in zip(self.output, desired))
            factor = min(1., self.c['test_slew_permille_s']*min(max(dt,0.),.1)/distance) if distance else 1.
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
            # Fixed wheel duty independent of +/- straight-drive scale.
            # v=0 makes both sides counter-rotate around the chassis center.
            w = self.c['wasd_spin_pwm_permille']/self.c['turn_gain_permille_per_rad_s']
            return 0., (1 if self.key == 'a' else -1)*w
        return 0., 0.
