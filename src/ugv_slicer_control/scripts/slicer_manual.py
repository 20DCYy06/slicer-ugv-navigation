# SPDX-License-Identifier: MIT
"""Small Slicer control adapter, copied verbatim into its derived build input.

No ROS, device, filesystem or clock side effects. The existing Slicer mux owns
arbitration; this policy gives that mux explicit generation-bound manual hold.
"""
from collections import OrderedDict
import json
import math
import uuid

INTENT_TOPIC = '/cup_nav/manual_intent'
STATE_TOPIC = '/cup_nav/manual_state'
MUX_OWNER = '/slicer_velocity_mux'
RELEASE_OWNER = '/cup_nav_executor'


def _identifier(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ValueError(name + ' must be a nonempty string of at most 128 characters')
    return value


def _generation(value):
    if type(value) is not int or not 0 <= value < 2**63:
        raise ValueError('invalid takeover_generation')
    return value


def _object(data):
    if isinstance(data, str):
        if len(data) > 4096:
            raise ValueError('manual control message too large')
        try:
            data = json.loads(data)
        except (ValueError, TypeError) as exc:
            raise ValueError('invalid manual control JSON') from exc
    if not isinstance(data, dict) or type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise ValueError('manual control schema_version must be 1')
    return dict(data)


def parse_manual_state(data):
    value = _object(data)
    _identifier(value.get('mux_instance_id'), 'mux_instance_id')
    _generation(value.get('takeover_generation'))
    for key in ('manual_hold', 'teleop_fresh', 'emergency_stop_latched', 'last_intent_accepted'):
        if type(value.get(key)) is not bool:
            raise ValueError('invalid manual state ' + key)
    if value.get('selected_source') not in ('zero', 'teleop', 'mission', 'nav'):
        raise ValueError('invalid manual selected_source')
    if value.get('hold_reason') not in ('startup', 'manual_takeover', 'emergency_stop', 'released'):
        raise ValueError('invalid manual hold_reason')
    for key in ('last_intent_id', 'last_intent_action', 'last_intent_reason'):
        if not isinstance(value.get(key), str) or len(value[key]) > 512:
            raise ValueError('invalid manual state ' + key)
    if value['last_intent_action'] not in ('', 'takeover', 'release'):
        raise ValueError('invalid manual acknowledgement action')
    return value


def takeover_intent(intent_id=None):
    return dict(schema_version=1, intent_id=_identifier(
        uuid.uuid4().hex if intent_id is None else intent_id, 'intent_id'), action='takeover')


def release_intent(state, intent_id=None):
    value = parse_manual_state(state)
    return dict(schema_version=1, intent_id=_identifier(
        uuid.uuid4().hex if intent_id is None else intent_id, 'intent_id'), action='release',
        mux_instance_id=value['mux_instance_id'], takeover_generation=value['takeover_generation'])


def wheel_pwm(v, w, c):
    """Continuous ratio-preserving open-loop reference, not physical calibration.

    The unsaturated maximum is |v|*straight_gain+|w|*turn_gain. Normalize
    geometry separately so even finite extreme commands never form inf/inf.
    """
    if not all(math.isfinite(x) for x in (v, w)):
        raise ValueError('Nonfinite velocity')
    half_track = c['track_width'] / 2
    linear_gain = c['test_duty_permille'] / c['linear_speed']
    if not math.isfinite(half_track) or half_track <= 0 or not math.isfinite(linear_gain) or linear_gain <= 0:
        raise ValueError('Invalid derived PWM mapping settings')
    largest = max(abs(v), abs(w))
    if largest == 0:
        return [0.] * 4
    cap = c['test_pwm_limit_permille']
    magnitude = min(cap, min(cap, abs(v)*linear_gain) +
                    min(cap, abs(w)*c['turn_gain_permille_per_rad_s']))
    forward, turn = v/largest, (w/largest)*half_track
    denominator = abs(forward) + abs(turn)
    pair = [(forward-turn)/denominator*magnitude,
            (forward+turn)/denominator*magnitude]
    values = {port: pair[0 if wheel.startswith('left_') else 1]
              for wheel, port in c['wheel_ports'].items()}
    return [values[p]*sign for p, sign in zip('ABCD', c['test_pwm_signs'])]


class ManualMux:
    """Policy embedded in the existing mux; callers serialize access with its lock."""
    def __init__(self, timeout, instance_id=None, release_owner=RELEASE_OWNER):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('finite positive command timeout required')
        self.timeout = timeout
        self.instance_id = _identifier(uuid.uuid4().hex if instance_id is None else instance_id, 'mux_instance_id')
        self.release_owner = _identifier(release_owner, 'release_owner')
        self.generation = 0
        self.held = True
        self.reason = 'startup'
        self.estop = False
        self.slots = {}
        self.seen = OrderedDict()
        self.ack = dict(last_intent_id='', last_intent_action='',
            last_intent_accepted=False, last_intent_reason='awaiting explicit initial release')

    def receive(self, name, velocity, now):
        if name not in ('teleop', 'mission', 'nav'):
            raise ValueError('unknown velocity input')
        if name == 'teleop' and not self.held:
            return
        values = tuple(velocity)
        if len(values) != 2 or not all(math.isfinite(v) for v in values):
            values = (0., 0.)
        self.slots[name] = (now, values)

    def _fresh(self, name, now):
        return name in self.slots and 0 <= now-self.slots[name][0] < self.timeout

    def select(self, now):
        if self.estop:
            return 'zero', (0., 0.)
        for name in (('teleop',) if self.held else ('mission', 'nav')):
            if self._fresh(name, now):
                return name, self.slots[name][1]
        return 'zero', (0., 0.)

    def emergency(self, active):
        if active and not self.estop:
            self.estop = True
            self.held = True
            self.reason = 'emergency_stop'
            self.generation += 1
            self.slots.clear()

    def state(self, now):
        return dict(schema_version=1, mux_instance_id=self.instance_id,
            takeover_generation=self.generation, manual_hold=self.held,
            hold_reason=self.reason, teleop_fresh=self._fresh('teleop', now),
            emergency_stop_latched=self.estop, selected_source=self.select(now)[0], **self.ack)

    def intent(self, data, caller_id, latched, now):
        intent_id = action = ''
        fingerprint = None
        try:
            value = _object(data)
            intent_id = _identifier(value.get('intent_id'), 'intent_id')
            action = value.get('action')
            if action not in ('takeover', 'release'):
                action = ''
                raise ValueError('unknown manual intent action')
            expected = {'schema_version', 'intent_id', 'action'}
            if action == 'release':
                expected |= {'mux_instance_id', 'takeover_generation'}
                _identifier(value.get('mux_instance_id'), 'mux_instance_id')
                _generation(value.get('takeover_generation'))
            if set(value) != expected:
                raise ValueError('unexpected manual intent fields')
            if latched:
                raise ValueError('latched manual intents are refused')
            if action == 'release' and caller_id != self.release_owner:
                raise ValueError('release requires bound executor owner')
            fingerprint = (caller_id, json.dumps(value, sort_keys=True, separators=(',', ':')))
            if intent_id in self.seen:
                previous, accepted, reason = self.seen[intent_id]
                if fingerprint != previous:
                    raise ValueError('manual intent ID reused with different content/owner')
            else:
                if action == 'takeover':
                    self.generation += 1
                    self.held = True
                    self.reason = 'emergency_stop' if self.estop else 'manual_takeover'
                    self.slots.clear()
                    accepted, reason = True, 'manual ownership held; explicit executor release required'
                else:
                    if (value['mux_instance_id'], value['takeover_generation']) != (self.instance_id, self.generation):
                        raise ValueError('mux instance or takeover generation changed')
                    if self.estop:
                        raise ValueError('emergency stop latched; release refused')
                    if self._fresh('teleop', now):
                        raise ValueError('teleop packet activity remains fresh')
                    self.held = False
                    self.reason = 'released'
                    self.slots.clear()
                    accepted, reason = True, 'explicit executor release accepted; awaiting new velocity input'
                self.seen[intent_id] = (fingerprint, accepted, reason)
                if len(self.seen) > 128:
                    self.seen.popitem(last=False)
            self.ack = dict(last_intent_id=intent_id, last_intent_action=action,
                last_intent_accepted=accepted, last_intent_reason=reason)
        except (ValueError, TypeError, OverflowError) as exc:
            # A transport retry is the same event, never fresh permission after
            # the reason for a rejected release disappears (e.g. key timeout).
            if fingerprint is not None and intent_id not in self.seen:
                self.seen[intent_id] = (fingerprint, False, str(exc)[:512])
                if len(self.seen) > 128:
                    self.seen.popitem(last=False)
            self.ack = dict(last_intent_id=intent_id, last_intent_action=action,
                last_intent_accepted=False, last_intent_reason=str(exc)[:512])
        return self.state(now)


def request_takeover(timeout=1.5, intent_topic=INTENT_TOPIC, state_topic=STATE_TOPIC, mux_owner=MUX_OWNER):
    """Field CLI entry: positive takeover plus bounded owned-state ACK, no velocity.

    ROS imports and node initialization happen only when explicitly called.
    """
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('finite positive takeover timeout required')
    import threading
    import time
    import rospy
    from std_msgs.msg import String
    if not rospy.core.is_initialized():
        rospy.init_node('cup_nav_manual_takeover', anonymous=True, disable_signals=True)
    request = takeover_intent()
    done = threading.Event()
    response = []
    def receive(msg):
        header = getattr(msg, '_connection_header', {}) or {}
        if header.get('callerid') != mux_owner or header.get('latching', '0') != '0':
            return
        try:
            value = parse_manual_state(msg.data)
        except ValueError:
            return
        if value['last_intent_id'] == request['intent_id']:
            response[:] = [value]
            done.set()
    subscriber = rospy.Subscriber(state_topic, String, receive, queue_size=10)
    publisher = rospy.Publisher(intent_topic, String, queue_size=10, latch=False)
    try:
        deadline = time.monotonic()+timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            publisher.publish(String(data=json.dumps(request, sort_keys=True)))
            if done.wait(min(.1, max(0., deadline-time.monotonic()))):
                value = response[0]
                accepted = value['last_intent_accepted'] and value['manual_hold']
                return dict(success=accepted, message=value['last_intent_reason'], manual_state=value)
        return dict(success=False, message='manual takeover ACK timed out; inspect mux owner/state and retry', manual_state=None)
    finally:
        publisher.unregister()
        subscriber.unregister()
