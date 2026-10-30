#!/usr/bin/env python3
"""Reject untrusted state and releases that could resume existing goals."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import contextlib
import io

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'


class ControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(SCRIPTS))
        spec = importlib.util.spec_from_file_location('control', SCRIPTS / 'control.py')
        cls.client = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.client)
        import slicer_manual
        cls.policy = slicer_manual

    def setUp(self):
        self.mux = self.policy.ManualMux(.25, instance_id='instance', release_owner='/ugv/control')
        self.before = self.mux.state(10.)

    def test_startup_and_explicit_new_owner(self):
        self.assertTrue(self.before['manual_hold'])
        request = self.policy.release_intent(self.before, 'release')
        wrong = self.mux.intent(request, '/cup_nav_executor', False, 10.)
        self.assertFalse(wrong['last_intent_accepted'])
        result = self.mux.intent(request, '/ugv/control', False, 10.)
        self.assertTrue(result['last_intent_accepted'])
        self.assertFalse(result['manual_hold'])

    def test_latched_generation_and_emergency_are_refused(self):
        request = self.policy.release_intent(self.before, 'release')
        self.assertFalse(self.mux.intent(request, '/ugv/control', True, 10.)['last_intent_accepted'])
        self.mux.intent(self.policy.takeover_intent('takeover'), '/operator', False, 10.)
        self.assertFalse(self.mux.intent(request, '/ugv/control', False, 10.)['last_intent_accepted'])
        self.mux.emergency(True)
        self.mux.emergency(False)
        request = self.policy.release_intent(self.mux.state(11.), 'after-estop')
        self.assertFalse(self.mux.intent(request, '/ugv/control', False, 11.)['last_intent_accepted'])

    def test_state_requires_mux_transport_owner_and_nonlatched_message(self):
        message = SimpleNamespace(data=json.dumps(self.before),
                                  _connection_header={'callerid': '/slicer_velocity_mux', 'latching': '0'})
        self.assertTrue(self.client.read_state(message, '/slicer_velocity_mux')['manual_hold'])
        for header in ({'callerid': '/other', 'latching': '0'},
                       {'callerid': '/slicer_velocity_mux', 'latching': '1'}):
            message._connection_header = header
            with self.assertRaises(ValueError):
                self.client.read_state(message, '/slicer_velocity_mux')

    def test_ack_requires_id_instance_generation_action_and_postcondition(self):
        request = self.policy.release_intent(self.before, 'release')
        result = self.mux.intent(request, '/ugv/control', False, 10.)
        self.assertTrue(self.client.acknowledged(result, request, self.before, False))
        for key, value in [('last_intent_id', 'other'), ('mux_instance_id', 'restart'),
                           ('takeover_generation', 1), ('last_intent_action', 'takeover'),
                           ('last_intent_accepted', False), ('manual_hold', True)]:
            changed = dict(result, **{key: value})
            self.assertFalse(self.client.acknowledged(changed, request, self.before, False), key)
        request = self.policy.takeover_intent('hold')
        result = self.mux.intent(request, '/ugv/control', False, 11.)
        self.assertTrue(self.client.acknowledged(result, request, self.before, True))

    def test_busy_or_missing_action_status_refuses_release(self):
        self.assertTrue(self.client.release_allowed([]))
        for status in (0, 1, 6, 7):
            self.assertFalse(self.client.release_allowed([SimpleNamespace(status=status)]))
        for status in (2, 3, 4, 5, 8, 9):
            self.assertTrue(self.client.release_allowed([SimpleNamespace(status=status)]))
        self.assertFalse(self.client.release_allowed(None))

    def test_unique_publisher_and_receive_age_are_required(self):
        self.assertTrue(self.client.fresh_owned((10., self.before), 10.2, .5,
                                               ['/slicer_velocity_mux'], '/slicer_velocity_mux'))
        for publishers, now in [([], 10.2), (['/other'], 10.2),
                                (['/slicer_velocity_mux', '/other'], 10.2),
                                (['/slicer_velocity_mux'], 11.)]:
            self.assertFalse(self.client.fresh_owned((10., self.before), now, .5,
                                                    publishers, '/slicer_velocity_mux'))

    def run_boundary_client(self, command, delayed_state=False, stale_status=False,
                            cancel_connected=False, ownership_change=None,
                            intent_subscribers=None, fresh_status=False):
        """Replace only ROS transport/clock; exercise main and the real mux policy."""
        clock, callbacks, calls = [10.], {}, [0]
        self.mux.intent(self.policy.release_intent(self.before, 'initial'), '/ugv/control', False, 10.)

        def state_message():
            return SimpleNamespace(data=json.dumps(self.mux.state(clock[0])),
                                   _connection_header={'callerid': '/slicer_velocity_mux', 'latching': '0'})

        class Master:
            def __init__(self, name):
                pass

            def getPid(self):
                return 1

            def getSystemState(inner):
                calls[0] += 1
                if ownership_change and calls[0] == 2:
                    if ownership_change == 'takeover':
                        self.mux.intent(self.policy.takeover_intent('operator'), '/operator', False, clock[0])
                    else:
                        self.mux = self.policy.ManualMux(.25, instance_id='restarted', release_owner='/ugv/control')
                if delayed_state:
                    clock[0] += 1.
                elif stale_status and calls[0] == 1:
                    clock[0] += .2
                if not delayed_state and '/cup_nav/manual_state' in callbacks:
                    callbacks['/cup_nav/manual_state'](state_message())
                if (ownership_change or fresh_status) and '/ugv/move_base/status' in callbacks:
                    callbacks['/ugv/move_base/status'](SimpleNamespace(
                        status_list=[], header=SimpleNamespace(stamp=Stamp(clock[0])),
                        _connection_header={'callerid': '/ugv/move_base'}))
                return ([('/cup_nav/manual_state', ['/slicer_velocity_mux']),
                         ('/ugv/move_base/status', ['/ugv/move_base'])],
                        [('/cup_nav/manual_intent', ['/slicer_velocity_mux']
                          if intent_subscribers is None else intent_subscribers)], [])

        class Publisher:
            def __init__(self, topic, *args, **kwargs):
                self.topic = topic

            def get_num_connections(self):
                return int(cancel_connected) if self.topic.endswith('/cancel') else 1

            def publish(inner, message):
                if inner.topic == '/cup_nav/manual_intent':
                    self.mux.intent(message.data, '/ugv/control', False, clock[0])
                    callbacks['/cup_nav/manual_state'](state_message())

        def subscribe(topic, message_type, callback, **kwargs):
            callbacks[topic] = callback
            if topic == '/cup_nav/manual_state':
                callback(state_message())
            if stale_status and topic.endswith('/status'):
                callback(SimpleNamespace(status_list=[], header=SimpleNamespace(stamp=Stamp(9.6)),
                                         _connection_header={'callerid': '/ugv/move_base'}))
            return object()

        class Stamp:
            def __init__(self, value):
                self.value = value

            def __sub__(self, other):
                return SimpleNamespace(to_sec=lambda: self.value - other.value)

        rospy = SimpleNamespace(myargv=lambda: ['control.py', command, '--timeout', '.1'],
                                init_node=lambda *a, **kw: None, get_name=lambda: '/ugv/control',
                                get_param=lambda key, default: default, resolve_name=lambda name: name,
                                Subscriber=subscribe, Publisher=Publisher, is_shutdown=lambda: False,
                                Time=SimpleNamespace(now=lambda: Stamp(clock[0])))
        modules = {'rospy': rospy, 'rosgraph': SimpleNamespace(Master=Master, myargv=rospy.myargv),
                   'actionlib_msgs.msg': SimpleNamespace(GoalID=SimpleNamespace, GoalStatusArray=object),
                   'std_msgs.msg': SimpleNamespace(String=SimpleNamespace)}
        with patch.dict(sys.modules, modules), patch.object(sys, 'argv', ['control.py', command]), \
                patch.object(self.client.socket, 'setdefaulttimeout'), \
                patch.object(self.client.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(self.client.time, 'sleep', side_effect=lambda delay: clock.__setitem__(0, clock[0] + delay)), \
                contextlib.redirect_stdout(io.StringIO()):
            return self.client.main()

    def test_hold_remains_confirmed_when_cancel_subscriber_is_missing(self):
        with self.assertRaises(RuntimeError) as failure:
            self.run_boundary_client('hold')
        self.assertTrue(self.mux.held, 'cancel transport failure must not prevent manual hold')
        self.assertEqual(self.mux.generation, 1)
        self.assertIn('held confirmed; cancel unconfirmed', str(failure.exception))

    def test_blocking_publisher_rpc_cannot_make_old_state_fresh(self):
        with self.assertRaisesRegex(RuntimeError, 'no fresh state'):
            self.run_boundary_client('state', delayed_state=True)

    def test_hold_remains_confirmed_when_cancel_status_ack_is_missing(self):
        with self.assertRaisesRegex(RuntimeError, 'held confirmed; cancel unconfirmed: .*status ACK'):
            self.run_boundary_client('hold', cancel_connected=True)
        self.assertTrue(self.mux.held)
        self.assertEqual(self.mux.generation, 1)

    def test_status_header_must_remain_fresh_at_release_decision(self):
        with self.assertRaisesRegex(RuntimeError, 'no fresh owned move_base status'):
            self.run_boundary_client('release', stale_status=True)

    def test_release_cannot_rebind_to_takeover_during_status_wait(self):
        with self.assertRaisesRegex(RuntimeError, 'changed.*explicit.*release'):
            self.run_boundary_client('release', ownership_change='takeover')
        self.assertTrue(self.mux.held)
        self.assertEqual(self.mux.generation, 1)

    def test_release_cannot_rebind_to_restarted_mux(self):
        with self.assertRaisesRegex(RuntimeError, 'changed.*explicit.*release'):
            self.run_boundary_client('release', ownership_change='restart')
        self.assertTrue(self.mux.held)
        self.assertEqual(self.mux.instance_id, 'restarted')

    def test_hold_can_use_latest_takeover_generation(self):
        with self.assertRaisesRegex(RuntimeError, 'held confirmed; cancel unconfirmed'):
            self.run_boundary_client('hold', ownership_change='takeover')
        self.assertTrue(self.mux.held)
        self.assertEqual(self.mux.generation, 2)

    def test_passive_intent_observer_does_not_block_release(self):
        self.assertEqual(self.run_boundary_client(
            'release', fresh_status=True,
            intent_subscribers=['/record', '/slicer_velocity_mux']), 0)
        self.assertFalse(self.mux.held)

    def test_passive_intent_observer_does_not_block_hold(self):
        with self.assertRaisesRegex(RuntimeError, 'held confirmed; cancel unconfirmed'):
            self.run_boundary_client('hold', intent_subscribers=['/slicer_velocity_mux', '/record'])
        self.assertTrue(self.mux.held)

    def test_observer_alone_cannot_replace_expected_mux_subscriber(self):
        for command in ('release', 'hold'):
            with self.subTest(command=command), self.assertRaisesRegex(RuntimeError, 'mux subscriber'):
                self.run_boundary_client(command, fresh_status=True,
                                         intent_subscribers=['/record'])


if __name__ == '__main__':
    unittest.main()
