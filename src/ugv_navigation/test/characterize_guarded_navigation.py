#!/usr/bin/env python3
"""TEST_ONLY reachability experiment; preserve ABORTED/timeout, never fake PASS.

Run this launch only through tools/noetic.sh (no devices, network none).
Process success means data collection completed, not that goals succeeded.
"""
import json
import math
from pathlib import Path
import time

import rospy
from std_msgs.msg import String
from test_guarded_navigation import GuardedNavigationTest


def main():
    rospy.init_node('TEST_ONLY_chain_operator')
    fixture = GuardedNavigationTest
    fixture.setUpClass()
    probe = fixture('test_01_startup_hold_blocks_real_teb_output')
    reasons = []
    sub = rospy.Subscriber('/slicer_nav_arc_guard/hold_reason', String,
                           lambda msg: reasons.append((time.monotonic(), msg.data)), queue_size=100)
    output = Path(rospy.get_param('~output'))
    result = dict(TEST_ONLY=True, plant='ideal differential model integrates final Mux Twist',
                  excludes=['driver PWM', 'MCU', 'deadband', 'inertia', 'physical calibration'],
                  completion_is_not_functional_pass=True, cases=[])
    try:
        probe.wait(lambda: fixture.state is not None and
                   fixture.plant.grid('local_costmap') is not None and
                   fixture.intent.get_num_connections() > 0, 20., 'Missing live chain')
        request = dict(schema_version=1, intent_id='TEST_ONLY_characterization_release',
                       action='release', mux_instance_id=fixture.state['mux_instance_id'],
                       takeover_generation=fixture.state['takeover_generation'])
        fixture.intent.publish(String(data=json.dumps(request)))
        probe.wait(lambda: fixture.state['last_intent_id'] == request['intent_id'] and
                   fixture.state['last_intent_accepted'] and not fixture.state['manual_hold'],
                   5., 'Test-only release refused')
        for name, x, y, yaw in [('forward_0.5', .5, 0., 0.),
                                 ('rotate_45deg', 0., 0., math.pi/4),
                                 ('side_0.2', 0., .2, 0.),
                                 ('side_0.5', 0., .5, 0.)]:
            start, since = fixture.plant.pose(), time.monotonic()
            target = (start[0]+math.cos(start[2])*x-math.sin(start[2])*y,
                      start[1]+math.sin(start[2])*x+math.cos(start[2])*y,
                      start[2]+yaw)
            probe.send(*target)
            terminal = fixture.client.wait_for_result(rospy.Duration(30.))
            outcome = dict(name=name, relative_goal=[x,y,yaw], start=list(start), target=list(target),
                           terminal_within_30s=terminal, state=fixture.client.get_state(),
                           text=fixture.client.get_goal_status_text())
            if not terminal:
                fixture.client.cancel_goal()
                outcome['cancel_terminal_received'] = fixture.client.wait_for_result(rospy.Duration(5.))
                outcome['state_after_cancel'] = fixture.client.get_state()
            time.sleep(.5)
            end = fixture.plant.pose()
            raw = [entry for entry in fixture.raw if entry[0] >= since]
            guarded = [entry for entry in fixture.guarded if entry[0] >= since]
            final = fixture.plant.commands_since(since)
            outcome.update(end=list(end), position_error_m=math.hypot(end[0]-target[0],end[1]-target[1]),
                           yaw_error_rad=abs(math.atan2(math.sin(end[2]-target[2]),math.cos(end[2]-target[2]))),
                           negative_raw_count=sum(v < 0 for t,v,w in raw),
                           raw=raw, guarded=guarded, final=final,
                           hold_reason_events=[entry for entry in reasons if entry[0] >= since])
            result['cases'].append(outcome)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            rospy.loginfo('CHARACTERIZATION %s state=%s error=%.3fm negative_raw=%d',
                          name, outcome['state'], outcome['position_error_m'], outcome['negative_raw_count'])
    finally:
        sub.unregister()
        fixture.tearDownClass()
    rospy.loginfo('Data collection complete: %s. Read each state; this is not a functional PASS.', output)


if __name__ == '__main__':
    main()
