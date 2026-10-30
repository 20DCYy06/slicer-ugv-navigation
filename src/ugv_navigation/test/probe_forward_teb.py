#!/usr/bin/env python3
"""TEST_ONLY fresh-master TEB diagnostics, never a vehicle acceptance test.

Run only through tools/noetic.sh: no devices, network none. The ideal plant
integrates the final Mux command. Collection completion does not mean success.
"""
import json
import math
from pathlib import Path
import threading
import time

import rospy
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import String
from teb_local_planner.msg import FeedbackMsg
from test_guarded_navigation import GuardedNavigationTest


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def segment(points, exact_arc_length):
    a, b = points[:2]
    dx, dy = b['x'] - a['x'], b['y'] - a['y']
    q = dx * math.cos(a['yaw']) + dy * math.sin(a['yaw'])
    lateral = -dx * math.sin(a['yaw']) + dy * math.cos(a['yaw'])
    dyaw = math.atan2(math.sin(b['yaw'] - a['yaw']),
                      math.cos(b['yaw'] - a['yaw']))
    dt = b['time_from_start'] - a['time_from_start']
    chord = math.hypot(dx, dy)
    arc = abs(dyaw * chord / (2 * math.sin(dyaw / 2))) if dyaw else chord
    nh = abs((math.cos(a['yaw']) + math.cos(b['yaw'])) * dy -
             (math.sin(a['yaw']) + math.sin(b['yaw'])) * dx)
    if dt <= 0:
        return dict(q=q, lateral=lateral, dyaw=dyaw, dt=dt, nh=nh,
                    invalid_timestep=True)
    sigmoid = 100 * q / (1 + abs(100 * q))
    v_edge = (arc if exact_arc_length else chord) / dt * sigmoid
    v_extract = (-1 if q < 0 else 1 if q > 0 else 0) * chord / dt
    omega = dyaw / dt
    if abs(omega) > 1e-9:
        px = v_extract / omega * (math.sin(a['yaw'] + dyaw) - math.sin(a['yaw']))
        py = -v_extract / omega * (math.cos(a['yaw'] + dyaw) - math.cos(a['yaw']))
    else:
        px, py = v_extract * dt * math.cos(a['yaw']), v_extract * dt * math.sin(a['yaw'])
    return dict(q=q, lateral=lateral, dyaw=dyaw, dt=dt, nh=nh,
                chord=chord, arc=arc, sigmoid=sigmoid, v_edge=v_edge,
                v_extract=v_extract, omega_extract=omega,
                constant_twist_endpoint_error_m=math.hypot(px - dx, py - dy))


def main():
    rospy.init_node('TEST_ONLY_chain_operator')
    fixture = GuardedNavigationTest
    fixture.setUpClass()
    probe = fixture('test_01_startup_hold_blocks_real_teb_output')
    output = Path(rospy.get_param('~output'))
    initial_yaw = float(rospy.get_param('~initial_yaw'))
    goal_yaw = float(rospy.get_param('~goal_yaw'))
    records, reasons, global_plans = [], [], []
    lock = threading.Lock()
    errors = []
    ns = '/ugv/move_base/TebLocalPlannerROS/'
    exact_arc_length = bool(rospy.get_param(ns + 'exact_arc_length'))

    def feedback(msg):
        try:
            trajectory = msg.trajectories[msg.selected_trajectory_idx].trajectory
            points = [dict(x=p.pose.position.x, y=p.pose.position.y, yaw=yaw(p.pose),
                           time_from_start=p.time_from_start.to_sec()) for p in trajectory[:3]]
            record = dict(monotonic=time.monotonic(), ros_stamp=msg.header.stamp.to_sec(),
                          count=len(trajectory), first_three=points)
            if len(points) >= 2:
                record['first_segment'] = segment(points, exact_arc_length)
            with lock:
                records.append(record)
        except Exception as error:
            with lock:
                errors.append(repr(error))

    def global_plan(msg):
        with lock:
            global_plans.append(dict(monotonic=time.monotonic(),
                                     poses=[dict(x=p.pose.position.x, y=p.pose.position.y,
                                                 yaw=yaw(p.pose)) for p in msg.poses]))

    def reason(msg):
        with lock:
            reasons.append((time.monotonic(), msg.data))

    subs = [rospy.Subscriber(ns + 'teb_feedback', FeedbackMsg, feedback, queue_size=100),
            rospy.Subscriber(ns + 'global_plan', PathMsg, global_plan, queue_size=10),
            rospy.Subscriber('/slicer_nav_arc_guard/hold_reason', String, reason, queue_size=100)]
    result = dict(TEST_ONLY=True, fresh_master_required=True,
                  plant='ideal differential model integrates final Mux Twist',
                  excludes=['driver PWM', 'MCU', 'deadband', 'inertia', 'physical calibration'],
                  completion_is_not_functional_pass=True, status='starting',
                  initial_yaw=initial_yaw, goal=[0.0, 0.5, goal_yaw])

    def save():
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')

    try:
        with fixture.plant.lock:
            fixture.plant.yaw = initial_yaw
        probe.wait(lambda: fixture.state is not None and
                   fixture.plant.grid('local_costmap') is not None and
                   fixture.intent.get_num_connections() > 0 and
                   subs[0].get_num_connections() > 0, 20., 'Missing live chain/feedback transport')
        time.sleep(1.)
        names = ['max_vel_x', 'max_vel_x_backwards', 'max_vel_y', 'max_vel_theta',
                 'acc_lim_x', 'acc_lim_theta', 'allow_init_with_backwards_motion',
                 'global_plan_overwrite_orientation', 'exact_arc_length', 'min_samples',
                 'max_samples', 'dt_ref', 'dt_hysteresis', 'control_look_ahead_poses',
                 'no_inner_iterations', 'no_outer_iterations', 'weight_kinematics_nh',
                 'weight_kinematics_forward_drive', 'publish_feedback', 'footprint_model']
        result['effective_teb_params'] = {name: rospy.get_param(ns + name, None) for name in names}
        result['effective_allow_unknown'] = rospy.get_param('/ugv/move_base/GlobalPlanner/allow_unknown')
        result['effective_guard_params'] = rospy.get_param('/slicer_nav_arc_guard')
        result['effective_costmap_footprint'] = rospy.get_param('/ugv/move_base/local_costmap/footprint')
        request = dict(schema_version=1, intent_id='TEST_ONLY_forward_probe_release',
                       action='release', mux_instance_id=fixture.state['mux_instance_id'],
                       takeover_generation=fixture.state['takeover_generation'])
        fixture.intent.publish(String(data=json.dumps(request)))
        probe.wait(lambda: fixture.state['last_intent_id'] == request['intent_id'] and
                   fixture.state['last_intent_accepted'] and not fixture.state['manual_hold'],
                   5., 'Explicit TEST_ONLY release refused')
        start, since = fixture.plant.pose(), time.monotonic()
        result.update(status='collecting', start=list(start), since_monotonic=since)
        save()
        probe.send(0., .5, goal_yaw)
        terminal = fixture.client.wait_for_result(rospy.Duration(30.))
        result.update(terminal_within_30s=terminal, state=fixture.client.get_state(),
                      text=fixture.client.get_goal_status_text())
        if not terminal:
            fixture.client.cancel_goal()
            result['cancel_terminal_received'] = fixture.client.wait_for_result(rospy.Duration(5.))
            result['state_after_cancel'] = fixture.client.get_state()
        time.sleep(.5)
        end = fixture.plant.pose()
        with fixture.lock:
            raw = [entry for entry in fixture.raw if entry[0] >= since]
            guarded = [entry for entry in fixture.guarded if entry[0] >= since]
        final = fixture.plant.commands_since(since)
        with lock:
            result.update(feedback=list(records), global_plans=list(global_plans),
                          hold_reason_events=list(reasons), callback_errors=list(errors))
        result.update(status='collected', end=list(end),
                      position_error_m=math.hypot(end[0], end[1] - .5),
                      yaw_error_rad=abs(math.atan2(math.sin(end[2] - goal_yaw),
                                                  math.cos(end[2] - goal_yaw))),
                      raw=raw, guarded=guarded, final=final,
                      negative_raw_count=sum(v < 0 for t, v, w in raw),
                      nonzero_guarded_count=sum(abs(v) > 1e-9 or abs(w) > 1e-9 for t, v, w in guarded),
                      nonzero_final_count=sum(abs(c[1]) > 1e-9 or abs(c[6]) > 1e-9 for c in final),
                      final_command_limits_ok=all(0 <= c[1] <= .125001 and abs(c[6]) <= .250001 for c in final))
        save()
        rospy.loginfo('FORWARD_PROBE state=%s error=%.6fm negative_raw=%d feedback=%d output=%s',
                      result['state'], result['position_error_m'], result['negative_raw_count'], len(records), output)
    except Exception as error:
        result.update(status='collection_error', error=repr(error))
        save()
        raise
    finally:
        for sub in subs:
            sub.unregister()
        fixture.tearDownClass()


if __name__ == '__main__':
    main()
