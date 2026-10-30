#!/usr/bin/env python3
"""TEST_ONLY real DWA -> real Guard -> real Mux -> ideal differential model.

Use only tools/noetic.sh with no devices and network none. DWA has no TEB
feedback message; this probe records genuine published paths and raw/final
commands. Collection and timeout cancellation never imply functional success.
"""
import json
import math
from pathlib import Path
import threading
import time

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from dwa_local_planner.cfg import DWAPlannerConfig
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import String
from test_guarded_navigation import GuardedNavigationTest


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node('TEST_ONLY_chain_operator')
    output = Path(rospy.get_param('~output'))
    goal = [float(rospy.get_param('~goal_' + key)) for key in ('x', 'y', 'yaw')]
    initial_yaw = float(rospy.get_param('~initial_yaw'))
    timeout = float(rospy.get_param('~timeout'))
    result = dict(TEST_ONLY=True, fresh_master_required=True,
                  plant='ideal differential model integrates final Mux Twist',
                  excludes=['driver PWM', 'MCU', 'deadband', 'inertia', 'physical calibration'],
                  completion_is_not_functional_pass=True, status='starting',
                  initial_yaw=initial_yaw, goal=goal, timeout_s=timeout,
                  upstream_dwa_dynamic_defaults=DWAPlannerConfig.defaults)
    lock = threading.Lock()
    paths = {'global_plan': [], 'local_plan': [], 'planner_plan': []}
    reasons, adjustments, statuses, callback_errors = [], [], [], []
    fixture, fixture_ready, subs = GuardedNavigationTest, False, []
    ns = '/ugv/move_base/DWAPlannerROS/'

    def save():
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')

    def path(msg, name):
        try:
            record = dict(monotonic=time.monotonic(), ros_stamp=msg.header.stamp.to_sec(),
                          frame=msg.header.frame_id,
                          poses=[dict(x=p.pose.position.x, y=p.pose.position.y,
                                      yaw=yaw(p.pose)) for p in msg.poses])
            with lock:
                paths[name].append(record)
        except Exception as error:
            with lock:
                callback_errors.append(repr(error))

    def reason(msg):
        with lock:
            reasons.append((time.monotonic(), msg.data))

    def adjustment(msg):
        with lock:
            adjustments.append((time.monotonic(), msg.data))

    def status(msg):
        with lock:
            statuses.append(dict(monotonic=time.monotonic(),
                                 statuses=[dict(goal_id=s.goal_id.id, status=s.status,
                                                text=s.text) for s in msg.status_list]))

    save()
    try:
        fixture.setUpClass()
        fixture_ready = True
        probe = fixture('test_01_startup_hold_blocks_real_teb_output')
        with fixture.plant.lock:
            fixture.plant.yaw = initial_yaw
        topics = {'global_plan': ns + 'global_plan', 'local_plan': ns + 'local_plan',
                  'planner_plan': '/ugv/move_base/GlobalPlanner/plan'}
        subs = [rospy.Subscriber(topic, PathMsg, path, callback_args=name, queue_size=100)
                for name, topic in topics.items()]
        subs += [rospy.Subscriber('/slicer_nav_arc_guard/hold_reason', String, reason, queue_size=100),
                 rospy.Subscriber('/slicer_nav_arc_guard/adjustment', String, adjustment, queue_size=100),
                 rospy.Subscriber('/ugv/move_base/status', GoalStatusArray, status, queue_size=100)]
        probe.wait(lambda: fixture.state is not None and
                   fixture.plant.grid('local_costmap') is not None and
                   fixture.plant.grid('global_costmap') is not None and
                   fixture.intent.get_num_connections() > 0,
                   20., 'Missing live Guard/Mux/costmap context')
        time.sleep(1.)
        result.update(effective_base_local_planner=rospy.get_param('/ugv/move_base/base_local_planner'),
                      effective_dwa_params=rospy.get_param(ns),
                      effective_selected_planner_params=rospy.get_param(ns),
                      inactive_teb_params=rospy.get_param('/ugv/move_base/TebLocalPlannerROS', None),
                      effective_allow_unknown=rospy.get_param('/ugv/move_base/GlobalPlanner/allow_unknown'),
                      effective_guard_params=rospy.get_param('/slicer_nav_arc_guard'),
                      effective_global_costmap_params=rospy.get_param('/ugv/move_base/global_costmap'),
                      effective_local_costmap_params=rospy.get_param('/ugv/move_base/local_costmap'))
        grid = fixture.plant.grid('local_costmap')
        result['initial_local_costmap'] = dict(
            frame=grid.header.frame_id, stamp=grid.header.stamp.to_sec(),
            width=grid.info.width, height=grid.info.height, resolution=grid.info.resolution,
            origin=dict(x=grid.info.origin.position.x, y=grid.info.origin.position.y),
            data=list(grid.data))
        if result['effective_base_local_planner'] != 'dwa_local_planner/DWAPlannerROS':
            raise AssertionError('The active local planner is not DWA')
        if result['effective_allow_unknown'] is not False:
            raise AssertionError('Unknown-space planning was weakened')
        request = dict(schema_version=1, intent_id='TEST_ONLY_dwa_probe_release',
                       action='release', mux_instance_id=fixture.state['mux_instance_id'],
                       takeover_generation=fixture.state['takeover_generation'])
        fixture.intent.publish(String(data=json.dumps(request)))
        probe.wait(lambda: fixture.state['last_intent_id'] == request['intent_id'] and
                   fixture.state['last_intent_accepted'] and not fixture.state['manual_hold'],
                   5., 'Explicit TEST_ONLY release refused')
        start, since = fixture.plant.pose(), time.monotonic()
        result.update(status='collecting', start=list(start), since_monotonic=since)
        save()
        probe.send(*goal)
        terminal, next_progress, deadline = False, since + 5., since + timeout
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            received = fixture.client.wait_for_result(rospy.Duration(min(.25, remaining)))
            if received:
                terminal = time.monotonic() <= deadline
                break
            if time.monotonic() >= next_progress:
                rospy.loginfo('DWA_PROBE collecting elapsed=%.1fs state=%s pose=%s',
                              time.monotonic() - since, fixture.client.get_state(), fixture.plant.pose())
                next_progress += 5.
            if fixture.plant.error:
                raise AssertionError(fixture.plant.error)
        terminal_pose = fixture.plant.pose()
        result.update(terminal_before_timeout=terminal,
                      elapsed_before_cancel_s=time.monotonic() - since,
                      state=fixture.client.get_state(), text=fixture.client.get_goal_status_text(),
                      terminal_pose=list(terminal_pose))
        if not terminal:
            fixture.client.cancel_goal()
            result['cancel_terminal_received'] = fixture.client.wait_for_result(rospy.Duration(5.))
            result['state_after_cancel'] = fixture.client.get_state()
            result['text_after_cancel'] = fixture.client.get_goal_status_text()
        time.sleep(.5)
        end = fixture.plant.pose()
        with fixture.lock:
            raw = [entry for entry in fixture.raw if entry[0] >= since]
            guarded = [entry for entry in fixture.guarded if entry[0] >= since]
        final, poses = fixture.plant.commands_since(since), fixture.plant.poses_since(since)
        with lock:
            result.update(global_plans=list(paths['global_plan']), local_plans=list(paths['local_plan']),
                          planner_plans=list(paths['planner_plan']),
                          hold_reason_events=list(reasons), adjustment_events=list(adjustments),
                          action_status_events=list(statuses), callback_errors=list(callback_errors))
        position_error = math.hypot(terminal_pose[0] - goal[0], terminal_pose[1] - goal[1])
        yaw_error = abs(math.atan2(math.sin(terminal_pose[2] - goal[2]),
                                   math.cos(terminal_pose[2] - goal[2])))
        final_limits_ok = all(0 <= c[1] <= .125001 and abs(c[6]) <= .250001 and
                              all(abs(c[i]) <= 1e-9 for i in (2, 3, 4, 5)) for c in final)
        negative_raw_count = sum(v < 0 for t, v, w in raw)
        stopped = abs(fixture.plant.velocity) < 1e-9 and abs(fixture.plant.omega) < 1e-9
        result.update(status='collected', end=list(end), position_error_m=position_error,
                      yaw_error_rad=yaw_error, raw=raw, guarded=guarded, final=final, poses=poses,
                      negative_raw_count=negative_raw_count,
                      nonzero_guarded_count=sum(abs(v) > 1e-9 or abs(w) > 1e-9 for t, v, w in guarded),
                      nonzero_final_count=sum(abs(c[1]) > 1e-9 or abs(c[6]) > 1e-9 for c in final),
                      final_command_limits_ok=final_limits_ok,
                      stopped_after_goal_or_cancel=stopped,
                      functional_pass=terminal and result['state'] == GoalStatus.SUCCEEDED and
                                      position_error <= .15 and yaw_error <= .25 and
                                      negative_raw_count == 0 and final_limits_ok and stopped)
        save()
        rospy.loginfo('DWA_PROBE state=%s terminal=%s error=%.6fm yaw=%.6frad negative_raw=%d output=%s',
                      result['state'], terminal, position_error, yaw_error, negative_raw_count, output)
    except Exception as error:
        result.update(status='collection_error', error=repr(error))
        save()
        raise
    finally:
        for sub in subs:
            sub.unregister()
        if fixture_ready:
            fixture.tearDownClass()


if __name__ == '__main__':
    main()
