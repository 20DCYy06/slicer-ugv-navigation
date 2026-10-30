#!/usr/bin/env python3
"""TEST_ONLY real selected-DWA -> Guard -> Mux safety and finite obstacle probe.

The existing ideal plant integrates only final Twist. New finite lidar segments
never clamp that plant or inspect its goal. Genuine Action state, cancellation,
map cells and every integrated footprint sample are retained, including failures.
"""
import json
import math
from pathlib import Path
import threading
import time

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import String
from test_guarded_navigation import GuardedNavigationTest

FOOTPRINT = ((-.165, -.14), (-.165, .14), (.27, .14), (.27, -.14))
RECTANGLE = (.75, .85, -.15, .15)


def polygon(pose):
    x, y, yaw = pose
    return [(x + fx * math.cos(yaw) - fy * math.sin(yaw),
             y + fx * math.sin(yaw) + fy * math.cos(yaw)) for fx, fy in FOOTPRINT]


def footprint_intersects_rectangle(pose, rectangle):
    """Separating-axis check against fixed ground-truth geometry, no response."""
    xmin, xmax, ymin, ymax = rectangle
    body = polygon(pose)
    obstacle = ((xmin, ymin), (xmin, ymax), (xmax, ymax), (xmax, ymin))
    axes = [(1., 0.), (0., 1.)]
    for a, b in zip(body, body[1:] + body[:1]):
        axes.append((a[1] - b[1], b[0] - a[0]))
    for ax, ay in axes:
        left = [x * ax + y * ay for x, y in body]
        right = [x * ax + y * ay for x, y in obstacle]
        if max(left) < min(right) - 1e-9 or max(right) < min(left) - 1e-9:
            return False
    return True


def cell_snapshot(grid, x, y):
    if grid is None:
        return None
    mx = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
    my = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
    cells = [(i, j, grid.data[j * grid.info.width + i])
             for i in range(mx - 1, mx + 2) for j in range(my - 1, my + 2)
             if 0 <= i < grid.info.width and 0 <= j < grid.info.height]
    return dict(point=[x, y], neighboring_cells=cells,
                max_occupancy=max((v for i, j, v in cells), default=-1),
                frame=grid.header.frame_id, stamp=grid.header.stamp.to_sec(),
                origin=[grid.info.origin.position.x, grid.info.origin.position.y],
                width=grid.info.width, height=grid.info.height,
                resolution=grid.info.resolution, data=list(grid.data))


def main():
    rospy.init_node('TEST_ONLY_chain_operator')
    scenario = rospy.get_param('~scenario')
    if scenario not in ('cancel', 'scan_loss', 'blocked_wall', 'finite_obstacle'):
        raise ValueError('Unknown TEST_ONLY scenario')
    output = Path(rospy.get_param('~output'))
    result = dict(TEST_ONLY=True, scenario=scenario, status='starting', scenario_pass=False,
                  isolation_required='fresh tools/noetic.sh no-device network-none container/master',
                  plant='ideal differential model integrates final Mux Twist without collision clamp',
                  ground_truth_check='every recorded integrated footprint pose; discrete samples, not a continuous collision proof',
                  excludes=['driver PWM', 'MCU', 'deadband', 'inertia', 'physical calibration'],
                  completion_is_not_functional_pass=True, progress=[])
    fixture, ready, subs = GuardedNavigationTest, False, []
    lock, events, plans, statuses = threading.Lock(), [], [], []
    since, goal = None, ([1.5, .45, 0.] if scenario == 'finite_obstacle' else [1.6, 0., 0.])

    def save():
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')

    def reason(msg):
        with lock:
            events.append((time.monotonic(), msg.data))

    def plan(msg):
        with lock:
            plans.append(dict(monotonic=time.monotonic(), frame=msg.header.frame_id,
                              poses=[[p.pose.position.x, p.pose.position.y] for p in msg.poses]))

    def status(msg):
        with lock:
            statuses.append(dict(monotonic=time.monotonic(),
                                 statuses=[dict(goal_id=s.goal_id.id, status=s.status, text=s.text)
                                           for s in msg.status_list]))

    def snapshot():
        return dict(monotonic=time.monotonic(), pose=list(fixture.plant.pose()),
                    state=fixture.client.get_state(), text=fixture.client.get_goal_status_text())

    def stopped():
        return abs(fixture.plant.velocity) < 1e-9 and abs(fixture.plant.omega) < 1e-9

    def stop_window(label):
        probe.wait(stopped, 3., label + ': final output did not stop')
        at, pose = time.monotonic(), fixture.plant.pose()
        time.sleep(1.)
        commands = fixture.plant.commands_since(at)
        end = fixture.plant.pose()
        proof = dict(start_monotonic=at, end_monotonic=time.monotonic(), start=list(pose),
                     end=list(end), commands=commands,
                     all_zero=bool(commands) and all(abs(c[1]) < 1e-9 and abs(c[6]) < 1e-9
                                                    for c in commands),
                     displacement_m=math.hypot(end[0] - pose[0], end[1] - pose[1]))
        result[label] = proof
        probe.assertTrue(proof['all_zero'], label + ': nonzero command returned')
        probe.assertLess(proof['displacement_m'], .002, label + ': stopped plant translated')

    def cancel(label):
        result[label + '_before'] = snapshot()
        fixture.client.cancel_goal()
        received = fixture.client.wait_for_result(rospy.Duration(5.))
        result[label + '_terminal_received'] = received
        result[label + '_after'] = snapshot()
        probe.assertTrue(received, 'Actual cancellation result missing')
        probe.assertEqual(fixture.client.get_state(), GoalStatus.PREEMPTED)
        stop_window(label + '_stop')

    def wait_terminal(timeout):
        deadline, next_progress = time.monotonic() + timeout, time.monotonic() + 5.
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            if fixture.client.wait_for_result(rospy.Duration(min(.25, remaining))):
                return time.monotonic() <= deadline
            probe.assertIsNone(fixture.plant.error)
            if time.monotonic() >= next_progress:
                sample = snapshot()
                result['progress'].append(sample)
                rospy.loginfo('DWA_SAFETY scenario=%s progress=%s', scenario, sample)
                save()
                next_progress += 5.
        return False

    def observe_obstacle(x, y):
        def marked():
            checks = {name: cell_snapshot(fixture.plant.grid(name), x, y)
                      for name in ('global_costmap', 'local_costmap')}
            if all(c is not None and c['max_occupancy'] >= 65 for c in checks.values()):
                result['obstacle_costmap_snapshots'] = checks
                return True
            return False
        probe.wait(marked, 8., 'Obstacle did not enter both real costmaps')

    save()
    try:
        fixture.setUpClass()
        ready = True
        probe = fixture('test_01_startup_hold_blocks_real_teb_output')
        subs = [rospy.Subscriber('/slicer_nav_arc_guard/hold_reason', String, reason, queue_size=100),
                rospy.Subscriber('/ugv/move_base/GlobalPlanner/plan', PathMsg, plan, queue_size=100),
                rospy.Subscriber('/ugv/move_base/status', GoalStatusArray, status, queue_size=100)]
        probe.wait(lambda: fixture.state is not None and fixture.intent.get_num_connections() > 0 and
                   fixture.plant.grid('global_costmap') is not None and
                   fixture.plant.grid('local_costmap') is not None, 20., 'Missing real chain context')
        time.sleep(1.)
        result.update(effective_base_local_planner=rospy.get_param('/ugv/move_base/base_local_planner'),
                      effective_dwa_params=rospy.get_param('/ugv/move_base/DWAPlannerROS'),
                      effective_guard_params=rospy.get_param('/slicer_nav_arc_guard'),
                      effective_global_costmap_params=rospy.get_param('/ugv/move_base/global_costmap'),
                      effective_local_costmap_params=rospy.get_param('/ugv/move_base/local_costmap'),
                      effective_allow_unknown=rospy.get_param('/ugv/move_base/GlobalPlanner/allow_unknown'))
        probe.assertEqual(result['effective_base_local_planner'], 'dwa_local_planner/DWAPlannerROS')
        probe.assertIs(result['effective_allow_unknown'], False)
        for key in ('effective_global_costmap_params', 'effective_local_costmap_params'):
            footprint = result[key]['footprint']
            if isinstance(footprint, str):
                footprint = json.loads(footprint)
            probe.assertEqual(footprint, [list(p) for p in FOOTPRINT],
                              'Ground-truth footprint must equal the actual costmap footprint')
        for key, expected in [('max_vel_x', .125), ('max_vel_trans', .125), ('min_vel_x', 0.),
                              ('max_vel_y', 0.), ('min_vel_y', 0.), ('max_vel_theta', .25),
                              ('acc_lim_x', .2), ('acc_lim_theta', 1.), ('angular_sim_granularity', .025),
                              ('forward_point_distance', .325)]:
            probe.assertEqual(result['effective_dwa_params'][key], expected, key)
        if scenario == 'finite_obstacle':
            xmin, xmax, ymin, ymax = RECTANGLE
            corners = ((xmin, ymin), (xmin, ymax), (xmax, ymax), (xmax, ymin))
            segments = list(zip(corners, corners[1:] + corners[:1]))
            result['fixed_rectangle'] = list(RECTANGLE)
            result['fixed_segments'] = segments
            fixture.plant.set_segments(segments)
            observe_obstacle(xmin, 0.)
        request = dict(schema_version=1, intent_id='TEST_ONLY_dwa_safety_release', action='release',
                       mux_instance_id=fixture.state['mux_instance_id'],
                       takeover_generation=fixture.state['takeover_generation'])
        fixture.intent.publish(String(data=json.dumps(request)))
        probe.wait(lambda: fixture.state['last_intent_id'] == request['intent_id'] and
                   fixture.state['last_intent_accepted'] and not fixture.state['manual_hold'],
                   5., 'Explicit TEST_ONLY release refused')
        since = time.monotonic()
        result.update(status='collecting', goal=goal, start=list(fixture.plant.pose()), since_monotonic=since)
        save()
        probe.send(*goal)
        if scenario in ('cancel', 'scan_loss', 'blocked_wall'):
            probe.wait(lambda: fixture.plant.velocity > .02, 10., 'Final Mux Twist did not move plant')
            time.sleep(.5)
        if scenario == 'cancel':
            cancel('explicit_cancel')
        elif scenario == 'scan_loss':
            result['scan_disabled'] = snapshot()
            fixture.plant.set_scan(False)
            probe.wait(stopped, 2.5, 'Scan loss did not stop the final chain')
            result['scan_loss_stopped'] = snapshot()
            stop_window('scan_outage_stop')
            probe.assertEqual(fixture.client.get_state(), GoalStatus.ACTIVE,
                              'A short sensor outage must retain the original Action')
            result['scan_reenabled'] = snapshot()
            fixture.plant.set_scan(True)
            probe.wait(lambda: fixture.plant.velocity > .02, 5., 'Original goal did not resume on scan recovery')
            result['same_goal_resumed'] = snapshot()
            time.sleep(.3)
            cancel('after_scan_recovery_cancel')
        elif scenario == 'blocked_wall':
            result['wall_x'] = .8
            result['wall_inserted'] = snapshot()
            fixture.plant.set_wall(.8)
            observe_obstacle(.8, 0.)
            terminal = wait_terminal(25.)
            result.update(terminal_before_timeout=terminal, actual_terminal=snapshot())
            stop_window('blocked_wall_stop_before_cleanup')
            probe.assertTrue(terminal, 'Blocked-wall Action did not genuinely terminate in25s')
            probe.assertEqual(fixture.client.get_state(), GoalStatus.ABORTED,
                              'A full-width wall must not yield arrival')
        else:
            terminal = wait_terminal(55.)
            result.update(terminal_before_timeout=terminal, actual_terminal=snapshot())
            if not terminal:
                cancel('timeout_cleanup_cancel')
            else:
                stop_window('finite_obstacle_terminal_stop')
            result['finite_detour_arrival'] = terminal and fixture.client.get_state() == GoalStatus.SUCCEEDED
        result['scenario_steps_completed'] = True
    except Exception as error:
        result.update(error=repr(error), scenario_steps_completed=False)
    finally:
        if ready:
            result['state_before_cleanup'] = snapshot()
            with fixture.lock:
                result['raw'] = [c for c in fixture.raw if since is not None and c[0] >= since]
                result['guarded'] = [c for c in fixture.guarded if since is not None and c[0] >= since]
            result['final'] = fixture.plant.commands_since(since or 0.)
            result['poses'] = fixture.plant.poses_since(since or 0.)
            result['negative_raw_count'] = sum(c[1] < 0 for c in result['raw'])
            result['final_command_limits_ok'] = bool(result['final']) and all(
                all(math.isfinite(v) for v in c) and 0 <= c[1] <= .125001 and abs(c[6]) <= .250001 and
                all(abs(c[i]) < 1e-9 for i in (2, 3, 4, 5)) for c in result['final'])
            result['stopped_before_cleanup'] = stopped()
            result['ground_truth_no_penetration'] = True
            if scenario == 'finite_obstacle':
                hits = [c for c in result['poses'] if footprint_intersects_rectangle(c[1:4], RECTANGLE)]
                result['ground_truth_collision_pose_count'] = len(hits)
                result['first_ground_truth_collision_poses'] = hits[:3]
                result['ground_truth_no_penetration'] = not hits
                end = result['actual_terminal']['pose'] if 'actual_terminal' in result else fixture.plant.pose()
                result['position_error_m'] = math.hypot(end[0] - goal[0], end[1] - goal[1])
                result['yaw_error_rad'] = abs(math.atan2(math.sin(end[2] - goal[2]), math.cos(end[2] - goal[2])))
            elif scenario == 'blocked_wall':
                foremost = max((x for c in result['poses'] for x, y in polygon(c[1:4])), default=0.)
                result['foremost_footprint_x'] = foremost
                result['ground_truth_no_penetration'] = foremost < .8
            passed = (result.get('scenario_steps_completed', False) and result['negative_raw_count'] == 0 and
                      result['final_command_limits_ok'] and result['stopped_before_cleanup'] and
                      result['ground_truth_no_penetration'])
            if scenario == 'finite_obstacle':
                passed = passed and result.get('finite_detour_arrival', False) and \
                         result['position_error_m'] <= .15 and result['yaw_error_rad'] <= .25
            result.update(status='collected', scenario_pass=passed)
        with lock:
            result.update(hold_reason_events=list(events), global_plans=list(plans),
                          action_status_events=list(statuses))
        save()
        rospy.loginfo('DWA_SAFETY scenario=%s pass=%s output=%s', scenario, result['scenario_pass'], output)
        for sub in subs:
            sub.unregister()
        if ready:
            fixture.tearDownClass()


if __name__ == '__main__':
    main()
