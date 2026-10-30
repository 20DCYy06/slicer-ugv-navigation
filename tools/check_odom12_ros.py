#!/usr/bin/env python3
"""TEST_ONLY: real navigation plugins with ideal sensors in Docker --network none.

Run from the workspace, after building, with:
  tools/noetic.sh python3 tools/check_odom12_ros.py /workspace/tmp/odom12-check

This starts its own loopback master and the existing slicer_odom.launch with
connect_control=false. It publishes only synthetic odometry, TF and scans via
the established TestOnlyDifferentialPlant. It does not validate hardware,
localization fusion, guard behavior, obstacle avoidance or goal completion.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback
import xmlrpc.client


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def cell(grid, x, y):
    column = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
    row = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
    if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
        return None
    return grid.data[row * grid.info.width + column]


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New directory for TEST_ONLY evidence')
    args = parser.parse_args()
    require(Path('/.dockerenv').is_file(), 'TEST_ONLY requires Docker')
    require({p.name for p in Path('/sys/class/net').iterdir()} == {'lo'},
            'TEST_ONLY requires --network none (only loopback may exist)')
    root = Path(__file__).resolve().parents[1]
    profile = root / 'src/ugv_bringup/config/rolling_global_12m.yaml'
    require(profile.is_file(), '12m production profile is missing: {}'.format(profile))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:12417', ROS_IP='127.0.0.1',
                      ROS_HOME=str(output / 'ros'), ROS_LOG_DIR=str(output / 'roslog'))
    os.environ.pop('ROS_HOSTNAME', None)
    result = dict(TEST_ONLY=True, passed=False, no_hardware=True,
                  boundary='Ideal sensor/kinematics software check; no real fusion, guard or hardware validation',
                  cases=[])
    core = launch = plant = client = None
    log_files = []
    started = time.monotonic()
    try:
        core_log = (output / 'master.log').open('w')
        log_files.append(core_log)
        core = subprocess.Popen(['roscore', '-p', '12417'], stdout=core_log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic() + 12
        while True:
            require(core.poll() is None, 'TEST_ONLY roscore exited before becoming ready')
            try:
                require(master.getPid('/TEST_ONLY_odom12')[0] == 1, 'Master rejected getPid')
                break
            except (OSError, xmlrpc.client.Error):
                if time.monotonic() > deadline:
                    raise RuntimeError('TEST_ONLY private master did not start')
                time.sleep(0.1)

        import actionlib
        import rospy
        from actionlib_msgs.msg import GoalStatus, GoalStatusArray
        from geometry_msgs.msg import PoseStamped
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from nav_msgs.msg import Path as RosPath
        from nav_msgs.srv import GetPlan, GetPlanRequest

        sys.path.insert(0, str(root / 'src/ugv_navigation/test'))
        from test_odom_navigation import TestOnlyDifferentialPlant

        class OpenRoomPlant(TestOnlyDifferentialPlant):
            # Keep the mature finite-ray fixture; move its artificial room walls
            # beyond sensor range so they do not enclose the (3,4) unknown goal.
            ROOM = (-20.0, 20.0, -20.0, 20.0)

        rospy.init_node('TEST_ONLY_odom12', disable_signals=True)
        require(not rospy.get_param('/use_sim_time', False), 'TEST_ONLY requires wall time')
        plant = OpenRoomPlant()
        plans, statuses = [], []
        observation_lock = threading.Lock()

        def on_plan(message):
            points = [(round(p.pose.position.x, 3), round(p.pose.position.y, 3))
                      for p in message.poses]
            with observation_lock:
                plans.append(dict(at=time.monotonic(), stamp=message.header.stamp.to_sec(),
                                  poses=len(points), first=points[0] if points else None,
                                  sha256=hashlib.sha256(repr(points).encode()).hexdigest()))

        def on_status(message):
            with observation_lock:
                statuses.extend(dict(at=time.monotonic(), id=s.goal_id.id, state=s.status)
                                for s in message.status_list)

        subscribers = [
            rospy.Subscriber('/ugv/move_base/GlobalPlanner/plan', RosPath, on_plan, queue_size=50),
            rospy.Subscriber('/ugv/move_base/status', GoalStatusArray, on_status, queue_size=50)]
        launch_command = ['roslaunch', 'ugv_bringup', 'slicer_odom.launch',
                          'connect_control:=false', 'local_planner:=dwa',
                          'global_costmap_config:=' + str(profile)]
        result['launch_command'] = launch_command
        launch_log = (output / 'navigation.log').open('w')
        log_files.append(launch_log)
        launch = subprocess.Popen(launch_command, stdout=launch_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)

        def wait(predicate, timeout, message):
            end = time.monotonic() + timeout
            while time.monotonic() < end and not rospy.is_shutdown():
                require(plant.error is None, 'TEST_ONLY plant failed: {}'.format(plant.error))
                require(launch.poll() is None, 'Navigation launch exited; inspect navigation.log')
                if predicate():
                    return
                time.sleep(0.025)
            raise AssertionError('{}; pose={}'.format(message, plant.pose()))

        def run_case(name, operation):
            case = dict(name=name, passed=False)
            result['cases'].append(case)
            try:
                case['evidence'] = operation()
                case['passed'] = True
            except Exception as error:
                case['error'] = str(error)
                with (output / 'errors.log').open('a') as log:
                    traceback.print_exc(file=log)
            print('TEST_ONLY ' + json.dumps(case), flush=True)
            (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')

        wait(lambda: all(plant.grid(name) is not None for name in
                         ('global_costmap', 'local_costmap')), 25, 'Costmaps did not publish')
        params = rospy.get_param('/ugv/move_base')
        (output / 'move_base_parameters.json').write_text(json.dumps(params, indent=2) + '\n')
        client = actionlib.SimpleActionClient('/ugv/move_base', MoveBaseAction)
        require(client.wait_for_server(rospy.Duration(10)), 'MoveBaseAction server unavailable')
        rospy.wait_for_service('/ugv/move_base/make_plan', timeout=10)
        make_plan = rospy.ServiceProxy('/ugv/move_base/make_plan', GetPlan)

        def pose_message(x, y):
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = 'odom', rospy.Time.now()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1
            return pose

        def dimensions():
            dimensions = {}
            for name, expected in (('global_costmap', 240), ('local_costmap', 80)):
                grid = plant.grid(name)
                dimensions[name] = dict(width=grid.info.width, height=grid.info.height,
                                        resolution=grid.info.resolution)
                require(grid.info.width == grid.info.height == expected,
                        '{} must be {}x{}'.format(name, expected, expected))
                require(abs(grid.info.resolution - 0.05) < 1e-6, 'Resolution changed')
            require(params['base_global_planner'] == 'global_planner/GlobalPlanner',
                    'Real GlobalPlanner is not selected')
            require(params['base_local_planner'] == 'dwa_local_planner/DWAPlannerROS',
                    'Real DWA is not selected')
            require(params['GlobalPlanner']['allow_unknown'] is True, 'Unknown planning disabled')
            require(params['global_costmap']['rolling_window'] is True, 'Global map must roll')
            require(params['planner_frequency'] == 1.0, 'planner_frequency must remain 1Hz')
            return dimensions

        def parameter_comparison():
            import yaml
            base = ['roslaunch', '--dump-params', 'ugv_bringup', 'slicer_odom.launch',
                    'connect_control:=false', 'local_planner:=dwa']
            dumps = {}
            for name, extra in (('default', []), ('12m', ['global_costmap_config:=' + str(profile)])):
                text = subprocess.check_output(base + extra, stderr=subprocess.STDOUT,
                                               timeout=20, universal_newlines=True)
                (output / ('launch_parameters_' + name + '.yaml')).write_text(text)
                dumps[name] = yaml.safe_load(text)
            differences = {key: [dumps['default'].get(key), dumps['12m'].get(key)]
                           for key in set(dumps['default']) | set(dumps['12m'])
                           if dumps['default'].get(key) != dumps['12m'].get(key)}
            expected = {'/ugv/move_base/global_costmap/width': [8.0, 12.0],
                        '/ugv/move_base/global_costmap/height': [8.0, 12.0],
                        '/ugv/move_base/GlobalPlanner/allow_unknown': [False, True]}
            require(differences == expected, 'Unexpected launch parameter differences: {}'.format(differences))
            local_keys = [key for key in dumps['default'] if '/local_costmap/' in key]
            require(local_keys, 'No local costmap parameters in launch dump')
            require(all(dumps['default'][key] == dumps['12m'].get(key) for key in local_keys),
                    'Local costmap parameters changed')
            return dict(differences=differences, identical_local_parameters=len(local_keys),
                        other_parameters_unchanged=True)

        def plans_and_unknown():
            wait(lambda: cell(plant.grid('global_costmap'), 1, 1) == 0,
                 8, '(1,1) did not become observed free')
            evidence = []
            require(math.hypot(3, 4) > params['global_costmap']['obstacle_layer']['scan']['raytrace_range'],
                    '(3,4) must be beyond initial raytrace range')
            for x, y in ((1, 1), (3, 4)):
                before = cell(plant.grid('global_costmap'), x, y)
                request = GetPlanRequest(start=pose_message(*plant.pose()[:2]),
                                         goal=pose_message(x, y), tolerance=0)
                outcome, completed = {}, threading.Event()

                def call():
                    try:
                        outcome['response'] = make_plan(request)
                    except Exception as error:
                        outcome['error'] = str(error)
                    finally:
                        completed.set()

                thread = threading.Thread(target=call, name='TEST_ONLY_GetPlan_timeout', daemon=True)
                thread.start()
                require(completed.wait(5), 'GetPlan exceeded 5 seconds')
                require('error' not in outcome, 'GetPlan failed: {}'.format(outcome.get('error')))
                response = outcome['response']
                require(len(response.plan.poses) > 2, 'GetPlan returned no path to {}'.format((x, y)))
                end = response.plan.poses[-1].pose.position
                require(math.hypot(end.x - x, end.y - y) < 0.1, 'GetPlan did not reach requested goal')
                after = cell(plant.grid('global_costmap'), x, y)
                evidence.append(dict(goal=[x, y], poses=len(response.plan.poses),
                                     cell_before=before, cell_after=after))
                if (x, y) == (3, 4):
                    require(before == after == -1, 'Unknown goal was changed into observed free')
            return evidence

        def mark_and_clear():
            def patch(name):
                grid = plant.grid(name)
                return [cell(grid, x, y) for x in (0.45, 0.5, 0.55)
                        for y in (0.4, 0.45, 0.5, 0.55, 0.6)]
            plant.set_segments([((0.5, -0.2), (0.5, 1.3))])
            try:
                wait(lambda: all(max(patch(name)) == 100 for name in
                                 ('global_costmap', 'local_costmap')), 8,
                     'Finite scan wall was not marked lethal in both maps')
                marked = {name: patch(name) for name in ('global_costmap', 'local_costmap')}
            finally:
                plant.set_segments([])
            wait(lambda: all(set(patch(name)) == {0} for name in
                             ('global_costmap', 'local_costmap')), 8,
                 'Removed scan wall did not ray-clear to free in both maps')
            return dict(wall=[[0.5, -0.2], [0.5, 1.3]], marked=marked,
                        cleared={name: patch(name) for name in ('global_costmap', 'local_costmap')})

        def action_replanning_and_cancel():
            goal = MoveBaseGoal(target_pose=pose_message(3, 4))
            since = time.monotonic()
            client.send_goal(goal)
            wait(lambda: client.get_state() == GoalStatus.ACTIVE, 5, 'Action did not become ACTIVE')
            try:
                wait(lambda: len([p for p in plans if p['at'] >= since]) >= 4
                     and len({p['sha256'] for p in plans if p['at'] >= since}) > 1,
                     9, '1Hz planner did not publish updated plans during one Action')
                require(client.get_state() == GoalStatus.ACTIVE, 'Action terminated before cancel')
                with observation_lock:
                    action_plans = [dict(p) for p in plans if p['at'] >= since]
                    active_ids = sorted({s['id'] for s in statuses
                                         if s['at'] >= since and s['state'] == GoalStatus.ACTIVE})
                require(len(active_ids) == 1, 'Expected exactly one active Action goal ID')
                require(action_plans[-1]['at'] - action_plans[0]['at'] >= 1.5,
                        'Plan publications did not span successive planner periods')
                require(len({p['sha256'] for p in action_plans}) > 1,
                        'Published global plan did not update as ideal plant moved')
                commands = plant.commands_since(since)
                require(any(abs(c[1]) > 0.001 or abs(c[6]) > 0.001 for c in commands),
                        'Real DWA emitted no nonzero raw command')
                evidence = dict(goal_id=active_ids[0], plans=action_plans,
                                nonzero_raw_commands=sum(abs(c[1]) > 0.001 or abs(c[6]) > 0.001
                                                         for c in commands))
            finally:
                canceled_at = time.monotonic()
                client.cancel_goal()
            require(client.wait_for_result(rospy.Duration(5)), 'Cancel did not return within 5s')
            require(client.get_state() == GoalStatus.PREEMPTED, 'Cancel result was not PREEMPTED')
            wait(lambda: any(max(abs(v) for v in c[1:]) < 1e-6
                             for c in plant.commands_since(canceled_at)), 3,
                 'Cancel did not emit an actual zero raw Twist')
            time.sleep(0.3)
            stopped_at, stopped_pose = time.monotonic(), plant.pose()
            time.sleep(1.2)
            commands = plant.commands_since(stopped_at)
            require(all(max(abs(v) for v in c[1:]) < 1e-6 for c in commands),
                    'Nonzero raw Twist revived after cancel')
            require(math.hypot(plant.pose()[0] - stopped_pose[0], plant.pose()[1] - stopped_pose[1]) < 0.01,
                    'Ideal plant kept translating after cancel')
            require(abs(plant.pose()[2] - stopped_pose[2]) < 0.02, 'Ideal plant kept rotating after cancel')
            require(client.get_state() == GoalStatus.PREEMPTED, 'Canceled Action revived')
            evidence.update(cancel_state=client.get_state(), stop_observation_seconds=1.2,
                            pose_after_cancel=plant.pose())
            return evidence

        run_case('global_12m_local_4m_real_plugins', dimensions)
        run_case('default_vs_12m_launch_parameter_dump', parameter_comparison)
        run_case('get_plan_observed_and_unknown_targets', plans_and_unknown)
        run_case('scan_wall_mark_and_ray_clear', mark_and_clear)
        run_case('same_action_1hz_replanning_and_cancel_stop', action_replanning_and_cancel)
        result['master_state'] = master.getSystemState('/TEST_ONLY_odom12')[2]
        result['passed'] = all(case['passed'] for case in result['cases'])
        for subscriber in subscribers:
            subscriber.unregister()
    except Exception as error:
        result['error'] = str(error)
        with (output / 'errors.log').open('a') as log:
            traceback.print_exc(file=log)
    finally:
        if client is not None:
            client.cancel_all_goals()
        stop_process(launch)
        if plant is not None:
            plant.close()
        stop_process(core)
        for log in log_files:
            log.close()
        result['elapsed_seconds'] = round(time.monotonic() - started, 3)
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print('TEST_ONLY ' + json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
