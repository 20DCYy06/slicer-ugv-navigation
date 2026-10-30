#!/usr/bin/env python3
"""TEST_ONLY real TEB/DWA characterization in no-device Docker --network none.

After the workspace is built, run through tools/noetic.sh. Every case starts
fresh navigation and an ideal differential plant on a private loopback master.
The plant integrates RAW output unchanged, including negative vx. No Guard,
Mux, driver, release, command clamp, or fake Action result is involved.

Success requires BOTH an actual Action result and strict nonnegative raw vx.
An Action completed using reverse motion is recorded as incompatible, not a
forward-only pass. No software result is evidence of vehicle performance.
"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback
import xmlrpc.client

from check_odom12_ros import require, stop_process


CASES = {
    'forward': (0.7, 0.0, 0.0),
    'rotate': (0.0, 0.0, math.pi / 2),
    'turnaround': (0.0, 0.0, math.pi),
    'side': (0.0, 0.5, math.pi / 2),
    'behind': (-0.5, 0.0, math.pi),
    'obstacle': (1.4, 0.0, 0.0),
}


def trajectory_metrics(poses, commands):
    """Describe the observed ideal trajectory; no goal or controller knowledge."""
    if not poses:
        return {}
    first = poses[0]
    path_length = yaw_change = abs_yaw_change = 0.0
    first_quarter_turn = first_translation = None
    maximum_displacement = 0.0
    for previous, current in zip(poses, poses[1:]):
        path_length += math.hypot(current[1] - previous[1], current[2] - previous[2])
        dyaw = math.atan2(math.sin(current[3] - previous[3]),
                         math.cos(current[3] - previous[3]))
        yaw_change += dyaw
        abs_yaw_change += abs(dyaw)
        displacement = math.hypot(current[1] - first[1], current[2] - first[2])
        maximum_displacement = max(maximum_displacement, displacement)
        if first_quarter_turn is None and abs(yaw_change) >= math.pi / 2:
            first_quarter_turn = dict(elapsed_seconds=current[0] - first[0],
                                      displacement_m=displacement, path_length_m=path_length)
        if first_translation is None and displacement >= 0.02:
            first_translation = dict(elapsed_seconds=current[0] - first[0],
                                     heading_change_rad=yaw_change, displacement_m=displacement)
    return dict(observed_seconds=poses[-1][0] - first[0], path_length_m=path_length,
                maximum_displacement_m=maximum_displacement,
                total_abs_yaw_change_rad=abs_yaw_change,
                first_90deg_heading_change=first_quarter_turn,
                first_2cm_translation=first_translation,
                reverse_distance_from_raw_m=sum(max(-a[1], 0.0) * (b[0] - a[0])
                                                for a, b in zip(commands, commands[1:])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New directory for TEST_ONLY evidence')
    parser.add_argument('--case', action='append', choices=CASES,
                        help='Repeat to select cases; default: all available cases')
    parser.add_argument('--timeout', type=float, default=60.0, help='Action timeout per case')
    parser.add_argument('--planner', choices=('teb', 'dwa'), default='teb',
                        help='Real plugin to compare using identical ideal plant and goals')
    args = parser.parse_args()
    require(args.timeout > 0 and math.isfinite(args.timeout), 'Timeout must be positive and finite')
    require(Path('/.dockerenv').is_file(), 'TEST_ONLY requires Docker')
    require({p.name for p in Path('/sys/class/net').iterdir()} == {'lo'},
            'TEST_ONLY requires --network none (only loopback may exist)')
    root = Path(__file__).resolve().parents[1]
    profile = root / 'src/ugv_bringup/config/rolling_global_12m.yaml'
    require(profile.is_file(), 'The existing 12m profile is required')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:12427', ROS_IP='127.0.0.1',
                      ROS_HOME=str(output / 'ros'), ROS_LOG_DIR=str(output / 'roslog'))
    os.environ.pop('ROS_HOSTNAME', None)
    result = dict(TEST_ONLY=True, passed=False, no_hardware=True, cases=[],
                  planner=args.planner,
                  boundary='Real navigation; ideal raw-integrating plant; no Guard/Mux or real fusion',
                  strict_forward_only_rule='Any raw linear.x < 0 is incompatible, without epsilon')
    core = launch = plant = client = None
    started = time.monotonic()
    logs = []

    def save():
        (output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')

    try:
        log = (output / 'master.log').open('w')
        logs.append(log)
        core = subprocess.Popen(['roscore', '-p', '12427'], stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic() + 12
        while True:
            require(core.poll() is None, 'Private ROS master exited')
            try:
                require(master.getPid('/TEST_ONLY_teb')[0] == 1, 'Master rejected getPid')
                break
            except (OSError, xmlrpc.client.Error):
                if time.monotonic() > deadline:
                    raise RuntimeError('Private ROS master did not become ready')
                time.sleep(0.1)

        import actionlib
        import rospy
        from actionlib_msgs.msg import GoalStatus
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from nav_msgs.msg import Path as RosPath

        sys.path.insert(0, str(root / 'src/ugv_navigation/test'))
        from test_odom_navigation import TestOnlyDifferentialPlant

        rospy.init_node('TEST_ONLY_teb', disable_signals=True)
        require(not rospy.get_param('/use_sim_time', False), 'TEST_ONLY requires wall time')
        result['plugin_package'] = subprocess.check_output(
            ['dpkg-query', '-W', 'ros-noetic-{}-local-planner'.format(args.planner)],
            universal_newlines=True, timeout=5).strip()

        for name in args.case or CASES:
            case_started = time.monotonic()
            case = dict(name=name, goal=CASES[name], passed=False)
            result['cases'].append(case)
            case_output = output / name
            case_output.mkdir()
            plans, subscribers = [], []
            observation_lock = threading.Lock()
            plant = TestOnlyDifferentialPlant()

            def on_plan(message):
                with observation_lock:
                    # Retain the first plan's orientation near the robot, enough
                    # to inspect a behind-goal seed without copying every path.
                    if len(plans) < 3:
                        plans.append(dict(stamp=message.header.stamp.to_sec(),
                                          count=len(message.poses), first_poses=[
                            dict(x=p.pose.position.x, y=p.pose.position.y,
                                 yaw=2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                            for p in message.poses[:8]]))

            subscribers.append(rospy.Subscriber('/ugv/move_base/GlobalPlanner/plan',
                                                RosPath, on_plan, queue_size=5))
            command = ['roslaunch', 'ugv_bringup', 'slicer_odom.launch',
                       'connect_control:=false', 'local_planner:=' + args.planner,
                       'global_costmap_config:=' + str(profile)]
            case['launch_command'] = command
            log = (case_output / 'navigation.log').open('w')
            logs.append(log)
            launch = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)

            def wait(predicate, timeout, message):
                until = time.monotonic() + timeout
                while time.monotonic() < until and not rospy.is_shutdown():
                    require(plant.error is None, 'Ideal plant failed: {}'.format(plant.error))
                    require(launch.poll() is None, 'Navigation exited; inspect navigation.log')
                    if predicate():
                        return
                    time.sleep(0.025)
                raise AssertionError('{}; pose={}'.format(message, plant.pose()))

            since = None
            try:
                wait(lambda: all(plant.grid(n) is not None for n in
                                 ('global_costmap', 'local_costmap')), 25, 'Missing real costmaps')
                params = rospy.get_param('/ugv/move_base')
                (case_output / 'parameters.json').write_text(json.dumps(params, indent=2) + '\n')
                planner_name = 'TebLocalPlannerROS' if args.planner == 'teb' else 'DWAPlannerROS'
                active = params[planner_name]
                require(params['base_local_planner'] ==
                        '{}_local_planner/{}'.format(args.planner, planner_name),
                        'Requested real local planner is not selected')
                require(params['global_costmap']['width'] == params['global_costmap']['height'] == 12.0,
                        'The 12m global profile is not active')
                # Costmap dynamic_reconfigure writes its polygon back as a
                # JSON string; TEB retains the original nested parameter list.
                footprints = [json.loads(params[n]['footprint'])
                              if isinstance(params[n]['footprint'], str)
                              else params[n]['footprint']
                              for n in ('global_costmap', 'local_costmap')]
                require(footprints[0] == footprints[1], 'Costmaps do not share one footprint')
                if args.planner == 'teb':
                    require(footprints[0] == active['footprint_model']['vertices'],
                            'TEB does not share the costmap footprint')
                require(active['max_vel_y'] == 0 and active.get('min_turning_radius', 0) == 0,
                        'Differential drive and in-place rotation settings changed')
                publishers, receivers, services = master.getSystemState('/TEST_ONLY_teb')[2]
                require(not any(t == '/cmd_vel' for t, nodes in publishers),
                        'A chassis command publisher appeared')
                require(not any('guard' in node or 'mux' in node or 'driver' in node
                                for topic, nodes in publishers + receivers for node in nodes),
                        'Unexpected hardware/control chain node')
                case['runtime_contract'] = dict(plugin=params['base_local_planner'],
                    footprint=footprints[0], global_size_m=12.0,
                    local_size_m=params['local_costmap']['width'],
                    max_vel_x=active['max_vel_x'], max_vel_theta=active['max_vel_theta'])
                if name == 'obstacle':
                    plant.set_segments([((0.65, -0.3), (0.65, 0.3))])

                    def obstacle_marked():
                        grid = plant.grid('local_costmap')
                        col = math.floor((0.65 - grid.info.origin.position.x) / grid.info.resolution)
                        row = math.floor((0.0 - grid.info.origin.position.y) / grid.info.resolution)
                        return grid.data[row * grid.info.width + col] == 100

                    wait(obstacle_marked, 8, 'Synthetic finite obstacle was not marked')
                    case['obstacle'] = [[0.65, -0.3], [0.65, 0.3]]
                client = actionlib.SimpleActionClient('/ugv/move_base', MoveBaseAction)
                require(client.wait_for_server(rospy.Duration(10)), 'MoveBaseAction server missing')
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'odom'
                goal.target_pose.header.stamp = rospy.Time.now()
                x, y, yaw = CASES[name]
                goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = x, y
                goal.target_pose.pose.orientation.z = math.sin(yaw / 2)
                goal.target_pose.pose.orientation.w = math.cos(yaw / 2)
                since = time.monotonic()
                client.send_goal(goal)
                terminal = client.wait_for_result(rospy.Duration(args.timeout))
                case['action_elapsed_seconds'] = time.monotonic() - since
                state, text = client.get_state(), client.get_goal_status_text()
                end = plant.pose()
                position_error = math.hypot(end[0] - x, end[1] - y)
                yaw_error = abs(math.atan2(math.sin(end[2] - yaw), math.cos(end[2] - yaw)))
                case.update(terminal_within_timeout=terminal, action_state=state, action_text=text,
                            end_pose=end, position_error_m=position_error, yaw_error_rad=yaw_error)
                # Cancel only a still-active goal. Canceling after SUCCEEDED
                # makes actionlib report PREEMPTING after its DONE state.
                if not terminal:
                    client.cancel_goal()
                    require(client.wait_for_result(rospy.Duration(5)), 'Cancel did not terminate Action')
                wait(lambda: abs(plant.velocity) < 1e-6 and abs(plant.omega) < 1e-6,
                     3, 'Navigation did not emit a stop')
                time.sleep(0.3)
                stopped_at, stopped_pose = time.monotonic(), plant.pose()
                time.sleep(0.7)
                stopped_commands = plant.commands_since(stopped_at)
                stopped = (all(max(abs(v) for v in c[1:]) < 1e-6 for c in stopped_commands)
                           and math.hypot(plant.pose()[0] - stopped_pose[0],
                                          plant.pose()[1] - stopped_pose[1]) < 0.01)
                samples = plant.commands_since(since)
                require(samples, 'No raw Twist samples')
                min_vx = (-active['max_vel_x_backwards'] if args.planner == 'teb'
                          else active['min_vel_x'])
                limits = (all(min_vx - 1e-6 <= c[1] <= active['max_vel_x'] + 1e-6 and
                              abs(c[6]) <= active['max_vel_theta'] + 1e-6
                              for c in samples))
                planar = all(max(abs(v) for v in c[2:6]) < 1e-9 for c in samples)
                moving = any(abs(c[1]) > 0.001 or abs(c[6]) > 0.001 for c in samples)
                negatives = [c for c in samples if c[1] < 0]
                functional = (terminal and state == GoalStatus.SUCCEEDED and
                              position_error <= active['xy_goal_tolerance'] + 0.02 and
                              yaw_error <= active['yaw_goal_tolerance'] + 0.02
                              and moving and limits and planar and stopped)
                case.update(functional_pass=functional, raw_limits_ok=limits,
                            differential_twist_ok=planar, stop_after_action_ok=stopped,
                            raw_count=len(samples), negative_raw_count=len(negatives),
                            minimum_raw_vx=min(c[1] for c in samples),
                            maximum_abs_raw_wz=max(abs(c[6]) for c in samples),
                            guard_forward_compatible=(not negatives),
                            passed=(functional and not negatives))
                (case_output / 'raw_commands.json').write_text(json.dumps(samples) + '\n')
                poses = plant.poses_since(since)[::5]
                case['trajectory_metrics'] = trajectory_metrics(poses, samples)
                (case_output / 'poses.json').write_text(json.dumps(poses) + '\n')
            except Exception as error:
                case['error'] = str(error)
                with (case_output / 'errors.log').open('w') as error_log:
                    traceback.print_exc(file=error_log)
                if since is not None:
                    samples = plant.commands_since(since)
                    case['negative_raw_count'] = sum(c[1] < 0 for c in samples)
                    (case_output / 'raw_commands.json').write_text(json.dumps(samples) + '\n')
            finally:
                if client is not None:
                    if client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE,
                                              GoalStatus.PREEMPTING, GoalStatus.RECALLING):
                        client.cancel_goal()
                    client.stop_tracking_goal()
                stop_process(launch)
                launch = client = None
                for subscriber in subscribers:
                    subscriber.unregister()
                with observation_lock:
                    case['initial_global_plans'] = list(plans)
                plant.close()
                plant = None
                # Remove private navigation parameters before the fresh launch;
                # each case must start from its actual production config.
                if rospy.has_param('/ugv'):
                    rospy.delete_param('/ugv')
                case['total_case_elapsed_seconds'] = time.monotonic() - case_started
            save()
            print('TEST_ONLY ' + json.dumps(case, allow_nan=False), flush=True)
        result['passed'] = all(case['passed'] for case in result['cases'])
    except Exception as error:
        result['error'] = str(error)
        with (output / 'errors.log').open('w') as log:
            traceback.print_exc(file=log)
    finally:
        stop_process(launch)
        if plant is not None:
            plant.close()
        stop_process(core)
        for log in logs:
            log.close()
        result['elapsed_seconds'] = round(time.monotonic() - started, 3)
        save()
    print('TEST_ONLY result={} passed={}'.format(output / 'result.json', result['passed']), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
