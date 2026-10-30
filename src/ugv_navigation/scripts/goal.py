#!/usr/bin/env python3
"""Direct move_base Action client; no task database or automatic resubmission."""
import argparse
import math
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan, GetPlanRequest
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


def finite(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError('must be finite')
    return result


def positive(value):
    result = finite(value)
    if result <= 0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return result


def frame(value):
    if not value or value.startswith('/') or any(c.isspace() for c in value):
        raise argparse.ArgumentTypeError('use a nonempty TF frame without leading slash or spaces')
    return value


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', default='/ugv/move_base', help='MoveBaseAction name')
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('send', 'plan'):
        target = sub.add_parser(command, help=('send one goal; Ctrl-C cancels it' if command == 'send'
                                              else 'call GetPlan only; never submit an Action goal'))
        target.add_argument('--x', required=True, type=finite)
        target.add_argument('--y', required=True, type=finite)
        target.add_argument('--yaw', default=0.0, type=finite, help='radians')
        target.add_argument('--frame', default='odom', type=frame, help='fixed target frame')
        target.add_argument('--relative', action='store_true', help='sample base pose once and fix target in --frame')
        target.add_argument('--base-frame', default='base_link', type=frame)
        target.add_argument('--tf-max-age', default=0.5, type=positive, help='seconds; existing Slicer Guard TF limit')
        target.add_argument('--timeout', default=120.0, type=positive, help='seconds; send expiry cancels this goal')
    sub.add_parser('cancel', help='explicitly cancel ALL goals on this action server')
    args = parser.parse_args(argv)
    if getattr(args, 'relative', False) and args.frame == args.base_frame:
        parser.error('relative goal needs a fixed target frame different from base frame')
    return args


def goal_pose(args):
    pose = PoseStamped()
    pose.header.frame_id = args.frame
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    return pose


def relative_goal_pose(args, transform):
    relative = goal_pose(args)
    relative.header.frame_id = args.base_frame
    return do_transform_pose(relative, transform)


def current_transform(args, buffer):
    transform = buffer.lookup_transform(args.frame, args.base_frame, rospy.Time(0), rospy.Duration(3.0))
    age = (rospy.Time.now() - transform.header.stamp).to_sec()
    if abs(age) > args.tf_max_age:
        raise ValueError('base TF age {:.3f}s exceeds {:.3f}s'.format(age, args.tf_max_age))
    return transform


def main():
    args = arguments(rospy.myargv()[1:])
    rospy.init_node('ugv_goal_client', anonymous=True, disable_signals=True)
    client = None
    try:
        transform = None
        if args.command == 'plan' or (args.command == 'send' and args.relative):
            buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buffer)
            transform = current_transform(args, buffer)
        if args.command in ('send', 'plan'):
            pose = relative_goal_pose(args, transform) if args.relative else goal_pose(args)
            pose.header.stamp = rospy.Time.now()
            q = pose.pose.orientation
            print('Fixed target {}: x={:.3f} y={:.3f} q=({:.4f},{:.4f},{:.4f},{:.4f})'.format(
                pose.header.frame_id, pose.pose.position.x, pose.pose.position.y, q.x, q.y, q.z, q.w), flush=True)
        if args.command == 'plan':
            start = PoseStamped()
            start.header = pose.header
            start.pose.position.x = transform.transform.translation.x
            start.pose.position.y = transform.transform.translation.y
            start.pose.position.z = transform.transform.translation.z
            start.pose.orientation = transform.transform.rotation
            service = rospy.resolve_name(args.action).rstrip('/') + '/make_plan'
            rospy.wait_for_service(service, timeout=5.0)
            response = rospy.ServiceProxy(service, GetPlan)(GetPlanRequest(start=start, goal=pose, tolerance=0.0))
            print('GetPlan only: {} poses; no Action goal sent'.format(len(response.plan.poses)))
            return 0 if response.plan.poses else 1
        client = actionlib.SimpleActionClient(args.action, MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(10.0)):
            print('Action server unavailable: ' + args.action, file=sys.stderr)
            return 1
        if args.command == 'cancel':
            client.cancel_all_goals()
            rospy.sleep(0.3)
            print('Cancel ALL requested; verify terminal state on ' + args.action + '/status')
            return 0
        client.send_goal(MoveBaseGoal(target_pose=pose))
        print('Goal sent to {} in {}; Ctrl-C cancels this goal'.format(args.action, args.frame), flush=True)
        if not client.wait_for_result(rospy.Duration(args.timeout)):
            client.cancel_goal()
            acknowledged = client.wait_for_result(rospy.Duration(3.0))
            print('Timeout; cancel sent, terminal result received={}'.format(acknowledged), file=sys.stderr)
            return 1
        state = client.get_state()
        print('state={} {}'.format(state, client.get_goal_status_text()))
        return 0 if state == GoalStatus.SUCCEEDED else 1
    except KeyboardInterrupt:
        if args.command == 'send' and client is not None:
            client.cancel_goal()
            client.wait_for_result(rospy.Duration(3.0))
            print('Interrupted; goal cancellation requested', file=sys.stderr)
        return 130
    except (tf2_ros.TransformException, rospy.ROSException, rospy.ServiceException, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        rospy.signal_shutdown('client finished')


if __name__ == '__main__':
    sys.exit(main())
