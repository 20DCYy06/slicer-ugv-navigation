#!/usr/bin/env python3
"""TEST_ONLY: run only in a no-device, --network none Noetic container."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client


def main():
    if not Path('/.dockerenv').exists() or {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}:
        raise RuntimeError('TEST_ONLY fixture requires a --network none container')
    output = Path(sys.argv[1]); output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:12416'
    os.environ['ROS_IP'] = '127.0.0.1'
    core_log = (output/'master.log').open('w')
    core = subprocess.Popen(['roscore', '-p', '12416'], stdout=core_log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    stop = threading.Event(); thread = child = None
    result = dict(TEST_ONLY=True, no_hardware=True, passed=False)
    try:
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic()+10
        while True:
            try:
                master.getPid('/TEST_ONLY_collector_check'); break
            except OSError:
                if time.monotonic() > deadline: raise
                time.sleep(.1)
        import rospy
        import tf2_ros
        from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
        from sensor_msgs.msg import LaserScan, PointCloud2
        from sensor_msgs.point_cloud2 import create_cloud_xyz32
        from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
        from std_msgs.msg import Bool, Header
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        import rosbag
        rospy.init_node('TEST_ONLY_collector_sources', disable_signals=True)
        rospy.set_param('/slicer_c30d_driver/provenance', {'control_mode': 'wheel_speed', 'TEST_ONLY': True})
        rospy.set_param('/slicer_c30d_driver/control_mode', '')
        rospy.set_param('/slicer_ekf', {'odom0': '/lio/odom', 'odom1': '/wheel/odom',
                                     'password': 'TEST_ONLY_DO_NOT_EXPORT'})
        rospy.set_param('/slicer_cloud_to_scan', {'min_height': .04, 'password': 'TEST_ONLY_DO_NOT_EXPORT'})
        rospy.set_param('/preprocess', {'blind': .08})
        pubs = {topic: rospy.Publisher(topic, Odometry, queue_size=5) for topic in ('/odom','/lio/odom','/wheel/odom','/Odometry')}
        scan = rospy.Publisher('/scan', LaserScan, queue_size=5)
        cloud = rospy.Publisher('/cloud_registered_body', PointCloud2, queue_size=5)
        plan = rospy.Publisher('/ugv/move_base/DWAPlannerROS/local_plan', RosPath, queue_size=5)
        costmap = rospy.Publisher('/ugv/move_base/local_costmap/costmap', OccupancyGrid, queue_size=5)
        ready = rospy.Publisher('/slicer_c30d_driver/ready', Bool, queue_size=1, latch=True)
        diag = rospy.Publisher('/slicer_c30d_driver/diagnostics', DiagnosticArray, queue_size=10)
        twists = [rospy.Publisher(topic, Twist, queue_size=5) for topic in ('/cmd_vel/nav_raw','/cmd_vel/nav','/cmd_vel')]
        dynamic = tf2_ros.TransformBroadcaster(); static = tf2_ros.StaticTransformBroadcaster()
        transform = TransformStamped(); transform.header.stamp = rospy.Time.now()
        transform.header.frame_id='base_link'; transform.child_frame_id='laser_link'; transform.transform.rotation.w=1
        static.sendTransform(transform)
        def publish():
            while not stop.is_set():
                now=rospy.Time.now()
                for topic,pub in pubs.items():
                    msg=Odometry();msg.header.stamp=now;msg.header.frame_id='odom';msg.child_frame_id='base_link'
                    msg.pose.pose.orientation.w=1;msg.pose.pose.position.x=100 if topic=='/lio/odom' else 0
                    pub.publish(msg)
                msg=LaserScan();msg.header.stamp=now;msg.header.frame_id='laser_link'
                msg.angle_min=-math.pi;msg.angle_max=math.pi;msg.angle_increment=math.pi/10
                msg.range_min=.05;msg.range_max=10;msg.ranges=[float('inf')]*20;scan.publish(msg)
                cloud.publish(create_cloud_xyz32(Header(stamp=now, frame_id='base_link'), [(1., .1, .05)]))
                goal=PoseStamped();goal.header.stamp=now;goal.header.frame_id='odom';goal.pose.orientation.w=1
                goal.pose.position.x=1
                path=RosPath();path.header=goal.header;path.poses=[goal];plan.publish(path)
                grid=OccupancyGrid();grid.header=goal.header;grid.info.resolution=.05
                grid.info.width=grid.info.height=2;grid.info.origin.orientation.w=1;grid.data=[0]*4;costmap.publish(grid)
                transform=TransformStamped();transform.header.stamp=now;transform.header.frame_id='odom'
                transform.child_frame_id='base_link';transform.transform.rotation.w=1;dynamic.sendTransform(transform)
                ready.publish(Bool(False))
                for pub in twists:pub.publish(Twist())
                msg=DiagnosticArray();msg.header.stamp=now
                status=DiagnosticStatus();status.message='serial_tx';status.name='TEST_ONLY_feedback'
                status.values=[KeyValue('kind','16'),KeyValue('speed_mm_s_abcd','[0,0,0,0]')]
                msg.status=[status];diag.publish(msg)
                stop.wait(.05)
        thread=threading.Thread(target=publish);thread.start();time.sleep(.3)
        summaries=[]
        for name,duration,interrupt in [('complete',4,False),('interrupt',30,True)]:
            log=(output/(name+'.log')).open('w');directory=output/name
            child=subprocess.Popen([sys.executable,'tools/collect_odom_test.py','--navigation','--perception',
                                    '--duration',str(duration),'--output',str(directory)],
                                   stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+12
            while 'RECORDING STARTED:' not in (output/(name+'.log')).read_text():
                if child.poll() is not None or time.monotonic()>deadline:
                    raise AssertionError('recorder did not start: '+(output/(name+'.log')).read_text())
                time.sleep(.1)
            state=master.getSystemState('/TEST_ONLY_check')[2]
            assert all(not any('ugv_readonly_capture' in node or 'ugv_readonly_bag' in node for node in nodes)
                       for topic,nodes in state[0] if topic in ('/cmd_vel','/cmd_vel/nav','/cmd_vel/nav_raw'))
            assert not any(topic=='/cup_nav/manual_intent' and any('ugv_readonly' in node for node in nodes)
                           for topic,nodes in state[1])
            if interrupt:
                time.sleep(.8);child.send_signal(signal.SIGINT)
            rc=child.wait(timeout=15);log.close();assert rc==0,(rc,(output/(name+'.log')).read_text())
            data=json.loads((directory/'manifest.json').read_text())
            assert data['bag_finalized'] and data['recorder_subscriptions_confirmed']
            assert data['driver_reported_control_mode']=='wheel_speed' and data['driver_requested_control_mode']==''
            assert data['before']['parameters']['/slicer_ekf']['password']=='[redacted]'
            for snapshot in ('before', 'after'):
                assert data[snapshot]['parameters']['/slicer_cloud_to_scan']=={'min_height': .04, 'password': '[redacted]'}
                assert data[snapshot]['parameters']['/preprocess']=={'blind': .08}
            assert data['topics']['/lio/odom']['count']>=5
            assert data['tf_lookup']['odom <- base_link']['available']
            assert data['tf_lookup']['base_link <- laser_link']['available']
            assert not data['decode_errors'],data['decode_errors']
            assert data['tf_edges']['odom -> base_link']['count']>=5
            assert data['tf_edges']['static: base_link -> laser_link']['count']>=1
            assert data['observed_publishers']['odom -> base_link']
            assert data['interrupted']==interrupt
            with rosbag.Bag(str(directory/'motion.bag')) as bag:
                bag_topics=set(bag.get_type_and_topic_info().topics)
                assert {'/odom','/lio/odom','/wheel/odom','/tf','/tf_static'}<=bag_topics
                assert {'/cloud_registered_body', '/ugv/move_base/DWAPlannerROS/local_plan',
                        '/ugv/move_base/local_costmap/costmap'}<=bag_topics
                messages={topic: msg for topic,msg,_ in bag.read_messages(topics=[
                    '/cloud_registered_body', '/ugv/move_base/DWAPlannerROS/local_plan',
                    '/ugv/move_base/local_costmap/costmap'])}
                assert messages['/cloud_registered_body'].width==1
                assert messages['/ugv/move_base/DWAPlannerROS/local_plan'].poses[0].pose.position.x==1
                assert list(messages['/ugv/move_base/local_costmap/costmap'].data)==[0]*4
                assert '/cup_nav/manual_intent' not in bag_topics
            summaries.append({'case':name,'passed':True,'bag_topics':sorted(bag_topics),
                              'lio_samples':data['topics']['/lio/odom']['count']})
        result.update(passed=True,cases=summaries)
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT);child.wait(timeout=10)
        stop.set()
        if thread is not None:thread.join(2)
        os.killpg(core.pid,signal.SIGINT)
        try:core.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(core.pid,signal.SIGKILL);core.wait()
        core_log.close();(output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':
    raise SystemExit(main())
