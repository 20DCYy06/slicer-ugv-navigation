#!/usr/bin/env python3
"""唯一/cmd_vel出口；显式手动接管锁存，执行器确认健康/重规划后归还。"""
import json
import threading
import time
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from control_core import DEFAULT_CONFIG, load_config
from slicer_manual import ManualMux, INTENT_TOPIC, STATE_TOPIC, RELEASE_OWNER


def main():
    rospy.init_node('slicer_velocity_mux')
    c=load_config(rospy.get_param('~config_file',str(DEFAULT_CONFIG)))
    lock=threading.Lock()
    policy=ManualMux(c['command_timeout_s'],
        release_owner=rospy.get_param('~manual_release_owner',RELEASE_OWNER))
    pub=rospy.Publisher(c['output_topic'],Twist,queue_size=1)
    state_pub=rospy.Publisher(rospy.get_param('~manual_state_topic',STATE_TOPIC),String,queue_size=10,latch=False)
    def receive(msg,name):
        with lock:policy.receive(name,(msg.linear.x,msg.angular.z),time.monotonic())
    def intent(msg):
        header=getattr(msg,'_connection_header',{}) or {}
        with lock:
            before=(policy.generation,policy.held)
            state=policy.intent(msg.data,header.get('callerid',''),
                header.get('latching','0')!='0',time.monotonic())
            if before!=(policy.generation,policy.held):pub.publish(Twist())
            state_pub.publish(String(data=json.dumps(state,sort_keys=True,allow_nan=False)))
    def emergency(msg):
        with lock:
            policy.emergency(msg.data)
            if msg.data:pub.publish(Twist())
            state_pub.publish(String(data=json.dumps(policy.state(time.monotonic()),sort_keys=True,allow_nan=False)))
    subs=[rospy.Subscriber(c[name+'_topic'],Twist,receive,callback_args=name,queue_size=1)
          for name in ('teleop','mission','nav')]
    subs += [rospy.Subscriber(rospy.get_param('~manual_intent_topic',INTENT_TOPIC),String,intent,queue_size=10),
        rospy.Subscriber(rospy.get_param('~emergency_stop_topic','/cup_nav/emergency_stop'),Bool,emergency,queue_size=1)]
    try:
        while not rospy.is_shutdown():
            now=time.monotonic();out=Twist()
            with lock:
                source,(out.linear.x,out.angular.z)=policy.select(now)
                state=policy.state(now)
                pub.publish(out)
                state_pub.publish(String(data=json.dumps(state,sort_keys=True,allow_nan=False)))
            time.sleep(1/c['publish_hz'])
    finally:
        try:pub.publish(Twist())
        except rospy.ROSException:pass


if __name__=='__main__':main()
