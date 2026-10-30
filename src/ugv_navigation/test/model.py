#!/usr/bin/env python3
"""TEST_ONLY manual demo plant: no goals, drivers, watchdog or fake results."""
import rospy
from test_odom_navigation import TestOnlyDifferentialPlant


if __name__ == '__main__':
    rospy.init_node('TEST_ONLY_kinematic_model')
    rospy.logwarn('TEST_ONLY ideal kinematics; run only without devices on isolated master')
    plant = TestOnlyDifferentialPlant()
    rospy.on_shutdown(plant.close)
    rospy.spin()
