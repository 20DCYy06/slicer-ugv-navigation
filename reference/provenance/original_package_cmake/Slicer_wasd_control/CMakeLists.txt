cmake_minimum_required(VERSION 3.0.2)
project(slicer_wasd_control)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs tf2_ros)
catkin_package()
catkin_install_python(PROGRAMS scripts/wasd_teleop_node.py scripts/c30d_driver_node.py scripts/velocity_mux.py scripts/gamepad_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/control_core.py scripts/c30d_link.py scripts/serial_transport.py scripts/chassis_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
