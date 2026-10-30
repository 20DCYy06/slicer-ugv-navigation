cmake_minimum_required(VERSION 3.0.2)
project(slicer_navigation)
find_package(catkin REQUIRED COMPONENTS rospy roscpp rviz pluginlib geometry_msgs nav_msgs sensor_msgs visualization_msgs std_msgs std_srvs tf tf2 tf2_geometry_msgs actionlib actionlib_msgs move_base_msgs nav_core global_planner costmap_2d base_local_planner)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
catkin_package()
add_library(slicer_bspline_global_planner src/bspline_global_planner.cpp)
target_compile_features(slicer_bspline_global_planner PRIVATE cxx_std_14)
target_include_directories(slicer_bspline_global_planner PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(slicer_bspline_global_planner ${catkin_LIBRARIES})
# This small executable also runs under $NAV check without a ROS master or hardware.
add_executable(slicer_bspline_check tests/check_bspline.cpp)
target_compile_features(slicer_bspline_check PRIVATE cxx_std_14)
target_include_directories(slicer_bspline_check PRIVATE include)
install(TARGETS slicer_bspline_global_planner slicer_bspline_check
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES global_planner_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
add_library(slicer_record_waypoint_tool src/record_waypoint_tool.cpp)
target_compile_features(slicer_record_waypoint_tool PRIVATE cxx_std_11)
target_include_directories(slicer_record_waypoint_tool PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(slicer_record_waypoint_tool ${catkin_LIBRARIES} Qt5::Widgets)
install(TARGETS slicer_record_waypoint_tool LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/amcl_ready.py scripts/record_ab.py scripts/self_scan_filter.py scripts/arm_cloud_filter.py
  scripts/waypoint_recorder.py scripts/uav_waypoint_receiver.py scripts/multi_goal_patrol.py scripts/arc_velocity_guard.py
  scripts/navigation_core.py scripts/wasd_panel.py scripts/runtime_preflight.py scripts/map_store.py
  scripts/navigation_visuals.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/diagnose_stall.py scripts/measure_patrol.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/save_map.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config rviz maps docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
