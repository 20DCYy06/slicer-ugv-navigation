cmake_minimum_required(VERSION 3.0.2)
project(ugv_slicer_control)
find_package(catkin REQUIRED)
catkin_package()
# Keep entry points beside the exact modules selected by their local sys.path.
catkin_install_python(PROGRAMS scripts/velocity_mux.py scripts/arc_velocity_guard.py scripts/control.py
  scripts/c30d_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/control_core.py scripts/slicer_manual.py scripts/navigation_core.py
  scripts/c30d_link.py scripts/chassis_core.py scripts/serial_transport.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# catkin's devel entry-point wrappers must find the same sibling modules.
foreach(module control_core slicer_manual navigation_core c30d_link chassis_core serial_transport)
  configure_file(scripts/${module}.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${module}.py COPYONLY)
endforeach()
install(FILES SOURCES.json DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_control.py)
  catkin_add_nosetests(test/test_driver.py)
  catkin_add_nosetests(test/test_body_config.py)
  catkin_add_nosetests(test/test_body_link.py)
  catkin_add_nosetests(test/test_navigation_guard.py)
  add_rostest(test/control.test)
  add_rostest(test/guard.test)
endif()
