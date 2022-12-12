cmake_minimum_required(VERSION 3.0.2)
project(robile_behaviors)

find_package(catkin REQUIRED 
COMPONENTS
  geometry_msgs
  nav_msgs
  py_trees
  py_trees_ros
  rospy
  tf
  sensor_msgs
  std_msgs
  rospy
)

catkin_python_setup()

install(DIRECTORY ros/launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)


catkin_install_python(PROGRAMS ros/scripts/wall_following ros/scripts/autonomous_mapping ros/scripts/marker_line ros/scripts/battery_voltage_publisher
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src/behavior_tree
)
