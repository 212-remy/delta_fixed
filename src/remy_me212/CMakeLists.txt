cmake_minimum_required(VERSION 2.8.3)
project(remy_me212)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  message_generation
)
add_message_files(
  FILES
  DeltaState.msg
  DeltaStateArray.msg
  Topping.msg
  ToppingArray.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES remy_me212
  CATKIN_DEPENDS geometry_msgs math numpy odrive odrive_ros rospy  message_runtime
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



catkin_install_python(PROGRAMS 
  scripts/inverse_kinematic_odrive.py
  scripts/trajectory.py
  scripts/color_thresh.py
  scripts/planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
