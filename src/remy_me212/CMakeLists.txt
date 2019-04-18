cmake_minimum_required(VERSION 2.8.3)
project(remy_me212)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES remy_me212
#  CATKIN_DEPENDS geomestry_msgs math numpy odrive odrive_ros rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS 
  scripts/inverse_kinematic_odrive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
