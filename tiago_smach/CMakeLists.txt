cmake_minimum_required(VERSION 2.8.3)
project(tiago_smach)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  move_base_msgs
  play_motion_msgs
  rosplan_tiago_core_msgs
  rospy
  tf
  message_generation
)

catkin_python_setup()


catkin_package(
    CATKIN_DEPENDS
    rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

#install(PROGRAMS
#        nodes/rosplan_sys_control
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )