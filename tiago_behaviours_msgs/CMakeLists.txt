cmake_minimum_required(VERSION 2.8.3)
project(tiago_behaviours_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
  people_msgs
  geometry_msgs
  message_generation
)

add_action_files(
  DIRECTORY
    action
  FILES
    Wander.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    people_msgs
    geometry_msgs
    std_msgs
)

catkin_package(

)


