cmake_minimum_required(VERSION 2.8.3)
project(rogata_engine)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation)

add_service_files(
  FILES
  RequestPos.srv
  RequestDist.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

