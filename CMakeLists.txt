cmake_minimum_required(VERSION 2.8.3)
project(rogata_engine)

catkin_python_setup()
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation)

add_service_files(
  FILES
  SetPos.srv
  GetPos.srv
  RequestDist.srv
  RequestInter.srv
  CheckInside.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

