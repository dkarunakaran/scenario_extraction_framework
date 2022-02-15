cmake_minimum_required(VERSION 3.1)
project(lane_points_msg)


find_package(catkin REQUIRED COMPONENTS
  message_runtime  
  std_msgs
  message_generation
)

add_message_files(
  FILES
  LanePoints.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_generation message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_dependencies(${PROJECT_NAME}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)

