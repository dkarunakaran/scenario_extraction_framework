cmake_minimum_required(VERSION 3.1)
project(ibeo_object_msg)


find_package(catkin REQUIRED COMPONENTS
  message_runtime  
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  shape_msgs
  message_generation
)

add_message_files(
  FILES
  IbeoObject.msg
)


generate_messages(
  DEPENDENCIES
  nav_msgs
  geometry_msgs
  sensor_msgs
  shape_msgs
  std_msgs
)


catkin_package(
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_generation message_runtime nav_msgs geometry_msgs sensor_msgs shape_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_dependencies(${PROJECT_NAME}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)

