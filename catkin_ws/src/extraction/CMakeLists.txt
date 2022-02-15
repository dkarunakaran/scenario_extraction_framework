cmake_minimum_required(VERSION 2.8.3)
project(extraction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  h264_bag_playback
  dataset_tools
  dataset_msgs
  sensor_msgs
  custom_point_types
  tf
  tf_conversions
  tf2_msgs
  ibeo_object_msg
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
)

# package currently needs opencv
find_package(OpenCV 3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/extraction.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})


#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
