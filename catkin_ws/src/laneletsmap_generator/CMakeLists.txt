cmake_minimum_required(VERSION 2.8.1)
project(laneletsmap_generator)

find_package(catkin REQUIRED COMPONENTS
        h264_bag_playback
        roscpp
        dataset_tools
        std_msgs
        pcl_ros
        tf
        tf_conversions
        custom_point_types
        lane_points_msg
        lanelet2_core
        lanelet2_io
        lanelet2_projection
        lanelet2_routing
        lanelet2_traffic_rules
        lanelet2_validation)

find_package(OpenCV REQUIRED)

## Setup include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs pcl_ros tf_conversions tf h264_bag_playback dataset_tools
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(${PROJECT_NAME}
    src/laneletsmap_generator.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
