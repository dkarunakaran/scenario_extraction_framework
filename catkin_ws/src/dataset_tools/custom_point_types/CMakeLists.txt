cmake_minimum_required(VERSION 2.8.3)
project(custom_point_types)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


#find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS pcl_ros
  INCLUDE_DIRS include
)

include_directories(
  include/${PROJECT_NAME}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
