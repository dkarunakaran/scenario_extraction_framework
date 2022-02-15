cmake_minimum_required(VERSION 2.8.3)
project(dataset_msgs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


#find catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  sensor_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
        FILES
        DatasetEvent.msg
        Observation.msg
        Innovation.msg
        LocaliserStats.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs sensor_msgs std_msgs sensor_msgs message_runtime
)


include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)


# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} 
)


