cmake_minimum_required(VERSION 2.8.3)
project(h264_bag_playback)

find_package(catkin REQUIRED COMPONENTS nodelet
	rosbag
	#rosbag_storage
	gmsl_frame_msg
	roscpp
	tf2_ros
	tf2_sensor_msgs
	image_transport
	dataset_msgs
	std_msgs
	cv_bridge
	nav_msgs
	sensor_msgs)

find_package(OpenCV REQUIRED)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS},
  include/${PROJECT_NAME}
  include)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS include
	CATKIN_DEPENDS nodelet
	roscpp
	rosbag
	rosbag_storage
	gmsl_frame_msg
	image_transport
	dataset_msgs
	std_msgs
	tf2_ros
	tf2_sensor_msgs
	nav_msgs
	sensor_msgs
	cv_bridge
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Create the nodelet tutorial library
add_executable(${PROJECT_NAME}_node
		src/h264_bag_playback.cpp
		src/video.cpp
		src/helper_functions.cpp)

target_link_libraries(${PROJECT_NAME}_node
		${catkin_LIBRARIES}
		${OpenCV_LIBS})

add_library(${PROJECT_NAME}
		src/h264_bag_playback.cpp
		src/video.cpp
		src/helper_functions.cpp
		src/bag_container.cpp)

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${OpenCV_LIBS})


if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME}
		  ${catkin_EXPORTED_LIBRARIES}
		  ${catkin_LIBRARIES}
		  ${OpenCV_LIBS})
endif()


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_definition.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})



## Add gtest based cpp test target and link libraries
get_filename_component(BAG_TEST_FOLDER "test/" ABSOLUTE)
configure_file(test/bag_data.h.in test/bag_data.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

catkin_add_gtest(${PROJECT_NAME}-test
		test/test_playback.cpp
		)

if(TARGET ${PROJECT_NAME}-test)
	target_link_libraries(${PROJECT_NAME}-test h264_bag_playback)
endif()
