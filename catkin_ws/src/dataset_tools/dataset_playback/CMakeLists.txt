cmake_minimum_required(VERSION 2.8.3)
project(dataset_playback)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(DISTRO $ENV{ROS_DISTRO})
message(STATUS "Distribution: " ${DISTRO})

#set(DEFAULT_BUILD_QT5 OFF)
#if(${DISTRO} STREQUAL "kinetic")
#    set(DEFAULT_BUILD_QT5 ON)
#endif()
set(DEFAULT_BUILD_QT5 ON)

#option(UseQt5 "Build w/ QT version 5" ${DEFAULT_BUILD_QT5})
set(UseQt5 True)

if (UseQt5)
	#find_package(Qt5 COMPONENTS Core Gui Network Concurrent REQUIRED)
	find_package(Qt5Widgets)
	find_package(Qt5Core)
	find_package(Qt5Gui)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
	include(${QT_USE_FILE})
endif()

# package currently needs opencv
find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  h264_bag_playback
  dataset_tools
  dataset_msgs
)

catkin_package(CATKIN_DEPENDS dataset_tools)


set(${PROJECT_NAME}_SOURCES
  src/dataset_panel.cpp
  src/main.cpp
  src/treeitem.cpp
  src/treemodel.cpp
)

set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/dataset_panel.h
  include/${PROJECT_NAME}/treeitem.h
  include/${PROJECT_NAME}/treemodel.h
)

# invoke MOC and UI/ include Qt headers/ link Qt libraries
if (UseQt5)
	qt5_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})
	include_directories(
		${Qt5Widgets_INCLUDE_DIRS}
		${Qt5Core_INCLUDE_DIRS}
		${Qt5Gui_INCLUDE_DIRS}
                ${Qt5Xml_INCLUDE_DIRS}
                ${Qt5Network_INCLUDE_DIRS}
		${Qt5Concurrent_INCLUDE_DIRS}
		)
	link_libraries(
		${Qt5Core_LIBRARIES}
		${Qt5Widgets_LIBRARIES}
		${Qt5Gui_LIBRARIES}
                ${Qt5Xml_LIBRARIES}
                ${Qt5Svg_LIBRARIES}
                ${Qt5Network_LIBRARIES}
		${Qt5Concurrent_LIBRARIES}
		)
else()
	qt4_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})
	include_directories(${Qt4_INCLUDE_DIR})
	link_libraries(
  		${QT_LIBRARIES}
		)
endif()

# Other includes
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${QT_LIBRARIES}
  include/${PROJECT_NAME}
)

#LINK_DIRECTORIES( ${LINK_DIRECTORIES} )

# Other libraries
link_libraries(
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_definitions("-Wall -Wunused -std=c++11")

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOCSrcs}
)

add_executable(${PROJECT_NAME}
  ${PROJECT_SOURCE_FILES}
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(DIRECTORY destination_files
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
