cmake_minimum_required(VERSION 3.1)
project(gmsl_frame_msg)

#set (CMAKE_CXX_STANDARD 11)

# FindDriveworks.cmake, ArchConfiguration.cmake, and LibFindMacros.cmake were needed for my setup they are taken from driveworks/samples/cmake/
# ArchConfiguration.cmake was the only file that needed small changes, remove the fatal error on line 17 and add the following lines in its place
#         set(VIBRANTE TRUE)
#         add_definitions(-DVIBRANTE)
# this is the path I placed the driveworks cmake files in
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  FrameInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

#link_directories(
#  ${Driveworks_LIBRARY})

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${Driveworks_INCLUDE_DIR}
#  ${CUDA_INCLUDE_DIRS}
#  ${CMAKE_CURRENT_SOURCE_DIR}/src
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/driveworks-src
#  ${GLEW_INCLUDE_DIR}
#)

# TODO: add a FindNvmedia.cmake file for this? Why does it not exist?
#include_directories(/usr/share/visionworks/sources/3rdparty/nvmedia/
#/usr/share/visionworks/VisionWorks-1.6-Samples/3rdparty/glfw3/include/
#)


# ros node name template
#set(NODE_NAME ${PROJECT_NAME}_node)

#add_executable(${NODE_NAME} src/main.cpp
#  src/driveworks-src/ProgramArguments.cpp
#  src/driveworks-src/ConsoleColor.cpp
#  src/driveworks-src/WindowEGL.cpp
#  src/driveworks-src/WindowGLFW.cpp  
#  src/driveworks-src/Grid.cpp
#  src/ROSInterface.cpp  
#)

#target_link_libraries(${NODE_NAME}
#  ${catkin_LIBRARIES}
#  nvmedia  
#  ${GLEW_LIBRARY}
#  ${Driveworks_LIBRARY}
#  #driveworks
#  GLESv2
#  EGL
#  drm
#  #glfw3
#  ${CUDA_LIBRARY}
#  ${OpenCV_LIBS} 
#)
