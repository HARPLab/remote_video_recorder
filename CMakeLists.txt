cmake_minimum_required(VERSION 2.8.7)

project(remote_video_recorder)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  image_transport
  cv_bridge
)
find_package(OpenCV REQUIRED)

###############################################################################
# MESSAGES
add_service_files(FILES
    RemoteRecord.srv
)

generate_messages()


catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
   DEPENDS
    OpenCV
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(
    remote_video_recorder
    src/remote_video_recorder.cpp
)
    
target_link_libraries(
    remote_video_recorder
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
