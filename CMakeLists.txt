cmake_minimum_required(VERSION 3.1)

project(remote_video_recorder)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} does not support C++0x or C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  image_transport
  cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_python_setup()

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
    ${PROJECT_NAME}
    src/remote_video_recorder.cpp
)
target_link_libraries(
    remote_video_recorder
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
