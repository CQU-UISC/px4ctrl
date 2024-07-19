cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl_lux)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

option(BUILD_CLIENT_ONLY "Build client only" OFF)

if(NOT BUILD_CLIENT_ONLY)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

find_package(cppzmq REQUIRED)


if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

catkin_package(
)

set(IMGUI_DIR imgui)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
  ${IMGUI_DIR} 
  ${IMGUI_DIR}/backends
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/px4ctrl_fsm.cpp
  src/px4ctrl_gcs_ros.cpp
  src/px4ctrl_gcs_zmq.cpp
  src/px4ctrl_mavros.cpp
  src/px4ctrl_se3_controller.cpp
)

add_executable(px4ctrl_gcs_client
  src/px4ctrl_gcs_client.cpp
  src/px4ctrl_gcs_ros.cpp
  src/px4ctrl_gcs_zmq.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp 
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp 
  ${IMGUI_DIR}/imgui.cpp 
  ${IMGUI_DIR}/imgui_draw.cpp 
  ${IMGUI_DIR}/imgui_demo.cpp 
  ${IMGUI_DIR}/imgui_tables.cpp 
  ${IMGUI_DIR}/imgui_widgets.cpp
)

add_dependencies(px4ctrl_node quadrotor_msgs)

target_link_libraries(px4ctrl_gcs_client
  PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  PUBLIC ${catkin_LIBRARIES}
  PUBLIC ${OPENGL_gl_LIBRARY} 
  PUBLIC glfw
  PUBLIC cppzmq
)

target_link_libraries(px4ctrl_node
  PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  PUBLIC ${catkin_LIBRARIES}
  PUBLIC cppzmq
)

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()


if(BUILD_CLIENT_ONLY)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

  #If macos
  if(APPLE)
    find_package(OpenGL REQUIRED)
    find_library(COCOA_LIBRARY Cocoa REQUIRED)
    find_library(IOKIT_LIBRARY IOKit REQUIRED)
    find_library(COREVIDEO_LIBRARY CoreVideo REQUIRED)
  endif()

  find_library(GLFW_LIBRARY glfw REQUIRED)
  find_package(spdlog REQUIRED)
  find_package(cppzmq REQUIRED)

  set(IMGUI_DIR imgui)

  include_directories(
  include
  ${IMGUI_DIR} 
  ${IMGUI_DIR}/backends
  )


  add_executable(px4ctrl_gcs_client
    src/px4ctrl_gcs_client.cpp
    src/px4ctrl_gcs_zmq.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp 
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp 
    ${IMGUI_DIR}/imgui.cpp 
    ${IMGUI_DIR}/imgui_draw.cpp 
    ${IMGUI_DIR}/imgui_demo.cpp 
    ${IMGUI_DIR}/imgui_tables.cpp 
    ${IMGUI_DIR}/imgui_widgets.cpp
  )

  target_link_libraries(px4ctrl_gcs_client
    PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
    PUBLIC ${GLFW_LIBRARY}
    PRIVATE ${OPENGL_LIBRARIES}
    PRIVATE ${COCOA_LIBRARY}
    PRIVATE ${IOKIT_LIBRARY}
    PRIVATE ${COREVIDEO_LIBRARY}
    PUBLIC cppzmq
  )

endif()