cmake_minimum_required(VERSION 3.0)
project(px4ctrl_lux)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

option(BUILD_CLIENT_ONLY "Build client only" ON)

find_package(spdlog REQUIRED)
find_package(cppzmq REQUIRED)
# Opengl
if(APPLE)
  find_package(OpenGL REQUIRED)
  find_library(COCOA_LIBRARY Cocoa REQUIRED)
  find_library(IOKIT_LIBRARY IOKit REQUIRED)
  find_library(COREVIDEO_LIBRARY CoreVideo REQUIRED)
  find_library(GLFW_LIBRARY glfw REQUIRED)
  include_directories(/usr/local/include)
elseif(UNIX)
  find_package(OpenGL REQUIRED)
  find_package(glfw3 REQUIRED)
endif()


include_directories(serial/include)
add_subdirectory("serial")

# Com
add_library(px4ctrl_com STATIC 
src/com/px4ctrl_gcs_zmq.cpp
src/com/crc.cpp
src/com/zmq_proxy.cpp
src/com/serial.cpp
)

target_link_libraries(px4ctrl_com 
PUBLIC cppzmq
PUBLIC serial
PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
)

# Imgui
set(IMGUI_DIR imgui)

include_directories(
  include
  ${IMGUI_DIR} 
  ${IMGUI_DIR}/backends
  serial/include
)

add_library(imgui
${IMGUI_DIR}/backends/imgui_impl_glfw.cpp 
${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp 
${IMGUI_DIR}/imgui.cpp 
${IMGUI_DIR}/imgui_draw.cpp 
${IMGUI_DIR}/imgui_demo.cpp 
${IMGUI_DIR}/imgui_tables.cpp 
${IMGUI_DIR}/imgui_widgets.cpp
)

if(APPLE)
  target_link_libraries(imgui
  PUBLIC ${GLFW_LIBRARY}
  PUBLIC ${OPENGL_LIBRARIES}
  PUBLIC ${COCOA_LIBRARY}
  PUBLIC ${IOKIT_LIBRARY}
  PUBLIC ${COREVIDEO_LIBRARY}
)
elseif(UNIX)
  target_link_libraries(imgui 
  PUBLIC ${OPENGL_gl_LIBRARY} 
  PUBLIC glfw
  )
endif()

# zmq proxy
add_executable(com_node
src/com_node.cpp
)

target_link_libraries(com_node
PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
PUBLIC px4ctrl_com
)


# Build on ros
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

  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    include/${PROJECT_NAME}
  )

  catkin_package(
  )

  add_executable(px4ctrl_node 
    src/px4ctrl_node.cpp
    src/fsm/px4ctrl_fsm.cpp
    src/fsm/px4ctrl_mavros.cpp
    src/fsm/px4ctrl_se3_controller.cpp
  )


  target_link_libraries(px4ctrl_node
    PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
    PUBLIC ${catkin_LIBRARIES}
    PUBLIC px4ctrl_com
  )

  catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()


if(BUILD_CLIENT_ONLY)
  add_executable(px4ctrl_gcs_client
    src/px4ctrl_gcs_client.cpp
  )
  target_link_libraries(px4ctrl_gcs_client
    PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
    PUBLIC px4ctrl_com
    PUBLIC imgui
  )
endif()

