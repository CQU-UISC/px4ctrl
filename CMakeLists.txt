cmake_minimum_required(VERSION 3.0)
project(px4ctrl_lux)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


add_subdirectory(modules/yaml-cpp)
add_subdirectory(modules/FTXUI)

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED) 

# Build on ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  mavros
  # quadrotor_msgs
  #   uav_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package()

add_executable(px4ctrl_node 
  src/mock.cpp
  # src/px4ctrl_node.cpp
  # src/fsm/px4ctrl_fsm.cpp
  # src/fsm/px4ctrl_mavros.cpp
  # src/fsm/px4ctrl_se3_controller.cpp
)

target_link_libraries(px4ctrl_node
  PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  PRIVATE ${catkin_LIBRARIES}
  PRIVATE yaml-cpp::yaml-cpp
  PRIVATE ftxui::screen
  PRIVATE ftxui::dom
  PRIVATE ftxui::component # Not needed for this example.
)
