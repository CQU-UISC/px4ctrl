cmake_minimum_required(VERSION 3.0)
project(px4ctrl_lux)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
option(BUILD_TESTS "Build tests" OFF)

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
  message_generation
)


add_message_files(
  FILES
  Command.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/fsm/px4ctrl_controller.cpp
  src/fsm/px4ctrl_bridge.cpp
  src/fsm/px4ctrl_fsm.cpp
)

target_link_libraries(px4ctrl_node
  PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  PRIVATE ${catkin_LIBRARIES}
  PRIVATE Eigen3::Eigen
  PRIVATE yaml-cpp::yaml-cpp
  PRIVATE ftxui::screen
  PRIVATE ftxui::dom
  PRIVATE ftxui::component
)

if(BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()

  add_executable(px4ctrl_tests
    tests/test_def.cpp
    tests/test_params.cpp
  )

  target_link_libraries(px4ctrl_tests
    PRIVATE GTest::GTest GTest::Main
    PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
    PRIVATE ${catkin_LIBRARIES}
    PRIVATE Eigen3::Eigen
    PRIVATE yaml-cpp::yaml-cpp
    PRIVATE ftxui::screen
    PRIVATE ftxui::dom
    PRIVATE ftxui::component
  )

  include(GoogleTest)
  gtest_discover_tests(px4ctrl_tests)
endif()