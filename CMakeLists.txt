cmake_minimum_required(VERSION 3.0)
project(px4ctrl_lux)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
option(BUILD_TESTS "Build tests" OFF)

add_subdirectory(modules/yaml-cpp)
add_subdirectory(modules/libzmq)
option(CPPZMQ_BUILD_TESTS "" OFF)
add_subdirectory(modules/cppzmq)
add_subdirectory(modules/spdlog)
# find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED) 

# Build on ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
  px4msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs nav_msgs mavros_msgs px4msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(px4ctrl_node
  src/node.cpp
  src/controller.cpp
  src/bridge.cpp
  src/fsm.cpp
  src/server.cpp
)

target_link_libraries(px4ctrl_node
  PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  PRIVATE ${catkin_LIBRARIES}
  PRIVATE Eigen3::Eigen
  PRIVATE yaml-cpp::yaml-cpp
  PRIVATE cppzmq
)

add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS})
if(BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()

  add_executable(px4ctrl_tests
    tests/def.cpp
    tests/params.cpp
  )

  target_link_libraries(px4ctrl_tests
    PRIVATE GTest::GTest GTest::Main
    PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
    PRIVATE ${catkin_LIBRARIES}
    PRIVATE Eigen3::Eigen
    PRIVATE yaml-cpp::yaml-cpp
    PRIVATE cppzmq
  )

  include(GoogleTest)
  gtest_discover_tests(px4ctrl_tests)
endif()