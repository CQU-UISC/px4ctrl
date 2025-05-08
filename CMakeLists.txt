cmake_minimum_required(VERSION 3.5)
project(px4ctrl)

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

find_package(Eigen3 REQUIRED) 
find_package(spdlog REQUIRED)

# Build on ros
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(px4ctrl_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(px4ctrl_node
  src/node.cpp
  src/controller.cpp
  src/bridge.cpp
  src/fsm.cpp
  src/server.cpp
  src/frame_transforms.cpp
)

ament_target_dependencies(px4ctrl_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
  px4ctrl_msgs
  px4_msgs
)

target_link_libraries(px4ctrl_node
  spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  cppzmq
)

install(TARGETS px4ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

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
    PRIVATE Eigen3::Eigen
    PRIVATE yaml-cpp::yaml-cpp
    PRIVATE cppzmq
  )

  include(GoogleTest)
  gtest_discover_tests(px4ctrl_tests)
endif()