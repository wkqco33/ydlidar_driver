cmake_minimum_required(VERSION 3.8)
project(ydlidar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(DRIVER_SOURCES
  src/x4pro_serial.cpp
  src/x4pro_protocol.cpp
  src/x4pro_lidar.cpp
  src/ydlidar_node.cpp
)

add_executable(ydlidar_node ${DRIVER_SOURCES})

target_include_directories(ydlidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(ydlidar_node PUBLIC cxx_std_17)

ament_target_dependencies(ydlidar_node
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS ydlidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
