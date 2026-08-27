cmake_minimum_required(VERSION 3.8)
project(ydlidar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Core driver library
add_library(ydlidar_driver_core STATIC
  src/x4pro_serial.cpp
  src/x4pro_protocol.cpp
  src/x4pro_lidar.cpp
)

target_include_directories(ydlidar_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(ydlidar_driver_core PUBLIC cxx_std_17)

# Driver ROS 2 Node Executable
add_executable(ydlidar_node src/ydlidar_node.cpp)

target_include_directories(ydlidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(ydlidar_node PUBLIC cxx_std_17)

target_link_libraries(ydlidar_node ydlidar_driver_core)

ament_target_dependencies(ydlidar_node
  rclcpp
  sensor_msgs
  std_srvs
)

# Installation rules
install(TARGETS ydlidar_node ydlidar_driver_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# Testing (TDD)
if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  ament_flake8()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_x4pro_protocol
    test/test_x4pro_protocol.cpp
  )
  target_link_libraries(test_x4pro_protocol ydlidar_driver_core)

  ament_add_gtest(test_x4pro_parser
    test/test_x4pro_parser.cpp
  )
  target_link_libraries(test_x4pro_parser ydlidar_driver_core)
endif()

ament_package()
