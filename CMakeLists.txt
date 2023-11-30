cmake_minimum_required(VERSION 3.8)
project(move_tb3_algo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(move_tb3 src/move_tb3.cpp)
ament_target_dependencies(move_tb3 rclcpp geometry_msgs sensor_msgs)

# Adding the walker node
install(TARGETS
  move_tb3
  DESTINATION lib/${PROJECT_NAME}/
)

# Adding the launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
