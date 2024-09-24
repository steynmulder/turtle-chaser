cmake_minimum_required(VERSION 3.8)
project(turtle_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_runner src/turtle_runner.cpp)
ament_target_dependencies(turtle_runner rclcpp geometry_msgs)

add_executable(turtle_chaser src/turtle_chaser.cpp)
ament_target_dependencies(turtle_chaser rclcpp turtlesim)

install(TARGETS
  turtle_runner
  turtle_chaser
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
