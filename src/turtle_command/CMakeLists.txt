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

add_executable(turtle_chaser_spawn src/turtle_chaser_spawn.cpp)
ament_target_dependencies(turtle_chaser_spawn rclcpp turtlesim)

add_executable(turtle_chaser_set_pen src/turtle_chaser_set_pen.cpp)
ament_target_dependencies(turtle_chaser_set_pen rclcpp turtlesim)


install(TARGETS
  turtle_runner
  turtle_chaser
  turtle_chaser_spawn
  turtle_chaser_set_pen
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
