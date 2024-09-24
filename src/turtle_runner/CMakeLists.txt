cmake_minimum_required(VERSION 3.8)
project(turtle_runner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_runner src/turtle_runner.cpp)
ament_target_dependencies(turtle_runner rclcpp geometry_msgs)

install(TARGETS
  turtle_runner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
