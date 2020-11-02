cmake_minimum_required(VERSION 3.5)
project(tf_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp geometry_msgs std_msgs tf2 tf2_ros)

add_executable(waiter src/waiter.cpp)
ament_target_dependencies(waiter rclcpp geometry_msgs std_msgs tf2 tf2_ros)

add_executable(publisher_case2 src/publisher_case2.cpp)
ament_target_dependencies(publisher_case2 rclcpp geometry_msgs std_msgs tf2 tf2_ros)

add_executable(waiter_case2 src/waiter_case2.cpp)
ament_target_dependencies(waiter_case2 rclcpp geometry_msgs std_msgs tf2 tf2_ros)

install(TARGETS
  publisher
  waiter
  publisher_case2
  waiter_case2
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
