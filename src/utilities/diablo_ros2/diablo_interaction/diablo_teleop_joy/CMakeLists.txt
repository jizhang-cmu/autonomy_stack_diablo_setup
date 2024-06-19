cmake_minimum_required(VERSION 3.5)
project(diablo_teleop_joy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(sensor_msgs REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(ception_msgs REQUIRED)

include_directories(
    include
)

add_executable(diabloTeleopJoy src/diabloTeleopJoy.cpp)
ament_target_dependencies(diabloTeleopJoy rclcpp std_msgs sensor_msgs motion_msgs ception_msgs)

install(TARGETS
  diabloTeleopJoy
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
