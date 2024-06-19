cmake_minimum_required(VERSION 3.5)
project(diablo_body)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ception_msgs REQUIRED)
find_package(diablo_utils REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME}_core)

set(dependencies
  rclcpp
  ception_msgs
  sensor_msgs
  motion_msgs
  diablo_utils
  builtin_interfaces
)

add_library(${library_name} SHARED
  src/diablo_imu.cpp
  src/diablo_battery.cpp
  src/diablo_legmotors.cpp
  src/diablo_body_state.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
