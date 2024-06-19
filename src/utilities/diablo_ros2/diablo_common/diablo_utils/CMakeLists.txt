cmake_minimum_required(VERSION 3.5)
project(diablo_utils)

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
add_definitions(-D_LOG_CONSOLE)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(rclcpp REQUIRED)


include_directories(include)

set(library_name ${PROJECT_NAME}_core)

set(dependencies
  rclcpp
  motion_msgs
)

add_library(${library_name} SHARED
  src/SerialPort/SerialPort.cpp
  src/diablo_tools/osdk_crc.cpp
  src/diablo_tools/osdk_hal.cpp
  src/diablo_tools/osdk_log.cpp
  src/diablo_tools/osdk_movement.cpp
  src/diablo_tools/osdk_telemetry.cpp
  src/diablo_tools/osdk_vehicle.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  motion_msgs
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
