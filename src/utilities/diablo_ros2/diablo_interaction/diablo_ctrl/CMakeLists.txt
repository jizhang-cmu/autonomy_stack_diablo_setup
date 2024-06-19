cmake_minimum_required(VERSION 3.5)
project(diablo_ctrl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(diablo_body REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(diablo_utils REQUIRED)


include_directories(
    include
)

set(dependencies
  rclcpp
  diablo_body
  motion_msgs
  diablo_utils
)

set(executable_name diablo_ctrl_node)

add_executable(${executable_name} 
      src/diablo_ctrl.cpp
  )

ament_target_dependencies(${executable_name} ${dependencies})
# ament_target_dependencies(${library_name} ${dependencies})


install(TARGETS ${library_name}
  DESTINATION lib)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()

