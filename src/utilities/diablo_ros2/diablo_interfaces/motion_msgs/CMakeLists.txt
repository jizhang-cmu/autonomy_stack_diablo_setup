cmake_minimum_required(VERSION 3.5)
project(motion_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # msg
  "msg/CtrlPlot.msg"
  "msg/LegMotors.msg"
  "msg/MotionCtrl.msg"
  "msg/RobotStatus.msg"
  "msg/MovementCtrlData.msg"
  "msg/MovementCtrlMode.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()



