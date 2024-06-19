#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joy.hpp>

#include "motion_msgs/msg/motion_ctrl.hpp"
#include "motion_msgs/msg/robot_status.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include "ception_msgs/msg/imu_euler.hpp"

using namespace std;

const double PI = 3.1415926;

float joySpeed = 0;
float joyYaw = 0;
float joyHeight = 0;
float joyPitch = 0;

bool vehicleReady = false;
float bodyHeight = 1.0;
float bodyPitch = 0;

float bodyLow = 0.28;
float bodyHigh = 0.52;
float bodySpeedGain = 0.5;
float bodyMaxSpeed = 0.2;

float pitchRateGain = 2.0;
float pitchMaxRate = 0.5;

motion_msgs::msg::MotionCtrl ctrl_msg;

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joySpeed = joy->axes[4];
  joyYaw = joy->axes[3];
  joyHeight = joy->axes[1];
  joyPitch = joy->axes[0];
  
  if (joyHeight > 0) joyHeight = 0;
}

void robotStatusHandler(const motion_msgs::msg::RobotStatus::ConstSharedPtr status)
{
  if (status->robot_mode_msg == 3) vehicleReady = true;
  else vehicleReady = false;
  
  printf ("Ctrl mode %d, Robot mode %d\n", status->ctrl_mode_msg, status->robot_mode_msg);
}

void motorStatusHandler(const motion_msgs::msg::LegMotors::ConstSharedPtr status)
{
  bodyHeight = (status->left_leg_length + status->right_leg_length - bodyLow) / (bodyHigh - bodyLow);
}

void imuHandler(const ception_msgs::msg::IMUEuler::ConstSharedPtr status)
{
  bodyPitch = status->pitch;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("diabloTeleopJoy");

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subRobotStatus = nh->create_subscription<motion_msgs::msg::RobotStatus>("/diablo/sensor/Body_state", 5, robotStatusHandler);

  auto subMotorStatus = nh->create_subscription<motion_msgs::msg::LegMotors>("/diablo/sensor/Motors", 5, motorStatusHandler);

  auto subIMU = nh->create_subscription<ception_msgs::msg::IMUEuler>("/diablo/sensor/ImuEuler", 5, imuHandler);

  auto pubMotionCtrl = nh->create_publisher<motion_msgs::msg::MotionCtrl>("/diablo/MotionCmd", 5);

  ctrl_msg.value.forward = 0;
  ctrl_msg.value.left = 0;
  ctrl_msg.value.up = 0;
  ctrl_msg.value.roll = 0;
  ctrl_msg.value.pitch = 0;
  ctrl_msg.value.leg_split = 0;
  ctrl_msg.mode.pitch_ctrl_mode = true;
  ctrl_msg.mode.roll_ctrl_mode = false;
  ctrl_msg.mode.height_ctrl_mode = true;
  ctrl_msg.mode.stand_mode = true;
  ctrl_msg.mode.jump_mode = false;
  ctrl_msg.mode.split_mode = false;

  int ctrlInitFrameCount = 100;
  rclcpp::Rate rate(50);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (vehicleReady) {
      ctrl_msg.value.forward = 1.0 * joySpeed;
      ctrl_msg.value.left = 80.0 * joyYaw * PI / 180;

      float desiredHeight = joyHeight + 1.0;
      ctrl_msg.value.up = bodySpeedGain * (desiredHeight - bodyHeight);
      if (ctrl_msg.value.up < -bodyMaxSpeed) ctrl_msg.value.up = -bodyMaxSpeed;
      else if (ctrl_msg.value.up > bodyMaxSpeed) ctrl_msg.value.up = bodyMaxSpeed;

      ctrl_msg.value.pitch = -pitchRateGain * bodyPitch;
      if (ctrl_msg.value.pitch < -pitchMaxRate) ctrl_msg.value.pitch = -pitchMaxRate;
      else if (ctrl_msg.value.pitch > pitchMaxRate) ctrl_msg.value.pitch = pitchMaxRate;
    } else {
      ctrl_msg.value.forward = 0;
      ctrl_msg.value.left = 0;
      ctrl_msg.value.up = 0;
      ctrl_msg.value.pitch = 0;
    }

    ctrl_msg.mode_mark = false;
    if (ctrlInitFrameCount > 0) {
      ctrl_msg.mode_mark = true;
      ctrlInitFrameCount--;
    }

    pubMotionCtrl->publish(ctrl_msg);

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
