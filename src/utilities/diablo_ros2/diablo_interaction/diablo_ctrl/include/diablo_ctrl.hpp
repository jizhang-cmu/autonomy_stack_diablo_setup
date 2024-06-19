#pragma once

#include "diablo_imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diablo_battery.hpp"
#include "diablo_legmotors.hpp"
#include "diablo_body_state.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

#define CMD_GO_FORWARD                               0x08
#define CMD_GO_LEFT                                  0x04
#define CMD_ROLL_RIGHT                               0x09

#define CMD_HEIGH_MODE                               0x01 //set 0 or 1
#define CMD_BODY_UP                                  0x11

#define CMD_STAND_UP                                 0x02
#define CMD_STAND_DOWN                               0x12

#define CMD_PITCH                                    0x03
#define CMD_PITCH_MODE                               0x13

#define CMD_SPEED_MODE                               0x05


class diabloCtrlNode : public rclcpp::Node
{

public:
    diabloCtrlNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Sub node: %s.",name.c_str());
        sub_movement_cmd = this->create_subscription<motion_msgs::msg::MotionCtrl>("diablo/MotionCmd", 10, std::bind(&diabloCtrlNode::Motion_callback, this, std::placeholders::_1));
        ctrl_msg_.value.up = 1.0;
    }
    ~diabloCtrlNode();

    void run_(void);
    void heart_beat_loop(void);
    std::shared_ptr<std::thread> thread_;
    DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;
    DIABLO::OSDK::Telemetry* pTelemetry;




private:
    void Motion_callback(const motion_msgs::msg::MotionCtrl::SharedPtr msg);

private:
    rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr sub_movement_cmd;
    OSDK_Movement_Ctrl_t    cmd_value;
    bool                onSend = true;
    bool        thd_loop_mark_ = true;
    motion_msgs::msg::MotionCtrl                                       ctrl_msg_;
};


