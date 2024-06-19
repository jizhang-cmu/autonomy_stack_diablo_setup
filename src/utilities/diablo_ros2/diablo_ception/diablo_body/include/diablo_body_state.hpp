#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/robot_status.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_body_state_publisher
{
private:

    rclcpp::TimerBase::SharedPtr                                                timer_;
    DIABLO::OSDK::Vehicle*                                                     vehicle;
    rclcpp::Node::SharedPtr                                                   node_ptr;
    motion_msgs::msg::RobotStatus                                     robot_state_msg_;
    builtin_interfaces::msg::Time                                robot_state_timestamp;
    rclcpp::Publisher<motion_msgs::msg::RobotStatus>::SharedPtr robot_state_Publisher_;

public:
    diablo_body_state_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle);
    ~diablo_body_state_publisher(){}
    void body_pub_init(void);
    void lazyPublisher(void);
};


