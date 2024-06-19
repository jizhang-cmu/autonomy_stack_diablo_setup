#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_motors_publisher
{
private:

    rclcpp::TimerBase::SharedPtr                                             timer_;
    DIABLO::OSDK::Vehicle*                                                  vehicle;
    rclcpp::Node::SharedPtr                                                node_ptr;
    motion_msgs::msg::LegMotors                                         motors_msg_;
    builtin_interfaces::msg::Time                                  motors_timestamp;
    rclcpp::Publisher<motion_msgs::msg::LegMotors>::SharedPtr     motors_Publisher_;

public:
    diablo_motors_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle);
    ~diablo_motors_publisher(){}
    void motors_pub_init(void);
    void lazyMotorsPublisher(void);
};


