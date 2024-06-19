#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_battery_publisher
{
private:

    rclcpp::TimerBase::SharedPtr                                             timer_;
    DIABLO::OSDK::Vehicle*                                                  vehicle;
    rclcpp::Node::SharedPtr                                                node_ptr;
    sensor_msgs::msg::BatteryState                                     battery_msg_;
    builtin_interfaces::msg::Time                                 battery_timestamp;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_Publisher_;

public:
    diablo_battery_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle);
    ~diablo_battery_publisher(){}
    void battery_pub_init(void);
    void lazyBatteryPublisher(void);
};


