#include "diablo_battery.hpp"

using namespace std::chrono;

void diablo_battery_publisher::battery_pub_init(void)
{

    battery_Publisher_ = this->node_ptr->create_publisher<sensor_msgs::msg::BatteryState>("diablo/sensor/Battery",10);
    timer_ = this->node_ptr->create_wall_timer(1s,std::bind(&diablo_battery_publisher::lazyBatteryPublisher, this));
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_1Hz);
    this->vehicle->telemetry->configUpdate(); 
}

void diablo_battery_publisher::lazyBatteryPublisher(void){
    if(battery_Publisher_->get_subscription_count() > 0)
    {
        bool battery_Pub_mark = false;
        if(this->vehicle->telemetry->newcome & 0x02)
        {
            battery_Pub_mark = true;
            battery_msg_.voltage = this->vehicle->telemetry->power.voltage;
            battery_msg_.current = this->vehicle->telemetry->power.current;
            battery_msg_.capacity = this->vehicle->telemetry->power.capacitor_energy;
            battery_msg_.percentage = (this->vehicle->telemetry->power.power_percent)*1.0;
            battery_Publisher_->publish(battery_msg_);
            this->vehicle->telemetry->eraseNewcomeFlag(0xFD);
        }
        if(battery_Pub_mark){

            battery_timestamp = this->node_ptr->get_clock()->now();
            battery_msg_.header.stamp = battery_timestamp;
            battery_msg_.header.frame_id = "diablo_robot";
            battery_Publisher_->publish(battery_msg_);
            battery_Pub_mark = false;
        }
    }

}

diablo_battery_publisher::diablo_battery_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle)
{
    this->node_ptr = node_ptr;
    this->vehicle = vehicle;

}
