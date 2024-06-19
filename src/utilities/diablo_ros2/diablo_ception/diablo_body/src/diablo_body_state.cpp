#include "diablo_body_state.hpp"

using namespace std::chrono;

void diablo_body_state_publisher::body_pub_init(void)
{
    robot_state_Publisher_ = this->node_ptr->create_publisher<motion_msgs::msg::RobotStatus>("diablo/sensor/Body_state", 10);
    timer_ = this->node_ptr->create_wall_timer(100ms,std::bind(&diablo_body_state_publisher::lazyPublisher, this));
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_STATUS, OSDK_PUSH_DATA_10Hz);
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_RC, OSDK_PUSH_DATA_OFF);
    this->vehicle->telemetry->configUpdate(); 
}


void diablo_body_state_publisher::lazyPublisher(void){
    if(robot_state_Publisher_->get_subscription_count() > 0)
    {
        bool body_state_Pub_mark = false;
        if(this->vehicle->telemetry->newcome & 0x40)
        {
            body_state_Pub_mark = true;
            robot_state_msg_.ctrl_mode_msg = this->vehicle->telemetry->status.ctrl_mode;
            robot_state_msg_.robot_mode_msg = this->vehicle->telemetry->status.robot_mode;
            robot_state_msg_.error_msg = this->vehicle->telemetry->status.error;
            robot_state_msg_.warning_msg = this->vehicle->telemetry->status.warning;

            this->vehicle->telemetry->eraseNewcomeFlag(0x40);
        }
        
        if(body_state_Pub_mark){

            robot_state_timestamp = this->node_ptr->get_clock()->now();
            robot_state_msg_.header.stamp = robot_state_timestamp;
            robot_state_msg_.header.frame_id = "diablo_robot";
            robot_state_Publisher_->publish(robot_state_msg_);
            body_state_Pub_mark = false;
        }
    }
}

diablo_body_state_publisher::diablo_body_state_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle)
{
    this->node_ptr = node_ptr;
    this->vehicle = vehicle;

}
