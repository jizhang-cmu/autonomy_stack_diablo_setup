#include "diablo_legmotors.hpp"

using namespace std::chrono;

void diablo_motors_publisher::motors_pub_init(void)
{
    motors_Publisher_ = this->node_ptr->create_publisher<motion_msgs::msg::LegMotors>("diablo/sensor/Motors",10);
    timer_ = this->node_ptr->create_wall_timer(20ms,std::bind(&diablo_motors_publisher::lazyMotorsPublisher, this));
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_50Hz);
    this->vehicle->telemetry->configUpdate(); 
}

void diablo_motors_publisher::lazyMotorsPublisher(void){
    if(motors_Publisher_->get_subscription_count() > 0)
    {
        bool motors_Pub_mark = false;
        
        if(this->vehicle->telemetry->newcome & 0x01)
        {
            motors_Pub_mark = true;
            motors_msg_.left_hip_enc_rev = this->vehicle->telemetry->motors.left_hip.rev;
            motors_msg_.left_hip_pos = this->vehicle->telemetry->motors.left_hip.pos;
            motors_msg_.left_hip_vel = this->vehicle->telemetry->motors.left_hip.vel;
            motors_msg_.left_hip_iq = this->vehicle->telemetry->motors.left_hip.iq;

            motors_msg_.left_knee_enc_rev = this->vehicle->telemetry->motors.left_knee.rev;
            motors_msg_.left_knee_pos = this->vehicle->telemetry->motors.left_knee.pos;
            motors_msg_.left_knee_vel = this->vehicle->telemetry->motors.left_knee.vel;
            motors_msg_.left_knee_iq = this->vehicle->telemetry->motors.left_knee.iq;

            motors_msg_.left_wheel_enc_rev = this->vehicle->telemetry->motors.left_wheel.rev;
            motors_msg_.left_wheel_pos = this->vehicle->telemetry->motors.left_wheel.pos;
            motors_msg_.left_wheel_vel = this->vehicle->telemetry->motors.left_wheel.vel;
            motors_msg_.left_wheel_iq = this->vehicle->telemetry->motors.left_wheel.iq;

            motors_msg_.right_hip_enc_rev = this->vehicle->telemetry->motors.right_hip.rev;
            motors_msg_.right_hip_pos = this->vehicle->telemetry->motors.right_hip.pos;
            motors_msg_.right_hip_vel = this->vehicle->telemetry->motors.right_hip.vel;
            motors_msg_.right_hip_iq = this->vehicle->telemetry->motors.right_hip.iq;

            motors_msg_.right_knee_enc_rev = this->vehicle->telemetry->motors.right_knee.rev;
            motors_msg_.right_knee_pos = this->vehicle->telemetry->motors.right_knee.pos;
            motors_msg_.right_knee_vel = this->vehicle->telemetry->motors.right_knee.vel;
            motors_msg_.right_knee_iq =  this->vehicle->telemetry->motors.right_knee.iq;

            motors_msg_.right_wheel_enc_rev = this->vehicle->telemetry->motors.right_wheel.rev;
            motors_msg_.right_wheel_pos = this->vehicle->telemetry->motors.right_wheel.pos;
            motors_msg_.right_wheel_vel = this->vehicle->telemetry->motors.right_wheel.vel;
            motors_msg_.right_wheel_iq = this->vehicle->telemetry->motors.right_wheel.iq;

            motors_msg_.left_leg_length = this->vehicle->telemetry->left_leg_length;
            motors_msg_.right_leg_length = this->vehicle->telemetry->right_leg_length;

            this->vehicle->telemetry->eraseNewcomeFlag(0xFE);
        }
        if(motors_Pub_mark){

            motors_timestamp = this->node_ptr->get_clock()->now();
            motors_msg_.header.stamp = motors_timestamp;
            motors_msg_.header.frame_id = "diablo_robot";
            motors_Publisher_->publish(motors_msg_);
            motors_Pub_mark = false;
        }
    }
}

diablo_motors_publisher::diablo_motors_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle)
{
    this->node_ptr = node_ptr;
    this->vehicle = vehicle;

}
