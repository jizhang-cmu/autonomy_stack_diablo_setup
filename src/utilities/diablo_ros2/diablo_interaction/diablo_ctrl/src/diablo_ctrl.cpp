#include <iostream>
#include "diablo_ctrl.hpp"

using namespace std;

void diabloCtrlNode::heart_beat_loop(void){
    //RCLCPP_INFO(this->get_logger(), "start.");
    //while (this->thd_loop_mark_)
    //{
        if (onSend)
        {
            if(!pMovementCtrl->in_control())
            {
                pMovementCtrl->obtain_control();
                //continue;
                return;
            }
            
            if(!ctrl_msg_.mode_mark){
                pMovementCtrl->ctrl_data.forward = ctrl_msg_.value.forward;
                pMovementCtrl->ctrl_data.left = ctrl_msg_.value.left;
                pMovementCtrl->ctrl_data.up = ctrl_msg_.value.up;
                pMovementCtrl->ctrl_data.roll = ctrl_msg_.value.roll;
                pMovementCtrl->ctrl_data.pitch = ctrl_msg_.value.pitch;
                pMovementCtrl->ctrl_data.leg_split = ctrl_msg_.value.leg_split;
                pMovementCtrl->SendMovementCtrlCmd();
                
            }else{
                if(ctrl_msg_.mode.stand_mode)
                    pMovementCtrl->SendTransformUpCmd();
                else{
                    pMovementCtrl->SendTransformDownCmd();
                }
                pMovementCtrl->ctrl_mode_data.height_ctrl_mode = ctrl_msg_.mode.height_ctrl_mode;
                pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = ctrl_msg_.mode.pitch_ctrl_mode;
                pMovementCtrl->ctrl_mode_data.roll_ctrl_mode = ctrl_msg_.mode.roll_ctrl_mode;
                pMovementCtrl->SendMovementModeCtrlCmd();
            }
            //usleep(180000);
        }
    //}
}

void diabloCtrlNode::run_(void){
    this->thd_loop_mark_ = true;
    this->thread_ = std::make_shared<std::thread>(&diabloCtrlNode::heart_beat_loop,this);
}

void diabloCtrlNode::Motion_callback(const motion_msgs::msg::MotionCtrl::SharedPtr msg)
{
    onSend = false;
    if(!pMovementCtrl->in_control())
    {
        pMovementCtrl->obtain_control();
        return;
    }
    ctrl_msg_.mode = msg->mode;
    ctrl_msg_.mode_mark = msg->mode_mark;
    ctrl_msg_.value = msg->value;
    
    if(!msg->mode_mark){
        pMovementCtrl->ctrl_data.forward = msg->value.forward;
        pMovementCtrl->ctrl_data.left = msg->value.left;
        pMovementCtrl->ctrl_data.up = msg->value.up;
        pMovementCtrl->ctrl_data.roll = msg->value.roll;
        pMovementCtrl->ctrl_data.pitch = msg->value.pitch;
        pMovementCtrl->ctrl_data.leg_split = msg->value.leg_split;
        pMovementCtrl->SendMovementCtrlCmd();
    }else{
 
        if(msg->mode.stand_mode)
            pMovementCtrl->SendTransformUpCmd();
        else{
            pMovementCtrl->SendTransformDownCmd();
        }
        
        if(pTelemetry->status.robot_mode == 3){
            pMovementCtrl->SendJumpCmd(msg->mode.jump_mode);
        }
        pMovementCtrl->SendDanceCmd(msg->mode.split_mode);
        pMovementCtrl->ctrl_mode_data.height_ctrl_mode = msg->mode.height_ctrl_mode;
        pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = msg->mode.pitch_ctrl_mode;
        pMovementCtrl->ctrl_mode_data.roll_ctrl_mode = msg->mode.roll_ctrl_mode;
        pMovementCtrl->SendMovementModeCtrlCmd();
    }
    onSend = true;
}

diabloCtrlNode::~diabloCtrlNode()
{
    RCLCPP_INFO(this->get_logger(), "done.");
    this->thd_loop_mark_ = false;
    thread_->join();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<diabloCtrlNode>("diablo_ctrl_node");

    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init("/dev/ttyUSB0")) return -1;

    DIABLO::OSDK::Vehicle vehicle(&Hal);                     
    if(vehicle.init()) return -1;

    vehicle.telemetry->activate();

    diablo_imu_publisher imuPublisher(node,&vehicle);
    imuPublisher.imu_pub_init();

    diablo_battery_publisher batteryPublisher(node,&vehicle);
    batteryPublisher.battery_pub_init();

	diablo_motors_publisher motorsPublisher(node,&vehicle);
    motorsPublisher.motors_pub_init();

    diablo_body_state_publisher bodyStatePublisher(node,&vehicle);
    bodyStatePublisher.body_pub_init();

    // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);
    // vehicle.telemetry->setMaxSpeed(1.0);
    node->pMovementCtrl = vehicle.movement_ctrl;
    node->pTelemetry = vehicle.telemetry;
    //node->run_();

    int heart_beat_count = 0;
    rclcpp::Rate rate(100);
    bool status = rclcpp::ok();
    while (status) {
      rclcpp::spin_some(node);

      heart_beat_count++;
      if (heart_beat_count > 100) {
        node->heart_beat_loop();
        heart_beat_count = 0;
      }

      status = rclcpp::ok();
      rate.sleep();
    }

    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
