/*
 * telemetry_node.cpp
 *
 *  Created on: 5 августа 2022 г.
 *      Author: Nickolay
 */

#include <coparos/DroneInfo.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// void write_callback(const std_msgs::String::ConstPtr &msg) {
//   //   ROS_INFO_STREAM("Writing to serial port" << msg->data);
//   //   ser.write(msg->data);
// }
class StateMachine {
private:
  ros::NodeHandle *nh_;
  bool gnss_status = true;
  int channel11_;
  unsigned int drone_mode;
  ros::Subscriber gnss_use_status_sub_;
  ros::Subscriber rc_channel_sub_;

  StateMachine(ros::NodeHandle *nh) : nh_(nh) {
    gnss_use_status_sub_ = nh_->subscribe("/gnss_use_status", 1,
                                          &StateMachine::callback_gnss, this);
    rc_channel_sub_ = nh_->subscribe("/droneInfo", 1,
                                     &StateMachine::callback_drone_info, this);
  }
  void callback_drone_info(const coparos::DroneInfo &msg) {
    channel11_ = msg.rc11_channel;
    drone_mode = msg.DRONE_MODE;
  }
  void callback_gnss(const std_msgs::Bool &msg) { gnss_status = msg.data; }

public:
};
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    // Чтение данных с коптера
    //Обновление состояния ноды
    ros::spinOnce();

    loop_rate.sleep();
  }
}