/*
 * telemetry_node.cpp
 *
 *  Created on: 5 августа 2022 г.
 *      Author: Nickolay
 */

#include <ros/ros.h>

// void write_callback(const std_msgs::String::ConstPtr &msg) {
//   //   ROS_INFO_STREAM("Writing to serial port" << msg->data);
//   //   ser.write(msg->data);
// }

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;
  //Чтение параметра  realtime, если этот параметр false то нода завершает
  //работу
  // nh.getParam("/realtime", realtime);
  // if (!realtime) {
  //   return 0;
  // }

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    // Чтение данных с коптера
    //Обновление состояния ноды
    ros::spinOnce();

    loop_rate.sleep();
  }
}