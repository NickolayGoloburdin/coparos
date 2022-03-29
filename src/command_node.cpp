
#include "command_handler.h"
#include "serial_link.h"
#include "telemetry_handler.h"
#include <coparos/Telemetry.h>
#include <iostream>
#include <queue>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
void write_callback(const std_msgs::String::ConstPtr &msg) {
  //   ROS_INFO_STREAM("Writing to serial port" << msg->data);
  //   ser.write(msg->data);
}

int main(int argc, char **argv) {
  bool realtime;
  ros::init(argc, argv, "command_node");
  ros::NodeHandle nh;
  
  nh.getParam("/realtime", realtime);
  if (!realtime) {
    return 0;
  }
  // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
  //  ros::Publisher read_pub = nh.advertise<coparos::Telemetry>("Telemetry",
  //  1000);
  SerialLink *link_ls = new SerialLink("/dev/ttyTHS1", 115200);

  COPA *copa = new COPA(link_ls, &nh);
  link_ls->up();
  copa->Preset_Set_Param();
  // ros::ServiceServer service =
  //     nh.advertiseService("ArmDisarm", &COPA::ArmDisarmFunc, copa);
  ros::Rate loop_rate(100);
  while (ros::ok()) {

    copa->parseFunc();

    ros::spinOnce();

    loop_rate.sleep();
  }
}
