
#include "serial_link.h"
#include "telemetry_handler.h"
#include <coparos/Telemetry.h>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>

// void write_callback(const std_msgs::String::ConstPtr &msg) {
//   //   ROS_INFO_STREAM("Writing to serial port" << msg->data);
//   //   ser.write(msg->data);
// }

int main(int argc, char **argv) {
  std::string ls_device, hs_device;
  int ls_baud, hs_baud;

  ros::init(argc, argv, "telemetry_reader");
  ros::NodeHandle nh;

  nh.getParam("/hs_port", hs_device);

  nh.getParam("/hs_baud", hs_baud);

  // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
  SerialLink *link_hs = new SerialLink(hs_device, hs_baud);
  TelemetryHandler_ACO *handler = new TelemetryHandler_ACO(link_hs, &nh);
  link_hs->up();
  handler->ACOTelemOnOff(1);
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    // if copa
    handler->parseFunc();
    ros::spinOnce();

    loop_rate.sleep();
  }
}