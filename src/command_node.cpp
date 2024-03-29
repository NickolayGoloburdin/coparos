/*
 * command_node.cpp
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */
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
  // Инициализация РОС ноды коммуникации
  ros::init(argc, argv, "command_node");
  ros::NodeHandle nh;

  // nh.getParam("/realtime", realtime);
  // if (!realtime) {
  //   return 0;
  // }
  ros::Duration(3).sleep();
  int port_num;
  std::string port;
  while (!nh.getParam("/hs_port", port_num)) {
    ros::Duration(0.1).sleep();
  }
  if (port_num == 1) {
    port = "/dev/ttyUSB0";
  } else {
    port = "/dev/ttyUSB1";
  }
  // std::string port = "/dev/ttyTHS1";
  //Инициализация модуля связи по последовательному порту
  SerialLink *link_ls = new SerialLink(port, 115200);
  //Инициализация модуля коммуникации
  COPA *copa = new COPA(link_ls, &nh);
  //Запуск модуля свзяи
  // ros::Duration(0.3).sleep();
  link_ls->up();
  //Выставление пресетов для получения параметров с коптера
  copa->Preset_Set_Param();
  // ros::Duration(0.1).sleep();
  // copa->parseFunc();
  // if (!copa->successPacket_) {
  //   link_ls->changeAddress("/dev/ttyUSB1");
  // }
  //Выставление частоты обработки команд с коптера
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    // Чтение и парсинг входных данных с коптера
    copa->parseFunc();
    // Обновление состояния ноды
    ros::spinOnce();
    // Засыпание ноды
    loop_rate.sleep();
  }
}
