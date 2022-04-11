/*
 * telemetry_node.cpp
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */
#include "mobile_handler.h"
#include "udp_link.h"
#include <coparos/Telemetry.h>
#include <iostream>
#include <ros/ros.h>

// void write_callback(const std_msgs::String::ConstPtr &msg) {
//   //   ROS_INFO_STREAM("Writing to serial port" << msg->data);
//   //   ser.write(msg->data);
// }

int main(int argc, char **argv) {
  std::string ls_device, hs_device;
  bool realtime;
  //Инициализаци ноды РОС
  ros::init(argc, argv, "mobile_reader");
  ros::NodeHandle nh;
  //Чтение параметра  realtime, если этот параметр false то нода завершает
  //работу
  nh.getParam("/realtime", realtime);
  if (!realtime) {
    return 0;
  }
  // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
  //Инициализация модуля связи
  UDPLink *link = new UDPLink("localhost", 6600, 5600);
  //Инициализация модуля парсинга и отправки команд на коптер
  MobileHandler *handler = new MobileHandler(link, &nh);
  //Запуск работы модуля связи
  link->up();
  //Выставление частоты работы ноды
  ros::Rate loop_rate(5);
  while (ros::ok()) {
    // Чтение данных с коптера
    handler->parseFunc();
    //Обновление состояния ноды
    ros::spinOnce();

    loop_rate.sleep();
  }
}