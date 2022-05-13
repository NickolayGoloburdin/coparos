/*
 * telemetry_node.cpp
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */
#include "aircraft_handler.h"
#include "serial_link.h"
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>

// void write_callback(const std_msgs::String::ConstPtr &msg) {
//   //   ROS_INFO_STREAM("Writing to serial port" << msg->data);
//   //   ser.write(msg->data);
// }

int main(int argc, char *argv[]) {

  bool realtime;
  //Инициализаци ноды РОС

  ros::init(argc, argv, "aircraft_communicator");
  ros::NodeHandle nh;
  //Чтение параметра  realtime, если этот параметр false то нода завершает
  //работу
  nh.getParam("/realtime", realtime);
  if (!realtime) {
    return 0;
  }
  std::string port = "/dev/ttyUSB0";
  if (argc > 1)
    port = argv[1];

  // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
  //Инициализация модуля связи
  SerialLink *link_hs = new SerialLink(port, 921600);
  //Инициализация модуля парсинга и отправки команд на коптер
  AircraftHandler *handler = new AircraftHandler(link_hs, &nh);
  //Запуск работы модуля связи
  // link_hs->up();
  //Выставление частоты работы ноды
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    // Чтение данных с коптера
    handler->sendInfo();
    //Обновление состояния ноды
    ros::spinOnce();

    loop_rate.sleep();
  }
}