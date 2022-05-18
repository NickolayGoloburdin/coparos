#ifndef AIRCRAFT_HANDLER_H
#define AIRCRAFT_HANDLER_H
#include "abstract_link.h"
#include <cstdint>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <string.h>

#define ACO_BUF_MAX_LENGTH 255

enum CommandTypes { INFO_CMD = 0x01 };
float degree_to_radian(float degree);
#pragma pack(push, 1)

typedef struct {
  uint8_t start_byte; // 0
  uint8_t packet_id;  // 1
  uint16_t packetLen; // 2
} Header_t;
typedef struct {
  uint8_t cmd;
  uint8_t state;
  float lat;
  float lon;
  float alt;
  float vel_ne;
  float heading;
  float yaw;
} Data_t;
#pragma pack(pop)

class AircraftHandler {
protected:       //номер байта входящего пакета
  uint8_t CRC_A; //для подсчёта контрольной суммы
  uint8_t CRC_B; //для подсчёта контрольной суммы

private:
  /****************заголовок пакета *****************/
  void CRC_calc();     //Подсчет контрольной сумм
  AbstractLink *link_; //Интерфейс предачи данных
  ros::NodeHandle *nh_;
  ros::Subscriber gps_sub_;
  ros::Subscriber baro_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber state_sub_;
  // std::ofstream tab;
  void callback_gps(const sensor_msgs::NavSatFix &msg);
  void callback_baro(const std_msgs::Float64 &msg);
  void callback_imu(const sensor_msgs::Imu &msg);
  void callback_state(const std_msgs::Int16 &msg);
  Data_t info;

public:
  AircraftHandler(AbstractLink *link, ros::NodeHandle *nh);
  ~AircraftHandler();
  uint8_t
      OutBuff[ACO_BUF_MAX_LENGTH]; //буфер для формирования исходящего пакета
  uint8_t OutSize; //количество байт исходящего пакета
  uint16_t packetNumber = 0; //Номера исходящего пакета
  void sendInfo();
  void sendPacket(); //Отправить пакет
  void PacketMake(void *body, uint8_t bodylen);
};

#endif // ABSTRACT_LINK_H