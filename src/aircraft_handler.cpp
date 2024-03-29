/*
 * telemetry_handler.cpp
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */
/************************************************************************************/

#include "aircraft_handler.h"
#include "abstract_link.h"
#include <tf/transform_datatypes.h>

AircraftHandler::AircraftHandler(AbstractLink *link, ros::NodeHandle *nh)
    : link_(link), nh_(nh) {
  info.cmd = 0x01;
  gps_sub_ = nh_->subscribe("/gps", 1000, &AircraftHandler::callback_gps, this);
  baro_sub_ =
      nh_->subscribe("/baro", 1000, &AircraftHandler::callback_baro, this);
  imu_sub_ = nh_->subscribe("/imu", 1000, &AircraftHandler::callback_imu, this);
  state_sub_ =
      nh_->subscribe("/state", 1000, &AircraftHandler::callback_state, this);
  // tab = std::ofstream("/home/jetson/aircraft_log.bin", std::ios::binary);
}

AircraftHandler::~AircraftHandler() { // tab.close();
}

// Подсчет контрольной суммы исходящего пакета

void AircraftHandler::CRC_calc() {
  uint8_t CRC_A = 0;
  uint8_t CRC_B = 0;
  for (int i = 1; i < (OutSize - 2); i++) {
    CRC_A += OutBuff[i];
    CRC_B += CRC_A;
  }
  OutBuff[OutSize - 2] = CRC_A;
  OutBuff[OutSize - 1] = CRC_B;
}

void AircraftHandler::callback_gps(const sensor_msgs::NavSatFix &msg) {

  info.lat = msg.latitude;
  info.lon = msg.longitude;
  info.alt = msg.altitude;
}
void AircraftHandler::callback_baro(const std_msgs::Float64 &msg) {
  info.alt = msg.data;
}
void AircraftHandler::callback_imu(const sensor_msgs::Imu &msg) {
  // info.yaw =
  tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z,
                   msg.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  info.yaw = yaw;
}
void AircraftHandler::callback_state(const std_msgs::Int16 &msg) {
  info.state = msg.data;
}

void AircraftHandler::sendInfo() {
  PacketMake(&info, sizeof(Data_t));
  // tab.write((char *)&OutBuff, OutSize);
  sendPacket();
}
/*******************сборка пакет для
 * отправки*************************************/
void AircraftHandler::PacketMake(void *body, uint8_t bodylen) {
  Header_t *Hdr = (Header_t *)OutBuff;
  Hdr->start_byte = 0xD5; // A
  Hdr->packet_id = 0x44;
  Hdr->packetLen = bodylen;
  if (bodylen) //если полезная нагрузка не пустая то присоединяется полезная
               //нагрузка
    memcpy(&(OutBuff[sizeof(Header_t)]), body, bodylen);
  OutSize = sizeof(Header_t) + bodylen +
            2; //длинна всего пакета плюс 2 байта контрольной суммы
  CRC_calc();  //контрольная сумма
               //отправить ответ
}

void AircraftHandler::sendPacket() { link_->sendData(OutBuff, OutSize); }