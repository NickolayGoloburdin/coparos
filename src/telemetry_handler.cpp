/*
 * telemetry_handler.cpp
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */
/************************************************************************************/

#include "telemetry_handler.h"
#include "abstract_link.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*********************************** Подсчёт CRC
 * *************************************/
/*подсчёт CRC */
float degree_to_radian(float degree) {
  float pi = 3.14159265359;
  return (degree * (pi / 180));
}
TelemetryHandler_ACO::TelemetryHandler_ACO(AbstractLink *link,
                                           ros::NodeHandle *nh,
                                           HandlerType type)
    : link_(link), nh_(nh), type_(type) {
  if (type_ == RECIEVER || type_ == DUPLEX) {
    imu_pub_ = nh_->advertise<sensor_msgs::Imu>("/imu", 1000);
    gps_pub_ = nh_->advertise<sensor_msgs::NavSatFix>("/gps", 1000);
    baro_pub_ = nh_->advertise<std_msgs::Float64>("/baro", 1000);
  }
  if (type_ == TRANSMITTER || type_ == DUPLEX)
    fake_gps_sub_ = nh_->subscribe(
        "/gpsFromVideo", 1000, &TelemetryHandler_ACO::callback_fake_gps, this);
}

TelemetryHandler_ACO::~TelemetryHandler_ACO() {}
// Подсчет контрольной суммы исходящего пакета
void TelemetryHandler_ACO::ACO_CRC_calc() {
  uint8_t CRC_A = 0;
  uint8_t CRC_B = 0;
  for (int i = 3; i < (OutSize - 2); i++) {
    CRC_A += OutBuff[i];
    CRC_B += CRC_A;
  }
  OutBuff[OutSize - 2] = CRC_A;
  OutBuff[OutSize - 1] = CRC_B;
}
//Метод для парсинга входящих данных
void TelemetryHandler_ACO::parseFunc() {
  if (type_ == TRANSMITTER)
    return;

  if (link_->isUp()) {
    auto list_ptr = link_->getData();
    if (list_ptr)
      ACOParseBUF(list_ptr);
  }
}

/******************************* Парсинг буфера
 * ******************************************************/
void TelemetryHandler_ACO::ACOParseBUF(
    std::shared_ptr<std::list<unsigned char>> list_ptr) {
  uint16_t InSize = 0;
  uint8_t InBuff[ACO_BUF_MAX_LENGTH];
  for (auto ptr = list_ptr->begin(); ptr != list_ptr->end(); ptr++)
    ACO_Parse_Byte(*ptr, InSize, InBuff);
}
/************************** Побайтное чтение пакета **************************/
void TelemetryHandler_ACO::ACO_Parse_Byte(uint8_t byte_, uint16_t &InSize,
                                          uint8_t *InBuff) {
  switch (InSize) {
  case 0:
    if (byte_ == ACOMAGIC1) { //чтение первого байта A
      InBuff[InSize++] = byte_;
      CRC_A = 0;
      CRC_B = 0;
    };
    break;
  case 1:
    if (byte_ == ACOMAGIC2) { //чтение второго байта C
      InBuff[InSize++] = byte_;
    } else
      InSize = 0;
    break;
  case 2:
    if (byte_ == ACOMAGIC3) { //чтение трктьего байта O
      InBuff[InSize++] = byte_;
      CRC_A = 0;
      CRC_B = 0;
    } else
      InSize = 0;
    break;
  case 3:
    InBuff[InSize++] = byte_; //количество байт пакета - начало
    CRC_A += byte_;           //подсчет контрольной суммы
    CRC_B += CRC_A;
    PackLen_ = byte_;
    /*проверяю на корректность полученной длины*/
    if ((PackLen_ < 4) || (PackLen_ > (ACO_BUF_MAX_LENGTH - 4)))
      InSize =
          0; //если размер будущего пакета не корректный то чтение прекращается
    break;
  default:
    InBuff[InSize++] = byte_;
    if (InSize < (PackLen_ + 3)) { //
      CRC_A += byte_;
      CRC_B += CRC_A;
    } else {
      if (InSize >= (PackLen_ + 4)) {

        if ((InBuff[InSize - 2] == CRC_A) && (InBuff[InSize - 1] == CRC_B)) {
          PackRec((Header_t *)InBuff, &(InBuff[7]));
          successPacket_ = true;
          InSize = 0; //сброс счетчика
        };
      }
    }
    break;
  };
}

/************************** Парсинг принятого пакета **************************/

void TelemetryHandler_ACO::PackRec(Header_t *header, void *body) {
  switch (header->commandType) {
  case OF_CMD_ACK: //ответ подтверждение
    break;
  case OF_CMD_FLOW: // OFM => FC это команда коптеру п
    break;
  case OF_CMD_TELEM:
    // FC => OFM пакет пришёл от коптера
    // Создание сообщений телемтрии для отправки в топики РОС
    Telemetry_data_t Telemetry_data;
    memcpy(&Telemetry_data, body, sizeof(Telemetry_data_t));
    //Создание сообщений спутниковой системы, инерциальной и барометра
    std_msgs::Float64 baro_msg;
    sensor_msgs::NavSatFix gps_msg;
    sensor_msgs::Imu imu_msg;
    //Выставление данных шапки сообщения: Времени и фрейма
    gps_msg.header.frame_id = "Global_frame";
    gps_msg.header.stamp = ros::Time::now();
    //Заполнение полей сообщения
    gps_msg.latitude = Telemetry_data.lat;
    gps_msg.longitude = Telemetry_data.lon;
    gps_msg.altitude = Telemetry_data.ALTITUDE;
    //Выставление статуса данных спутников, если связи ссо спутниками нет статус
    //-1, если есть то 0
    if (Telemetry_data.lat == 0 || Telemetry_data.lon == 0)
      gps_msg.status.status = -1;
    else
      gps_msg.status.status = -1;
    gps_msg.status.service = 1;
    // msg.lat = Telemetry_data.lat;
    // msg.lon = Telemetry_data.lon;

    // msg.gps_height = Telemetry_data.gps_height;
    // msg.hAcc = Telemetry_data.hAcc;
    imu_msg.header.frame_id = "local_frame";
    imu_msg.header.stamp = ros::Time::now();
    tf2::Quaternion myQuaternion;
    //Перевод данных инерциальной системы из углов эйлера в кватернион
    myQuaternion.setRPY(degree_to_radian(Telemetry_data.IMU_PITCH),
                        degree_to_radian(Telemetry_data.IMU_ROLL),
                        degree_to_radian(Telemetry_data.IMU_YAW));
    myQuaternion.normalize();
    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(myQuaternion);
    //Заполнение полей сообщения телеметрии
    imu_msg.orientation = quat_msg;
    imu_msg.orientation_covariance[0] = -1;
    imu_msg.angular_velocity.x = degree_to_radian(Telemetry_data.rateX);
    imu_msg.angular_velocity.y = degree_to_radian(Telemetry_data.rateY);
    imu_msg.angular_velocity.z = degree_to_radian(Telemetry_data.rateZ);
    // imu_msg.angular_velocity.x = Telemetry_data.IMU_PITCH;
    // imu_msg.angular_velocity.y = Telemetry_data.IMU_ROLL;
    // imu_msg.angular_velocity.z = Telemetry_data.IMU_YAW;

    imu_msg.angular_velocity_covariance[0] = -1;
    // msg.rateX = Telemetry_data.rateX;
    // msg.rateY = Telemetry_data.rateY;
    // msg.rateZ = Telemetry_data.rateZ;
    // msg.ALTITUDE = Telemetry_data.ALTITUDE;
    baro_msg.data = Telemetry_data.ALTITUDE;
    //Отправка сообщений в топики РОС
    baro_pub_.publish(baro_msg);
    gps_pub_.publish(gps_msg);
    imu_pub_.publish(imu_msg);
    // msg.presAltOffsetAtGround = Telemetry_data.presAltOffsetAtGround;
    // msg.altitudeMS = Telemetry_data.ms5611_altitude;
    break;
  }
}

/*******************сборка пакет для
 * отправки*************************************/
void TelemetryHandler_ACO::ACOPacketMake(uint8_t comand, void *body,
                                         uint8_t bodylen) {
  Header_t *Hdr = (Header_t *)OutBuff;
  Hdr->id0 = ACOMAGIC1; // A
  Hdr->id1 = ACOMAGIC2; // C
  Hdr->id2 = ACOMAGIC3; // O
  Hdr->packetLen =
      bodylen + (sizeof(Header_t) - 4) +
      2; //размер полезной нагрузки плюс заголовок кроме 3 байт идентификатора
  Hdr->packetNumber = packetNumber++;
  Hdr->commandType = comand; //тип команды
  if (bodylen) //если полезная нагрузка не пустая то присоединяется полезная
               //нагрузка
    memcpy(&(OutBuff[sizeof(Header_t)]), body, bodylen);
  OutSize = sizeof(Header_t) + bodylen +
            2; //длинна всего пакета плюс 2 байта контрольной суммы
  ACO_CRC_calc(); //контрольная сумма
                  //отправить ответ
}

/*  0=выключить, 1=включить телеметрию ACO в коптере  */
void TelemetryHandler_ACO::ACOTelemOnOff(uint8_t On_Off) {
  /******************
    0 - выключить телеметрию
1 - включить телеметрию
   *****************/
  if (On_Off == 0)
    ACOPacketMake(OF_CMD_STOP, 0, 0);
  if (On_Off == 1)
    ACOPacketMake(OF_CMD_START, 0, 0);
  sendPacket();
}
void TelemetryHandler_ACO::callback_fake_gps(
    const sensor_msgs::NavSatFix &msg) {
  bool fakeGps = false;
  if (nh_->getParam("/use_gps_from_video", fakeGps)) {
    if (fakeGps)
      SetLatLon_NoGPS(msg.latitude, msg.longitude, 2);
  }
}
void TelemetryHandler_ACO::SetLatLon_NoGPS(double lat, double lon,
                                           uint8_t flags) {
  opticFlowDataStruct_t data;
  data.lat = lat;
  data.lon = lon;
  data.flags = 2;
  ACOPacketMake(OF_CMD_FLOW, &data, sizeof(opticFlowDataStruct_t));
  sendPacket();
}
void TelemetryHandler_ACO::sendPacket() { link_->sendData(OutBuff, OutSize); }
void TelemetryHandler_ACO::changeLink(AbstractLink *link) { link_ = link; }