/*
 * copa_c
 *  Приём и отправка навигационных данных коптеру и от коптера
 *  Created on: 19 сент. 2019 г.
 *      Author: Grafenkov A. V.
 */
/************************************************************************************/

#include "telemetry_handler.h"
#include "abstract_link.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*********************************** Подсчёт CRC
 * *************************************/
/*подсчёт CRC */
TelemetryHandler_ACO::TelemetryHandler_ACO(AbstractLink *link,
                                           ros::NodeHandle *nh)
    : link_(link) {

  imu_pub_ = nh->advertise<sensor_msgs::Imu>("/imu", 1000);
  gps_pub_ = nh->advertise<sensor_msgs::NavSatFix>("/gps", 1000);
  baro_pub_ = nh->advertise<std_msgs::Float32>("/baro", 1000);
}

TelemetryHandler_ACO::~TelemetryHandler_ACO() {}
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
void TelemetryHandler_ACO::parseFunc() {

  unsigned char *buf;
  int size;
  if (link_->isUp()) {
    std::tie(buf, size) = link_->getData();
    ACOParseBUF(buf, size);
    delete[] buf;
  }
}

/******************************* Парсинг буфера
 * ******************************************************/
void TelemetryHandler_ACO::ACOParseBUF(const uint8_t *buffer, size_t size) {
  uint16_t InSize = 0;
  uint8_t InBuff[ACO_BUF_MAX_LENGTH];
  for (size_t i = 0; i < size; i++)
    ACO_Parse_Byte(buffer[i], InSize, InBuff);
}
/************************** Побайтное чтение пакета **************************/
void TelemetryHandler_ACO::ACO_Parse_Byte(uint8_t byte_, uint16_t &InSize,
                                          uint8_t *InBuff) {
  switch (InSize) {
  case 0:
    if (byte_ == ACOMAGIC1) { //читаю первый байт A
      InBuff[InSize++] = byte_;
      CRC_A = 0;
      CRC_B = 0;
    };
    break;
  case 1:
    if (byte_ == ACOMAGIC2) { //читаю второй байт C
      InBuff[InSize++] = byte_;
    } else
      InSize = 0;
    break;
  case 2:
    if (byte_ == ACOMAGIC3) { //читаю третий байт O
      InBuff[InSize++] = byte_;
      CRC_A = 0;
      CRC_B = 0;
    } else
      InSize = 0;
    break;
  case 3:
    InBuff[InSize++] = byte_; //количество байт пакета - начало
    CRC_A += byte_; //тут начинаю считать контрольную сумму
    CRC_B += CRC_A;
    PackLen_ = byte_;
    /*проверяю на корректность полученной длины*/
    if ((PackLen_ < 4) || (PackLen_ > (ACO_BUF_MAX_LENGTH - 4)))
      InSize =
          0; //если размер будущего пакета не корректный то прекращаю читать
    break;
  default:
    InBuff[InSize++] = byte_;
    if (InSize < (PackLen_ + 3)) { //
      CRC_A += byte_;
      CRC_B += CRC_A;
    } else {
      if (InSize >= (PackLen_ + 4)) {
        // Packet received
        if ((InBuff[InSize - 2] == CRC_A) && (InBuff[InSize - 1] == CRC_B)) {
          PackRec((Header_t *)InBuff, &(InBuff[7]));

          InSize = 0; //сбросить счётчик, он больше не нужен
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
  case OF_CMD_FLOW: // OFM => FC это команда коптеру поэтому тут её
    //игнорирую
    break;
  case OF_CMD_TELEM:
    // FC => OFM пакет пришёл от коптера
    Telemetry_data_t Telemetry_data;
    memcpy(&Telemetry_data, body, sizeof(Telemetry_data_t));

    std_msgs::Float32 baro_msg;
    sensor_msgs::NavSatFix gps_msg;
    sensor_msgs::Imu imu_msg;
    gps_msg.header.frame_id = "Global_frame";
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.latitude = Telemetry_data.lat;
    gps_msg.longitude = Telemetry_data.lon;
    gps_msg.altitude = Telemetry_data.gps_height;
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
    myQuaternion.setRPY(Telemetry_data.IMU_PITCH, Telemetry_data.IMU_ROLL,
                        Telemetry_data.IMU_YAW);
    myQuaternion.normalize();
    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(myQuaternion);
    // or for the other conversion direction
    imu_msg.orientation = quat_msg;
    imu_msg.orientation_covariance[0] = -1;
    imu_msg.angular_velocity.x = Telemetry_data.rateX;
    imu_msg.angular_velocity.y = Telemetry_data.rateY;
    imu_msg.angular_velocity.z = Telemetry_data.rateZ;
    imu_msg.angular_velocity_covariance[0] = -1;
    // msg.rateX = Telemetry_data.rateX;
    // msg.rateY = Telemetry_data.rateY;
    // msg.rateZ = Telemetry_data.rateZ;
    // msg.ALTITUDE = Telemetry_data.ALTITUDE;
    baro_msg.data = Telemetry_data.ALTITUDE;
    baro_pub_.publish(baro_msg);
    gps_pub_.publish(gps_msg);
    imu_pub_.publish(imu_msg);
    // msg.presAltOffsetAtGround = Telemetry_data.presAltOffsetAtGround;
    // msg.altitudeMS = Telemetry_data.ms5611_altitude;
    break;
  }
}

/*******************собираю пакет для
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
  if (bodylen) //если полезная нагрузка не пустая то присоединяю полезную
               //нагрузку
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
void TelemetryHandler_ACO::sendPacket() { link_->sendData(OutBuff, OutSize); }