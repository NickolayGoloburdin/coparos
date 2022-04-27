#ifndef TELEMETRY_HANDLER_H
#define TELEMETRY_HANDLER_H
#include "abstract_link.h"
#include <coparos/Telemetry.h>
#include <cstdint>
#include <list>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <stdint.h>
#include <string.h>
#include <thread>

#define ACO_BUF_MAX_LENGTH 255
#define ACOMAGIC1 0x41
#define ACOMAGIC2 0x43
#define ACOMAGIC3 0x4f

enum opticFlowCmdTypes {
  OF_CMD_ACK = 0x00,
  OF_CMD_FLOW,         // OFM => FC
  OF_CMD_TELEM = 0x03, // FC => OFM
  OF_CMD_START,
  OF_CMD_STOP
};

#pragma pack(push, 1)
typedef struct {
  uint8_t id0;       // 0
  uint8_t id1;       // 1
  uint8_t id2;       // 2
  uint8_t packetLen; // 3     //длина пакета с этого места
  uint16_t packetNumber;
  uint8_t commandType; // 4     //тип команды
} Header_t;            /*5 байт всего*/
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  double lat;       // 5-12
  double lon;       // 13-20
  float gps_height; // 21-24
  float hAcc;       // 25-28  // - точность  СКО у ГНСС)
  float IMU_PITCH;  // 29-32
  float IMU_ROLL;   // 33-36
  float IMU_YAW;    // 37-40
  float rateX;      // 41-44
  float rateY;      // 45-48
  float rateZ;      // 49-25
  float ALTITUDE; // 53-56 // - интегральная высота  баро или гнсс)
  float presAltOffsetAtGround; // 57-60 // - высота в точке взлёта  ноль
                               // высоты msl)
  float ms5611_altitude; // 61-64//- высота чисто по баро
} Telemetry_data_t;
#pragma pack(pop)

class TelemetryHandler_ACO {
protected:          //номер байта входящего пакета
  uint8_t CRC_A;    //для подсчёта контрольной суммы
  uint8_t CRC_B;    //для подсчёта контрольной суммы
  uint8_t PackLen_; //длина пакета при приёме

private:
  /****************заголовок пакета *****************/
  void ACO_CRC_calc(); //Подсчет контрольной суммы
  void ACO_Parse_Byte(uint8_t byte, uint16_t &InSize,
                      uint8_t *InBuff); //Парсер пакета
  void PackRec(Header_t *header,
               void *body); //Метод обработки полученного пакета
  AbstractLink *link_; //Интерфейс предачи данных

  ros::Publisher imu_pub_; //Издатель в РОС, который посылает в топик значения
                           //инерциальной системы
  ros::Publisher
      gps_pub_; //Издатель в РОС, который посылает в топик gps координаты
  ros::Publisher
      baro_pub_; //Издатель в РОС, который посылает в топик значения барометра
  ros::Publisher
      debug_pub_;

public:
  void parseFunc();
  TelemetryHandler_ACO(AbstractLink *link, ros::NodeHandle *nh);
  ~TelemetryHandler_ACO();
  uint8_t
      OutBuff[ACO_BUF_MAX_LENGTH]; //буфер для формирования исходящего пакета
  uint8_t OutSize; //количество байт исходящего пакета
  uint16_t packetNumber = 0; //Номера исходящего пакета

  void sendPacket(); //Отправить пакет
  void ACOParseBUF(std::shared_ptr<std::list<unsigned char>> list_ptr);
  void SetLatLon_NoGPS(); //Отправить подмену координат
  void ACOTelemOnOff(
      uint8_t On_Off); // 0=выключить, 1=включить телеметрию ACO в коптере
  void ACOPacketMake(uint8_t comand, void *body,
                     uint8_t bodylen); //Конструктор пакета для отправки
};

#endif // ABSTRACT_LINK_H