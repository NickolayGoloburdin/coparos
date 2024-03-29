#ifndef MOBILE_HANDLER
#define MOBILE_HANDLER
#include "abstract_link.h"
#include <list>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <stdint.h>
#include <string.h>

#define ACO_BUF_MAX_LENGTH 255
#define MOBMAGIC1 0x41
#define MOBMAGIC2 0x43
#define MOBMAGIC3 0x35
#define MOBMAGIC4 0x56
#define MOBMAGIC5 0x32
#define MOBMAGIC6 0x53
#pragma pack(push, 1)
typedef struct {
  uint8_t id0;       // 0
  uint8_t id1;       // 1
  uint8_t id2;       // 2
  uint8_t packetLen; // 3     //длина пакета с этого места
  uint16_t packetNumber;
  uint8_t commandType; // 4     //тип команды
} Mobile_Header_t;     /*5 байт всего*/
#pragma pack(pop)

class MobileHandler {
protected:
  uint16_t InSize; //номер байта входящего пакета
  uint8_t InBuff[ACO_BUF_MAX_LENGTH];
  uint8_t CRC_A;    //для подсчёта контрольной суммы
  uint8_t CRC_B;    //для подсчёта контрольной суммы
  uint8_t PackLen_; //длина пакета при приёме

private:
  /****************заголовок пакета *****************/
  void CRC_calc();
  void Parse_Byte(uint8_t byte);
  void PackRec(Mobile_Header_t *header, void *body);
  AbstractLink *link_;
  ros::Subscriber byteArray_sub_;

public:
  void parseFunc();
  MobileHandler(AbstractLink *link, ros::NodeHandle *nh);
  ~MobileHandler();
  void callback_bytes(const std_msgs::ByteMultiArray &msg);
  uint8_t OutBuff[ACO_BUF_MAX_LENGTH];
  uint8_t OutSize; //количество байт исходящего пакета
  uint16_t packetNumber = 0;
  void sendPacket();
  void ParseBUF(std::shared_ptr<std::list<unsigned char>> list_ptr);
  void PacketMake(uint8_t comand, void *body, uint8_t bodylen);
};

#endif // ABSTRACT_LINK_H