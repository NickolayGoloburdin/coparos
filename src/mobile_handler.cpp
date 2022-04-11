/*
 * copa_c
 *  Приём и отправка навигационных данных коптеру и от коптера
 *  Created on: 19 сент. 2019 г.
 *      Author: Grafenkov A. V.
 */
/************************************************************************************/

#include "mobile_handler.h"
#include "abstract_link.h"
#include "copa_types.h"
/*********************************** Подсчёт CRC
 * *************************************/
/*подсчёт CRC */
MobileHandler::MobileHandler(AbstractLink *link, ros::NodeHandle *nh)
    : link_(link) {
  byteArray_sub_ =
      nh->subscribe("/rowBytes", 1000, &MobileHandler::callback_bytes, this);
}

MobileHandler::~MobileHandler() {}
void MobileHandler::CRC_calc() {
  uint8_t CRC_A = 0;
  uint8_t CRC_B = 0;
  for (int i = 3; i < (OutSize - 2); i++) {
    CRC_A += OutBuff[i];
    CRC_B += CRC_A;
  }
  OutBuff[OutSize - 2] = CRC_A;
  OutBuff[OutSize - 1] = CRC_B;
}
void MobileHandler::parseFunc() {
  unsigned char *buf;
  int size;
  if (link_->isUp()) {
    std::tie(buf, size) = link_->getData();
    if (size > 0) {
      ParseBUF(buf, size);
      delete[] buf;
    }
  }
}
void MobileHandler::callback_bytes(const std_msgs::ByteMultiArray &msg) {
  int size = msg.data.size();
  unsigned char *buffer = new unsigned char[size];
  for (int i = 0; i < size; ++i) {
    buffer[i] = static_cast<unsigned char>(msg.data[i]);
  }
  link_->sendData(buffer, size);
  delete[] buffer;
}
/******************************* Парсинг буфера
 * ******************************************************/
void MobileHandler::ParseBUF(const uint8_t *buffer, size_t size) {
  for (size_t i = 0; i < size; i++)
    Parse_Byte(buffer[i]);
}
/************************** Побайтное чтение пакета **************************/
void MobileHandler::Parse_Byte(uint8_t byte_) {
  switch (InSize) {
  case 0:
    if (byte_ == MOBMAGIC1 || byte_ == MOBMAGIC4) { //читаю первый байт A
      InBuff[InSize++] = byte_;
      CRC_A = 0;
      CRC_B = 0;
    };
    break;
  case 1:
    if (byte_ == MOBMAGIC2 || byte_ == MOBMAGIC5) { //читаю второй байт C
      InBuff[InSize++] = byte_;
    } else
      InSize = 0;
    break;
  case 2:
    if (byte_ == MOBMAGIC3 || byte_ == MOBMAGIC6) { //читаю третий байт O
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
          PackRec((Mobile_Header_t *)InBuff, &(InBuff[7]));

          InSize = 0; //сбросить счётчик, он больше не нужен
        };
      }
    }
    break;
  };
}

/************************** Парсинг принятого пакета **************************/

void MobileHandler::PackRec(Mobile_Header_t *header, void *body) {
  switch (header->commandType) {
  case CMD_DEVICE_INFO: {
    uint16_t data = 1;
    PacketMake(CMD_TEXT, &data, 2);
    sendPacket();
    break;
  }
  case CMD_PING:
    break;
  case CMD_SYSTEM_RESET:
    break;
  default:
    printf("coomand is %d", header->commandType);
    break;
  }
}

/*******************собираю пакет для
 * отправки*************************************/
void MobileHandler::PacketMake(uint8_t comand, void *body, uint8_t bodylen) {
  Mobile_Header_t *Hdr = (Mobile_Header_t *)OutBuff;
  Hdr->id0 = MOBMAGIC4;
  Hdr->id1 = MOBMAGIC5;
  Hdr->id2 = MOBMAGIC6;
  Hdr->packetLen =
      bodylen + (sizeof(Mobile_Header_t) - 4) +
      2; //размер полезной нагрузки плюс заголовок кроме 3 байт идентификатора
  Hdr->packetNumber = packetNumber++;
  Hdr->commandType = comand; //тип команды
  if (bodylen) //если полезная нагрузка не пустая то присоединяю полезную
               //нагрузку
    memcpy(&(OutBuff[sizeof(Mobile_Header_t)]), body, bodylen);
  OutSize = sizeof(Mobile_Header_t) + bodylen +
            2; //длинна всего пакета плюс 2 байта контрольной суммы
  CRC_calc();  //контрольная сумма
               //отправить ответ
}

/*  0=выключить, 1=включить телеметрию ACO в коптере  */

void MobileHandler::sendPacket() { link_->sendData(OutBuff, OutSize); }