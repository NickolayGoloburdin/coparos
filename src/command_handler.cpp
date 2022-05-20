/*
 * command_handler.cpp
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */
#include "command_handler.h"
#include "abstract_link.h"
#include <coparos/Ack.h>
#include <coparos/DroneInfo.h>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <vector>

//#include <stdlib.h>

/***************************задать начальные параметры при первой
 * загрузке****************************/
COPA::COPA(AbstractLink *link, ros::NodeHandle *nh) : link_(link) {
  uint16_t tmp = 0x0001;
  BigEndian = 1 - *((uint8_t *)&tmp);
  FC_ID = 0x0fff;
  GCS_ID = 0x4001;
  NewTarget = 0;
  /*****************Инициализация издателей для отправки данных в топики
   * РОС*******/
  ack_pub_ = nh->advertise<coparos::Ack>("/ack", 1000);
  // byteArray_pub_ = nh->advertise<std_msgs::ByteMultiArray>("/rowBytes",
  // 1000);
  status_pub_ = nh->advertise<std_msgs::String>("/status", 1000);
  drone_info_pub_ = nh->advertise<coparos::DroneInfo>("/droneInfo", 1000);
  mission_point_request_pub_ =
      nh->advertise<std_msgs::Int16>("/mission_request", 1000);
  command_sub_ = nh->subscribe("/command", 1000, &COPA::callback_command, this);
  mission_point_sub_ =
      nh->subscribe("/missionPoint", 1000, &COPA::callback_mission_point, this);
  nh->getParam("/takeoff_height", takeoff_height);
}
COPA::~COPA() {}
//Метод обработчик входящих данных из топика команд
void COPA::callback_command(const coparos::Command &msg) {
  switch (msg.command) {
  case CMD_NAV_MOTORS_ON:
    Copa_Motors_On();
    break;
  case CMD_NAV_MOTORS_OFF:
    Copa_Motors_Off();
    break;
  case CMD_NAV_TAKE_OFF:
    Copa_Take_Off(takeoff_height);
    break;
  case CMD_NAV_TO_LAND: {
    landBody_t body = {0, 15, 4, 8};
    Copa_To_Land(&body);
    break;
  }
  case CMD_NAV_GOTO_HOME:
    Copa_GoTo_Home();
    break;
  case CMD_NAV_STOP_MOTION:
    Copa_StopMotion();
    break;
  case CMD_NAV_CONTINUE_MOTION:
    Copa_ContinueMotion();
    break;
  case CMD_NAV_CLEAR_WP:
    Miss_Clear();
    break;
  case CMD_EXEC_SWITCH_N:
    CopaExecSwitch_N(1);
  case CMD_SET_MAN_TARGET_ANGLES:
    CopaSetAngles(msg.data1, msg.data2);
    break;
  case CMD_SET_NAV_MODE:
    CopaSetNavMode(uint8_t(msg.data1));
    break;
  case CMD_NAV_SET_ABS_HEADING:
    CopaSetHeading(msg.data1, msg.data2);
    break;
  case CMD_GNSS_USE:
    SetGnss(uint8_t(msg.data1));
    break;
  }
}

//Метод обработчик точек миссий из топика
void COPA::callback_mission_point(const coparos::MissionPoint &msg) {
  sMissionPoint point;
  point.targetLat = msg.targetLat;
  point.targetLon = msg.targetLon;
  point.targetAlt = msg.targetAlt;
  point.targetRadius = msg.targetRadius;
  point.loiterTime = msg.loiterTime;
  point.maxHorizSpeed = msg.maxHorizSpeed;
  point.maxVertSpeed = msg.maxVertSpeed;
  point.poiLat = msg.poiLat;
  point.poiLon = msg.poiLon;
  point.poiHeading = msg.poiHeading;
  point.poiAltitude = msg.poiAltitude;
  point.flags = msg.flags;
  point.photo = msg.photo;
  point.panoSectorsCount = msg.panoSectorsCount;
  point.panoDeltaAngle = msg.panoDeltaAngle;
  point.poiPitch = msg.poiPitch;
  point.poiRoll = msg.poiRoll;
  point.type = msg.type;
  Mission_Up(&point);
}
//Метод парсинга данных с модуля связи
void COPA::parseFunc() {

  if (link_->isUp()) {
    auto list_ptr = link_->getData();
    if (list_ptr)
      CopaParseBUF(list_ptr);
  }
}

/******************************* Парсинг буфера
 * ******************************************************/
void COPA::CopaParseBUF(std::shared_ptr<std::list<unsigned char>> list_ptr) {
  for (auto ptr = list_ptr->begin(); ptr != list_ptr->end(); ptr++)
    CopaParseByte(*ptr);
}
/**********************************Побайтное чтение пакета с
 * коптера**********************************/
void COPA::CopaParseByte(uint8_t b_) {
  uint16_t PackLen_;
  switch (ParseInSize) {
  case 0:
    if (b_ == 0x41) { //читаю первый байт
      InBuff[ParseInSize++] = b_;
      InCRC_A = 0;
      InCRC_B = 0;
    };
    break;
  case 1:
    if (b_ == 0x43) { //читаю второй байт
      InBuff[ParseInSize++] = b_;
    } else
      ParseInSize = 0;
    break;
  case 2:
    if (b_ == 0x35) { //читаю третий байт
      InBuff[ParseInSize++] = b_;
      CopaActual_flag_on(
          ACTUAL_SPEED_UART); // UART принял правильный пакет приостановить
                              // сканирование скорости порта
    } else if (b_ == 0x4f) { //читаю третий байт
      CopaActual_flag_on(
          ACTUAL_SPEED_UART); // UART принял правильный пакет приостановить
                              // сканирование скорости порта
      // stat_rx = 2;
    } else
      ParseInSize = 0;
    break;
  case 3:
    InBuff[ParseInSize++] = b_; //количество байт пакета - начало
    InCRC_A += b_; //Подчсет контрольной суммы
    InCRC_B += InCRC_A;
    break;
  case 4:
    InBuff[ParseInSize++] = b_; //количество байт пакета - конец
    InCRC_A += b_;
    InCRC_B += InCRC_A;
    PackLen_ = InBuff[3]; //количество байт пакета из буфера - начало
    PackLen_ += InBuff[4] << 8; //количество байт пакета из буфера - конец
    // PackLen_ = e_uint16(((sCoptHdr*)InBuff)->packetLen);
    if ((PackLen_ < 14) || (PackLen_ > (CMD_CONTROL_MAX_PACKET_SIZE - 14)))
      ParseInSize =
          0; //если размер будущего пакета не корректный то закончить парсинг
    break;
  default:
    InBuff[ParseInSize++] = b_;
    PackLen_ = InBuff[3];
    PackLen_ += InBuff[4] << 8;
    // PackLen_ = e_uint16(((sCoptHdr*)InBuff)->packetLen);//количество байт
    // пакета - конец
    if (ParseInSize < (PackLen_ + 4)) {
      InCRC_A += b_;
      InCRC_B += InCRC_A;
    } else {
      if (ParseInSize >= (PackLen_ + 5)) {
        // Packet received
        if ((InBuff[ParseInSize - 2] == InCRC_A) &&
            (InBuff[ParseInSize - 1] == InCRC_B)) {
          // stat_rx = 1;
          InSize = ParseInSize;
          PacketReceived((sCoptHdr *)InBuff, &(InBuff[19]));
        };
        ParseInSize = 0;
      }
    }
    break;
  };
};

/**********************************парсинг входящего
 * пакета**********************************/
void COPA::PacketReceived(sCoptHdr *header, void *body) {

  switch (e_uint16(header->commandType)) {
  case CMD_ACK: {
    coparos::Ack msg;
    sAckBody AckBody;
    memcpy(&AckBody, body, sizeof(sAckBody));
    msg.command = AckBody.commandType;
    msg.result = true;
    ack_pub_.publish(msg);
    switch (e_uint16(AckBody.commandType)) {
    case CMD_TELEM_DISABLE: { //команда отключить телеметрию выполнена
      presetStatSet++;
      // Preset_Set_Param();

      break;
    }
    case CMD_TELEM_ENABLE: //команда включить телеметрию выполнена
      presetStatSet++;
      if (presetStatSet >= 5)
        CopaActual_flag_on(ACTUAL_SET_PRESET);

      // Preset_Set_Param();
      break;
    case CMD_TELEM_WRITE_CONFIG: //команда обновления пресета телеметрии
                                 //выполнена
      presetStatSet++;

      // Preset_Set_Param();

      break;
    case CMD_PING: //получен ответ от коптера на такую же команду CMD_PING,
                   //значит коптер на связи

      break;
    case CMD_NAV_CLEAR_WP: { //Команда очистить миссию в коптере выполнена
      Mission_Nup = 0;
      break;
    }
    case CMD_NAV_WRITE_POINT: { // Команда записать точку в коптер выполнена
      std_msgs::Int16 msg;
      Mission_Nup++;
      msg.data = Mission_Nup;
      mission_point_request_pub_.publish(msg);

      break;
    }
    default:

      break;
    }
    break;
  }
  case CMD_FC_TELEM_PRESET_BITMASK: { //ответ с битовой маской настроеного
                                      //пресета
    presetStatSet++;
  }
  case CMD_NAV_MOTORS_ON_ANSWER: {
    coparos::Ack msg; //Это команда, подтверждающая получения пакета с ошибочной
    uint32_t bitmask;
    // memcpy(&NAckBody, body, sizeof(sNAckBody));
    bitmask = *(reinterpret_cast<uint32_t *>(body));
    msg.command = CMD_NAV_MOTORS_ON;
    switch (bitmask) {
    case ERR_PCBM_NONE: //Флаг стабильной работы системы
      msg.result = true;
      msg.status = "System ready to start";
      break;
    case ERR_PCBM_RADIO_QUALITY: //Флаг ошибки радиосвязи
      msg.result = true;
      msg.status = "No fly transmitter";
      break;
    case ERR_PCBM_THRO_STATE: //Флаг ошибки стика пульта
      msg.result = true;
      msg.status = "Throttle failsafe";
      break;
    case ERR_PCBM_NAVI_SWITCH: //Флаг ошибки системы навигации
      msg.result = true;
      msg.status = "Transmitter stick in wrong mode";
      break;
    case ERR_PCBM_ALT_SWITCH: //Флаг ошибки барометра
      msg.result = true;
      msg.status = "Transmitter stick in wrong hold pos";
      break;
    case ERR_PCBM_BATTERY_LOW: //Флаг малого заряда батареи
      msg.result = false;
      msg.status = "Low battery";
      break;
    case ERR_PCBM_IMU_SENSORS: //Флаг ошибки инерциальной системы
      msg.result = false;
      msg.status = "IMU error";
      break;
    case ERR_PCBM_PITCH_ROLL: //Флаг ошибки стиков пульта
      msg.result = true;
      msg.status = "Transmitter stick in wrong pitch/roll";
      break;
    case ERR_PCBM_YAW: //Флаг ошибки радиосвязи
      msg.result = true;
      msg.status = "Transmitter stick in wrong yaw";
      break;
    case ERR_PCBM_MAG_X: //Флаг ошибки магнетометра
      msg.result = true;
      msg.status = "Compass error x";
      break;
    case ERR_PCBM_MAG_Y: //Флаг ошибки магнетометра
      msg.result = true;
      msg.status = "Compass error y";
      break;
    case ERR_PCBM_MAG_Z: //Флаг ошибки магнетометра
      msg.result = true;
      msg.status = "Compass error z";
      break;
    case ERR_PCBM_BARO: //Флаг ошибки барометра
      msg.result = false;
      msg.status = "Baro error";
      break;
    case ERR_PCBM_CD_CARD: //Флаг ошибки карты памяти
      msg.result = true;
      msg.status = "CD card error";
      break;
    case ERR_PCBM_DATA_LINK_LOST: //Флаг ошибки плохого канала связи
      msg.result = true;
      msg.status = "Ling gcs lost";
      break;
    }

    ack_pub_.publish(msg);
    break;
  }
  case CMD_DEVICE_INFO: //Информация о дроне
    presetStatSet++;
    // сканирование скорости порта
    Copa_Cmd_Ack(header);
    memcpy(&DeviceInfo, body, sizeof(sCoptHdr));
    break;
  case CMD_TELEM_PARAMS_COUNT: //Количество параметров телеметрии
    presetStatSet++;
    Copa_Cmd_Ack(header);
    memcpy(&telemCount, body, sizeof(telemCount));
    break;
  case CMD_PING: //получен ответ от коптера на такую же команду CMD_PING,
                 //значит коптер на связи

    break;
  case CMD_NACK: {
    coparos::Ack msg; //Ошибка обработки команды коптером
    sNAckBody NAckBody;
    memcpy(&NAckBody, body, sizeof(sNAckBody));
    msg.command = NAckBody.commandType;
    msg.result = false;
    msg.status = std::string(NAckBody.text);
    ack_pub_.publish(msg); // Отправка команды в топик РОС

    break;
  }
  case CMD_TEXT: {
    std_msgs::String
        text_msg; //Текстовое сообщение (в параметрах передается текстовое
                  //сообщение его размер определяется из размера пакета)
    Copa_Cmd_Ack(header);
    text_msg.data = std::string(reinterpret_cast<char *>(body),
                                size_t(header->packetLen - 16));
    // text_msg.data = text;
    status_pub_.publish(text_msg);

    break;
  }
  case TELEMETRY_DATA: {
    coparos::DroneInfo droneinfo_msg; //Пакет телеметрии телеметрия
    LowSpeedTelemetry TelemData;
    if (e_uint16(header->packetLen) - 16 == sizeof(TelemData)) {
      memcpy(&TelemData, body, sizeof(LowSpeedTelemetry));
      //Формирование сообщения телеметрии для отправки в топик РОС
      droneinfo_msg.preset_number = TelemData.preset_number;
      droneinfo_msg.VBAT = TelemData.VBAT;
      droneinfo_msg.STATE = TelemData.STATE;
      droneinfo_msg.FLAGS = TelemData.FLAGS;
      droneinfo_msg.ABSOLUTE_HEADING = TelemData.ABSOLUTE_HEADING;
      droneinfo_msg.ALTITUDE = TelemData.ALTITUDE;
      droneinfo_msg.ALTITUDE_HOLD_SPEED = TelemData.ALTITUDE_HOLD_SPEED;
      droneinfo_msg.GPS_ALT = TelemData.GPS_ALT;
      droneinfo_msg.GPS_LAT = TelemData.GPS_LAT;
      droneinfo_msg.GPS_LON = TelemData.GPS_LON;
      droneinfo_msg.GPS_HACC = TelemData.GPS_HACC;
      droneinfo_msg.GPS_HDOP = TelemData.GPS_HDOP;
      droneinfo_msg.GPS_SATELLITES_FIX = TelemData.GPS_SATELLITES_FIX;
      droneinfo_msg.GPS_NUMBER_OF_SATELLITES =
          TelemData.GPS_NUMBER_OF_SATELLITES;

      drone_info_pub_.publish(droneinfo_msg);
    }

    break;
  }
  case CMD_NAV_WP_COUNT: //Получен номер точчки полетного задания
    Copa_Cmd_Ack(header);
    if (Mission_stat == MISS_GET_POINT_COUNT_P) {
      memcpy(&Mission_count, body, sizeof(Mission_count));
      Mission_stat = MISS_DOWNLOAD_P;
      Mission_Dn();
    }
    if (Mission_stat == MISS_GET_POINT_COUNT) {
      memcpy(&Mission_count, body, sizeof(Mission_count));
      Mission_stat = MISS_DOWNLOAD;
      Mission_Dn();
    }
    break;
  case CMD_NAV_POINT_INFO: // Получена информация о точки полетного задания
                           // Copa_Cmd_Ack(header);
                           // Point_info_t Point_info;
                           // memcpy(&Point_info, body, sizeof(Point_info_t));
                           // Mission_Ndn++; //Прочитана очередная точка

    // if (Mission_compare_p(&Point_info) == 1) {
    //   Mission_Ndn++; //Прочитана очередная точка

    //   if (Mission_Ndn ==
    //       sizeof(Target_Mission)) { // если миссия полностью
    //                                 // прочитана то запускаю коптер
    //     Mission_stat = MISS_START;
    //   }
    // } else {
    //   Mission_stat = MISS_DEF;
    // }

    // break;
  default:
    break;
  }
}

// Big endian compatibility
// https://class-of-java.blogspot.com/2014/09/big-endian-vs-little-endian-byte.html
//перевернуть байты
float COPA::e_float(float t) {
  float retrn;
  if (BigEndian) {
    uint8_t *in = (uint8_t *)&t;
    uint8_t *out = (uint8_t *)&retrn;
    out[0] = in[3];
    out[1] = in[2];
    out[2] = in[1];
    out[3] = in[0];
  } else
    retrn = t;
  return retrn;
};
//------------------------------------------------
double COPA::e_double(double t) {
  double retrn;
  if (BigEndian) {
    uint8_t *in = (uint8_t *)&t;
    uint8_t *out = (uint8_t *)&retrn;
    out[0] = in[7];
    out[1] = in[6];
    out[2] = in[5];
    out[3] = in[4];
    out[4] = in[3];
    out[5] = in[2];
    out[6] = in[1];
    out[7] = in[0];
  } else
    retrn = t;
  return retrn;
};
//-------------------------------------------
uint16_t COPA::e_uint16(uint16_t t) {
  uint16_t retrn;
  if (BigEndian) {
    uint8_t *in = (uint8_t *)&t;
    uint8_t *out = (uint8_t *)&retrn;
    out[0] = in[1];
    out[1] = in[0];
  } else
    retrn = t;
  return retrn;
}
//-------------------------------------------
uint32_t COPA::e_uint32(uint32_t t) {
  uint32_t retrn;
  if (BigEndian) {
    uint8_t *in = (uint8_t *)&t;
    uint8_t *out = (uint8_t *)&retrn;
    out[0] = in[3];
    out[1] = in[2];
    out[2] = in[1];
    out[3] = in[0];
  } else
    retrn = t;
  return retrn;
}

/**********************************команда CMD_AC
 * подтверждения**********************************/
void COPA::Copa_Cmd_Ack(sCoptHdr *header) {
  sAckBody AckBody_;
  AckBody_.commandNumber = header->commandNumber;
  AckBody_.commandType = header->commandType;
  AckBody_.packetNumber = header->packetNumber;
  CopaPacketMake(CMD_ACK, &AckBody_, sizeof(sAckBody));
  CopaPacketSend();
}

/**********************************запросить параметры
 * телеметрии.**********************************/
void COPA::GetTelemConfig() {
  switch (ParamTelemStat) {
  case 0:
    CopterState = COPTSTATE_BUSYINIT;
    CopaPacketMake(CMD_TELEM_GET_PARAMS_COUNT, 0, 0);
    CopaPacketSend();
    ParamTelemStat++;
    break;
  case 1:
    ParamTelemStat++;
    break;
  }
}
//-------------------------------------------
/**********************************перезагрузить
 * коптер**********************************/
void COPA::copa_reset() {
  CopaPacketMake(CMD_SYSTEM_RESET, 0, 0); //
}
/**********************************Заполнить битовую маску для
 * пресета**********************************/
void COPA::SetParamPreset() {

  preset_bits_clear(0); //очистить пресет 1

  preset_bit_set(TELEM_VBAT, 0);             // 0
  preset_bit_set(TELEM_STATE, 0);            // 15
  preset_bit_set(TELEM_FLAGS, 0);            // 16
  preset_bit_set(TELEM_ABSOLUTE_HEADING, 0); // 38
  preset_bit_set(TELEM_ALTITUDE, 0);         // 47
  preset_bit_set(TELEM_ALTITUDE_HOLD_SPEED, 0);
  preset_bit_set(TELEM_GPS_ALT, 0); // 67
                                    // 51
  preset_bit_set(TELEM_GPS_LAT, 0); // 67
  preset_bit_set(TELEM_GPS_LON, 0); // 68
  preset_bit_set(TELEM_GPS_HACC, 0); // 70 точность определения
  preset_bit_set(TELEM_GPS_HDOP, 0); // 75
  preset_bit_set(TELEM_GPS_SATELLITES_FIX, 0); // 92  смотреть качество РТК
  preset_bit_set(TELEM_GPS_NUMBER_OF_SATELLITES, 0); // 93

  presetParam[0].presetN = 0; // Номер пресета.
  presetParam[0].rate = e_uint16(50); //Значение, определяющее как часто
                                      //отправлять телеметрию этого пресета.
  presetParam[0].offset = 0; //Значение, определяющее с каким сдвигом отправлять
                             //телеметрию этого пресета.

  for (uint16_t i = 0; i < TELEMETRY_PRESET_BITMASK_LENGTH; i++) {
    presetParam[0].bitmask[i] = preset_param_key[0][i];
  }
}
/**********************************настроить пресет 1 для
 * телеметрии**********************************/
void COPA::Preset_Set_Param() {

  CopaPacketMake(CMD_TELEM_DISABLE, 0, 0);
  CopaPacketSend();
  //отправить новые настройки телеметрии
  SetParamPreset(); //Заполнить битовую маску для пресета
  CopaPacketMake(CMD_TELEM_WRITE_CONFIG, &presetParam[0],
                 sizeof(telemetryParam_t_));
  CopaPacketSend();
  uint32_t presmask;
  presmask = e_uint32((uint32_t)1); // 1 - активен только первый пресет

  CopaPacketMake(CMD_TELEM_ENABLE, &presmask, sizeof(presmask));
  CopaPacketSend();
}
//-------------------------------------------
/*Тестовая посылка для проверки канала связи и общего рабочего состояния
 * устройств.*/
void COPA::Copa_Ping() { /*Ответ на тестовый пакет.
 Ни в запросе ни в ответе нет параметров.*/
  // FC_ID = 0x0fff;  GCS_ID = 0x4001;
  CopaPacketMake(CMD_PING, 0, 0);
  CopaPacketSend();
}
//-------------------------------------------
/**********************************Тестовый
 * пакет.**********************************/
void COPA::Copa_Pool() { /*Тестовый пакет, по которому САУ делает выводы
         (например, активизировать режим "FailSafe", если за последнюю минуту не
         пришло ни одного пакета) о наличии и качестве канала управления от НСУ.
         Частота пакетом выбирается исходя из возможностей канала связи.*/
  CopaPacketMake(CMD_POOL, 0, 0);
  CopaPacketSend();
}
/**********************************Предстартовая проверка готовности системы к
 * взлёту.**********************************/
void COPA::Copa_PreStartCheck(uint32_t AltMap_Pstart_) {
  /*Высота в точке старта по карте высот.
   * Этот параметр надо запомнить для вычисления
   * высоты посадки при передаче управления на
   * другой компьютер
   * */

  CopaPacketMake(CMD_NAV_PRE_START_CHECK, &AltMap_Pstart_, 4);
  CopaPacketSend();
}
telemetryFields_t telemetryFields[] = {
    {TELEM_VBAT, TELEM_TYPE_FLOAT},
    {TELEM_CURRENT_A, TELEM_TYPE_FLOAT},
    {TELEM_POWER_WATT, TELEM_TYPE_FLOAT},
    {TELEM_CAPACITY_MAH, TELEM_TYPE_FLOAT},
    {TELEM_ENERGY_WH, TELEM_TYPE_FLOAT},
    {TELEM_RADIO_QUALITY, TELEM_TYPE_FLOAT},
    {TELEM_CPU_LOAD, TELEM_TYPE_FLOAT},
    {TELEM_FLIGHT_TIME, TELEM_TYPE_FLOAT},
    {TELEM_FLIGHT_TIME_REMAINING, TELEM_TYPE_FLOAT},
    {TELEM_FLIGHT_TIME_SOC, TELEM_TYPE_FLOAT},
    {TELEM_BAT_CELL_COUNT, TELEM_TYPE_U8},
    {TELEM_ANGLE_PITCH, TELEM_TYPE_FLOAT},
    {TELEM_ANGLE_ROLL, TELEM_TYPE_FLOAT},
    {TELEM_EXT_TEMP, TELEM_TYPE_FLOAT},
    {TELEM_TEMP, TELEM_TYPE_FLOAT},
    {TELEM_STATE, TELEM_TYPE_U16},
    {TELEM_FLAGS, TELEM_TYPE_U16},
    {TELEM_GYRO_RAW_X_ROLL, TELEM_TYPE_FLOAT},
    {TELEM_GYRO_RAW_Y_PITCH, TELEM_TYPE_FLOAT},
    {TELEM_GYRO_RAW_Z_Yaw, TELEM_TYPE_FLOAT},
    {TELEM_GYRO_PITCH, TELEM_TYPE_FLOAT},
    {TELEM_GYRO_ROLL, TELEM_TYPE_FLOAT},
    {TELEM_GYRO_YAW, TELEM_TYPE_FLOAT},
    {TELEM_GYRO_TEMP1, TELEM_TYPE_FLOAT},
    {TELEM_GYRO_TEMP2, TELEM_TYPE_FLOAT},
    {TELEM_ACC_RAW_X, TELEM_TYPE_FLOAT},
    {TELEM_ACC_RAW_Y, TELEM_TYPE_FLOAT},
    {TELEM_ACC_RAW_Z, TELEM_TYPE_FLOAT},
    {TELEM_ACC_X, TELEM_TYPE_FLOAT},
    {TELEM_ACC_Y, TELEM_TYPE_FLOAT},
    {TELEM_ACC_Z, TELEM_TYPE_FLOAT},
    {TELEM_ACC_BIAS_X, TELEM_TYPE_FLOAT},
    {TELEM_ACC_BIAS_Y, TELEM_TYPE_FLOAT},
    {TELEM_ACC_BIAS_Z, TELEM_TYPE_FLOAT},
    {TELEM_ACC_STD_Z, TELEM_TYPE_FLOAT},
    {TELEM_MAG_X, TELEM_TYPE_FLOAT},
    {TELEM_MAG_Y, TELEM_TYPE_FLOAT},
    {TELEM_MAG_Z, TELEM_TYPE_FLOAT},
    {TELEM_ABSOLUTE_HEADING, TELEM_TYPE_FLOAT},
    {TELEM_HOLD_HEADING, TELEM_TYPE_FLOAT},
    {TELEM_ACTUAL_HEADING, TELEM_TYPE_FLOAT},
    {TELEM_BARO1_ADC, TELEM_TYPE_FLOAT},
    {TELEM_BARO2_ADC, TELEM_TYPE_FLOAT},
    {TELEM_BARO1_ALT, TELEM_TYPE_FLOAT},
    {TELEM_BARO2_ALT, TELEM_TYPE_FLOAT},
    {TELEM_BARO_MS5611_ALT, TELEM_TYPE_FLOAT},
    {TELEM_BARO_MS5611_TEMP, TELEM_TYPE_FLOAT},
    {TELEM_ALTITUDE, TELEM_TYPE_FLOAT},
    {TELEM_ALTITUDE_ABS, TELEM_TYPE_FLOAT},
    {TELEM_ALTITUDE_HOLD, TELEM_TYPE_FLOAT},
    {TELEM_GPS_ALT, TELEM_TYPE_FLOAT},
    {TELEM_ALTITUDE_HOLD_SPEED, TELEM_TYPE_FLOAT},
    {TELEM_ALTITUDE_VELOCITY, TELEM_TYPE_FLOAT},
    {TELEM_ACTUAL_ALT_VELOCITY, TELEM_TYPE_FLOAT},
    {TELEM_BARO_ALT_VELOCITY, TELEM_TYPE_FLOAT},
    {TELEM_RANGE_SENSOR1, TELEM_TYPE_FLOAT},
    {TELEM_RANGE_STD1, TELEM_TYPE_FLOAT},
    {TELEM_RANGE_SENSOR2, TELEM_TYPE_FLOAT},
    {TELEM_RANGE_STD2, TELEM_TYPE_FLOAT},
    {TELEM_MILEAGE, TELEM_TYPE_FLOAT},
    {TELEM_MILEAGE_TOTAL, TELEM_TYPE_FLOAT},
    {TELEM_ENERGY_EFFICIENT, TELEM_TYPE_FLOAT},
    {TELEM_AMP_EFFICIENT, TELEM_TYPE_FLOAT},
    {TELEM_POSITION_NORTH, TELEM_TYPE_FLOAT},
    {TELEM_POSITION_EAST, TELEM_TYPE_FLOAT},
    {TELEM_VELOCITY_NORTH, TELEM_TYPE_FLOAT},
    {TELEM_VELOCITY_EAST, TELEM_TYPE_FLOAT},
    {TELEM_GPS_LAT, TELEM_TYPE_DOUBLE},
    {TELEM_GPS_LON, TELEM_TYPE_DOUBLE},
    {TELEM_GPS_GOOD_FLAG, TELEM_TYPE_U8},
    {TELEM_GPS_HACC, TELEM_TYPE_FLOAT},
    {TELEM_GPS_VACC, TELEM_TYPE_FLOAT},
    {TELEM_GPS_VELD, TELEM_TYPE_FLOAT},
    {TELEM_GPS_HEADING, TELEM_TYPE_FLOAT},
    {TELEM_GPS_PDOP, TELEM_TYPE_FLOAT},
    {TELEM_GPS_HDOP, TELEM_TYPE_FLOAT},
    {TELEM_GPS_VDOP, TELEM_TYPE_FLOAT},
    {TELEM_GPS_TDOP, TELEM_TYPE_FLOAT},
    {TELEM_GPS_NDOP, TELEM_TYPE_FLOAT},
    {TELEM_GPS_EDOP, TELEM_TYPE_FLOAT},
    {TELEM_GPS_GDOP, TELEM_TYPE_FLOAT},
    {TELEM_HOLD_COURSE, TELEM_TYPE_FLOAT},
    {TELEM_HOLD_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_HOME_LAT, TELEM_TYPE_DOUBLE},
    {TELEM_HOME_LON, TELEM_TYPE_DOUBLE},
    {TELEM_HOME_ALT, TELEM_TYPE_FLOAT},
    {TELEM_DISTANCE_TO_HOME, TELEM_TYPE_FLOAT},
    {TELEM_HOLD_LAT, TELEM_TYPE_DOUBLE},
    {TELEM_HOLD_LON, TELEM_TYPE_DOUBLE},
    {TELEM_POI_LAT, TELEM_TYPE_DOUBLE},
    {TELEM_POI_LON, TELEM_TYPE_DOUBLE},
    {TELEM_GPS_SATELLITES_FLAGS, TELEM_TYPE_U8},
    {TELEM_GPS_SATELLITES_FIX, TELEM_TYPE_U8},
    {TELEM_GPS_NUMBER_OF_SATELLITES, TELEM_TYPE_U8},
    {TELEM_NAV_MODE, TELEM_TYPE_U8},
    {TELEM_MICROSPERSECOND, TELEM_TYPE_U32},
    {TELEM_MIXER_THROTTLE, TELEM_TYPE_FLOAT},
    {TELEM_MIXER_AVG_THROTTLE, TELEM_TYPE_FLOAT},
    {TELEM_MIXER_PITCH, TELEM_TYPE_FLOAT},
    {TELEM_MIXER_ROLL, TELEM_TYPE_FLOAT},
    {TELEM_MIXER_RUDDER, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_PWM1, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM2, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM3, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM4, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM5, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM6, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM7, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM8, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM9, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM10, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM11, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM12, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM13, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM14, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM15, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM16, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM17, TELEM_TYPE_U16},
    {TELEM_MOTORS_PWM18, TELEM_TYPE_U16},
    {TELEM_MOTORS_RPM1, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM2, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM3, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM4, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM5, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM6, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM7, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM8, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM9, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM10, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM11, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM12, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM13, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM14, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM15, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM16, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM17, TELEM_TYPE_S16},
    {TELEM_MOTORS_RPM18, TELEM_TYPE_S16},
    {TELEM_MOTORS_CURRENT1, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT2, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT3, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT4, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT5, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT6, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT7, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT8, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT9, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT10, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT11, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT12, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT13, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT14, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT15, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT16, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT17, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_CURRENT18, TELEM_TYPE_FLOAT},
    {TELEM_MOTORS_TEMP1, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP2, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP3, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP4, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP5, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP6, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP7, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP8, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP9, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP10, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP11, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP12, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP13, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP14, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP15, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP16, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP17, TELEM_TYPE_S16},
    {TELEM_MOTORS_TEMP18, TELEM_TYPE_S16},
    {TELEM_RC1, TELEM_TYPE_S16},
    {TELEM_RC2, TELEM_TYPE_S16},
    {TELEM_RC3, TELEM_TYPE_S16},
    {TELEM_RC4, TELEM_TYPE_S16},
    {TELEM_RC5, TELEM_TYPE_S16},
    {TELEM_RC6, TELEM_TYPE_S16},
    {TELEM_RC7, TELEM_TYPE_S16},
    {TELEM_RC8, TELEM_TYPE_S16},
    {TELEM_RC9, TELEM_TYPE_S16},
    {TELEM_RC10, TELEM_TYPE_S16},
    {TELEM_RC11, TELEM_TYPE_S16},
    {TELEM_RC12, TELEM_TYPE_S16},
    {TELEM_RC13, TELEM_TYPE_S16},
    {TELEM_RC14, TELEM_TYPE_S16},
    {TELEM_RC15, TELEM_TYPE_S16},
    {TELEM_RC16, TELEM_TYPE_S16},
    {TELEM_RC17, TELEM_TYPE_S16},
    {TELEM_RC18, TELEM_TYPE_S16},
    {TELEM_RAW_ADC0, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC1, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC2, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC3, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC4, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC5, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC6, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC7, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC8, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC9, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC10, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC11, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC12, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC13, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC14, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC15, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC16, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC17, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC18, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC19, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC20, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC21, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC22, TELEM_TYPE_FLOAT},
    {TELEM_RAW_ADC23, TELEM_TYPE_FLOAT},
    {TELEM_DGYRO_X, TELEM_TYPE_FLOAT},
    {TELEM_DGYRO_Y, TELEM_TYPE_FLOAT},
    {TELEM_DGYRO_Z, TELEM_TYPE_FLOAT},
    {TELEM_STDGYROX, TELEM_TYPE_FLOAT},
    {TELEM_STDGYROY, TELEM_TYPE_FLOAT},
    {TELEM_STDGYROZ, TELEM_TYPE_FLOAT},
    {TELEM_STDGYROSUM, TELEM_TYPE_FLOAT},
    {TELEM_GIMBAL_PWM_PITCH, TELEM_TYPE_U16},
    {TELEM_GIMBAL_PWM_ROLL, TELEM_TYPE_U16},
    {TELEM_GIMBAL_ANGLE_ROLL, TELEM_TYPE_FLOAT},
    {TELEM_GIMBAL_ANGLE_PITCH, TELEM_TYPE_FLOAT},
    {TELEM_GIMBAL_ANGLE_YAW, TELEM_TYPE_FLOAT},
    {TELEM_DEBUG1, TELEM_TYPE_FLOAT},
    {TELEM_DEBUG2, TELEM_TYPE_FLOAT},
    {TELEM_DEBUG3, TELEM_TYPE_FLOAT},
    {TELEM_DEBUG4, TELEM_TYPE_FLOAT},
    {TELEM_DEBUG5, TELEM_TYPE_FLOAT},
    {TELEM_DEBUG6, TELEM_TYPE_FLOAT},
    {TELEM_DEBUG7, TELEM_TYPE_FLOAT},
    {TELEM_DEBUG8, TELEM_TYPE_FLOAT},
    {TELEM_ALT_PSTART_MAP, TELEM_TYPE_FLOAT},
    {TELEM_PHOTO_FLAGS, TELEM_TYPE_U8},
    {TELEM_PHOTO_SHOT_CNT, TELEM_TYPE_U16},
    {TELEM_NAV_ACTIVE_WP, TELEM_TYPE_U16},
    {TELEM_NAV_SAT_TOTAL_GPS, TELEM_TYPE_U8},
    {TELEM_NAV_SAT_TOTAL_GLONASS, TELEM_TYPE_U8},
    {TELEM_NAV_SAT_USED_GPS, TELEM_TYPE_U8},
    {TELEM_NAV_SAT_USED_GLONASS, TELEM_TYPE_U8},
    {TELEM_NAV_HOLD_MAX_HSPEED, TELEM_TYPE_FLOAT},
    {TELEM_RTK_DIFF_AGE, TELEM_TYPE_FLOAT},
    {TELEM_FFT_DIV, TELEM_TYPE_U8},
    {TELEM_MARKER_LAT, TELEM_TYPE_DOUBLE},
    {TELEM_MARKER_LON, TELEM_TYPE_DOUBLE},
    {TELEM_MARKER_ALT, TELEM_TYPE_FLOAT},
    {TELEM_MARKER_HACC, TELEM_TYPE_FLOAT},
    {TELEM_MARKER_AGE, TELEM_TYPE_FLOAT},
    {TELEM_IN_DATA_AGE, TELEM_TYPE_FLOAT},
    {TELEM_RTKA_PRJ_LAT, TELEM_TYPE_FLOAT},
    {TELEM_RTKA_PRJ_LON, TELEM_TYPE_FLOAT},
    {TELEM_RTKA_PRJ_ALT, TELEM_TYPE_FLOAT},
    {TELEM_RTKA_DIST, TELEM_TYPE_FLOAT},
    {TELEM_RTKA_HEADING, TELEM_TYPE_FLOAT},
    {TELEM_RTKA_ANGLE, TELEM_TYPE_FLOAT},
    {TELEM_RTKA_MODE, TELEM_TYPE_U8},
    {TELEM_MUX_TEMPERATURE1, TELEM_TYPE_FLOAT},
    {TELEM_MUX_TEMPERATURE2, TELEM_TYPE_FLOAT},
    {TELEM_MUX_TEMPERATURE3, TELEM_TYPE_FLOAT},
    {TELEM_MUX_TEMPERATURE4, TELEM_TYPE_FLOAT},
    {TELEM_MUX_TEMPERATURE5, TELEM_TYPE_FLOAT},
    {TELEM_MUX_TEMPERATURE6, TELEM_TYPE_FLOAT},
    {TELEM_MUX_TEMPERATURE7, TELEM_TYPE_FLOAT},
    {TELEM_MUX_TEMPERATURE8, TELEM_TYPE_FLOAT},
    {TELEM_ADC_TEMPERATURE1, TELEM_TYPE_FLOAT},
    {TELEM_ADC_TEMPERATURE2, TELEM_TYPE_FLOAT},
    {TELEM_CUR_SENS1_A, TELEM_TYPE_FLOAT},
    {TELEM_CUR_SENS2_A, TELEM_TYPE_FLOAT},
    {TELEM_CUR_SENS3_A, TELEM_TYPE_FLOAT},
    {TELEM_CUR_SENS4_A, TELEM_TYPE_FLOAT},
    {TELEM_ENERGY_EFFICIENT_ALT, TELEM_TYPE_FLOAT},
    {TELEM_AMP_EFFICIENT_ALT, TELEM_TYPE_FLOAT},
    {TELEM_VOLT_SENS1, TELEM_TYPE_FLOAT},
    {TELEM_VOLT_SENS2, TELEM_TYPE_FLOAT},
    {TELEM_VOLT_SENS3, TELEM_TYPE_FLOAT},
    {TELEM_VOLT_SENS4, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_ENCODER1, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_ENCODER2, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_ENCODER3, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_EULER1, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_EULER2, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_EULER3, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_ZOOM, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_ZOOM_STATE, TELEM_TYPE_S16},
    {TELEM_CAMERA_OPTICAL_AXIS_LAT, TELEM_TYPE_DOUBLE},
    {TELEM_CAMERA_OPTICAL_AXIS_LON, TELEM_TYPE_DOUBLE},
    {TELEM_CAMERA_OPTICAL_AXIS_DIST, TELEM_TYPE_FLOAT},
    {TELEM_LIDAR_RANGE, TELEM_TYPE_FLOAT},
    {TELEM_DOSIMETER_DATA1, TELEM_TYPE_FLOAT},
    {TELEM_DOSIMETER_DATA2, TELEM_TYPE_FLOAT},
    {TELEM_DOSIMETER_DATA_TYPE, TELEM_TYPE_U8},
    {TELEM_DOSIMETER_DATA_STATUS, TELEM_TYPE_U8},
    {TELEM_DOSIMETER_VIN, TELEM_TYPE_FLOAT},
    {TELEM_DOSIMETER_DATA3, TELEM_TYPE_FLOAT},
    {TELEM_DOSIMETER_DATA_SEC, TELEM_TYPE_U32},
    {TELEM_NAV_AXIS_DIST, TELEM_TYPE_FLOAT},
    {TELEM_LIDAR_VELOCITY, TELEM_TYPE_FLOAT},
    {TELEM_GPS_VEL_N, TELEM_TYPE_FLOAT},
    {TELEM_GPS_VEL_E, TELEM_TYPE_FLOAT},
    {TELEM_LPS_LAT, TELEM_TYPE_DOUBLE},
    {TELEM_LPS_LON, TELEM_TYPE_DOUBLE},
    {TELEM_LPS_X, TELEM_TYPE_FLOAT},
    {TELEM_LPS_Y, TELEM_TYPE_FLOAT},
    {TELEM_LPS_Z, TELEM_TYPE_FLOAT},
    {TELEM_LPS_VELX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_VELY, TELEM_TYPE_FLOAT},
    {TELEM_LPS_VELZ, TELEM_TYPE_FLOAT},
    {TELEM_LPS_TAGSTATE, TELEM_TYPE_S8},
    {TELEM_LPS_GOODINDATAFREQ, TELEM_TYPE_U16},
    {TELEM_LPS_ANCHOR1_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR1_POWERRX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR1_VIN, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR2_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR2_POWERRX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR2_VIN, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR3_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR3_POWERRX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR3_VIN, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR4_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR4_POWERRX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR4_VIN, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR5_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR5_POWERRX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR5_VIN, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR6_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR6_POWERRX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR6_VIN, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR7_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR7_POWERRX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR7_VIN, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR8_DISTANCE, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR8_POWERRX, TELEM_TYPE_FLOAT},
    {TELEM_LPS_ANCHOR8_VIN, TELEM_TYPE_FLOAT},
    {TELEM_CAMERA_TEMPERATURE1, TELEM_TYPE_S8},
    {TELEM_CAMERA_TEMPERATURE2, TELEM_TYPE_S8},
    {TELEM_CAMERA_TEMPERATURE3, TELEM_TYPE_S8},
    {TELEM_CAMERA_TEMPERATURE4, TELEM_TYPE_S8},
    {TELEM_MOTORS_STATE1, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE2, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE3, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE4, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE5, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE6, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE7, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE8, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE9, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE10, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE11, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE12, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE13, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE14, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE15, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE16, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE17, TELEM_TYPE_U16},
    {TELEM_MOTORS_STATE18, TELEM_TYPE_U16},
    {TELEM_RADIO_FPS, TELEM_TYPE_U16},
    {TELEM_RADIO_SYNC_ERRORS, TELEM_TYPE_U32},
    {TELEM_RADIO_SYNC_ERRORS, TELEM_TYPE_U32},
    {TELEM_OPTIC_FLOW_DELTA_X, TELEM_TYPE_S32},
    {TELEM_OPTIC_FLOW_DELTA_Y, TELEM_TYPE_S32},
    {TELEM_LIDAR_LIDAR_QUALITY, TELEM_TYPE_U32},
    {TELEM_TOF_RANGE, TELEM_TYPE_FLOAT},
    {TELEM_ACC_MAG, TELEM_TYPE_FLOAT},
    {TELEM_GYRO_TOTAL_SPEED, TELEM_TYPE_FLOAT},
    {TELEM_SIDESLIP_DISTURB, TELEM_TYPE_FLOAT},
    {TELEM_ACC_SIDESLIP, TELEM_TYPE_FLOAT},
    {TELEM_ANGLE_TARGET_PITCH, TELEM_TYPE_FLOAT},
    {TELEM_ANGLE_TARGET_ROLL, TELEM_TYPE_FLOAT},
    {TELEM_PID_PITCH_ANGLE_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_PITCH_ANGLE_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_PITCH_ANGLE_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_PITCH_RATE_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_PITCH_RATE_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_PITCH_RATE_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_ROLL_ANGLE_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_ROLL_ANGLE_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_ROLL_ANGLE_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_ROLL_RATE_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_ROLL_RATE_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_ROLL_RATE_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_YAW_ANGLE_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_YAW_ANGLE_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_YAW_ANGLE_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_YAW_RATE_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_YAW_RATE_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_YAW_RATE_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_RUDD_ANGLE_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_RUDD_ANGLE_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_RUDD_ANGLE_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_SIDE_SLIP_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_SIDE_SLIP_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_SIDE_SLIP_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_ALT_DIST_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_ALT_DIST_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_ALT_DIST_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_ALT_VEL_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_ALT_VEL_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_ALT_VEL_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_DIST_N_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_DIST_N_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_DIST_N_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_DIST_E_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_DIST_E_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_DIST_E_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_SPEED_N_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_SPEED_N_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_SPEED_N_D, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_SPEED_E_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_SPEED_E_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_NAV_SPEED_E_D, TELEM_TYPE_FLOAT},
    {TELEM_JOY_SW_1, TELEM_TYPE_S16},
    {TELEM_JOY_SW_2, TELEM_TYPE_S16},
    {TELEM_JOY_SW_3, TELEM_TYPE_S16},
    {TELEM_JOY_SW_4, TELEM_TYPE_S16},
    {TELEM_PID_TO_AXIS_P, TELEM_TYPE_FLOAT},
    {TELEM_PID_TO_AXIS_I, TELEM_TYPE_FLOAT},
    {TELEM_PID_TO_AXIS_D, TELEM_TYPE_FLOAT},
    {TELEM_NAV_AXIS_ADD_ANG, TELEM_TYPE_FLOAT},
    {TELEM_NRL_2G, TELEM_TYPE_U8},
    {TELEM_NRL_3G, TELEM_TYPE_U8},
    {TELEM_NRL_VIN, TELEM_TYPE_U8},
    {TELEM_NRL_POWER, TELEM_TYPE_U8},
};
//-------------------------------------------

/*выполнить миссию номер N
Выполнить команду, закреплённую за многофункциональной кнопкой.*/
void COPA::CopaExecSwitch_N(uint8_t count) {
  /******************
    1-10 - выполнить команды, соответствующие
    числу нажатий на кнопку от 1 до 10.
    255 - эквивалентно длинному нажатию (LongClick)
   *****************/
  CopaPacketMake(CMD_EXEC_SWITCH_N, &count, 1);
  CopaPacketSend();
}
/*	Угол, на который следует довернуть
 *  аппарат относительно текущего курса
 *   с заданной угловой скоростью.*/
void COPA::CopaSetHeading(float yaw, float vel) {

  mov_t_ mov_;
  mov_.val_float1 = e_float(yaw);
  mov_.val_float2 = e_float(vel);

  CopaPacketMake(CMD_NAV_SET_ABS_HEADING, &mov_, 8);
  CopaPacketSend();
}
/*Установить вектор скорости движения
 * аппарата относительно системы координат
 * аппарата dVx, dVy, dVz на заданное
 * время =t (в мс) от момента прихода команды
 * (время ограничено TMAX=10сек).*/
void COPA::CopaSetAngles(float dx, float dy) {
  ManTargetAngeles_t mov_;
  mov_.Pitch = e_float(dx);
  mov_.Roll = e_float(dy);
  CopaPacketMake(CMD_SET_MAN_TARGET_ANGLES, &mov_, 8);
  CopaPacketSend();
}
void COPA::CopaSetNavMode(uint8_t mode) {

  CopaPacketMake(CMD_SET_NAV_MODE, &mode, 1);
  CopaPacketSend();
}
//-------------------------------------------
/**********************************Проверка коптера - чтение ID
 * коптера.**********************************/
void COPA::Copa_Get_info() {
  FC_ID = 0x0FFF;
  CopaPacketMake(CMD_GET_DEVICE_INFO, 0, 0);
  CopaPacketSend();
}
void SetGnss(uint8_t i) {
  CopaPacketMake(CMD_GNSS_USE, &i, 1);
  CopaPacketSend();
}

/**********************************	Запуск
 * двигателей.**********************************/
void COPA::Copa_Motors_On() {
  CopaPacketMake(CMD_NAV_MOTORS_ON, 0, 0);
  CopaPacketSend();
}
/*	Стоп Двигателей (отключает двигатели). Срабатывает по 3 последовательным
 * командам, между ними не должно быть других команд!*/
void COPA::Copa_Motors_Off() {
  CopaPacketMake(CMD_NAV_MOTORS_OFF, 0, 0);
  CopaPacketSend();
}
//-------------------------------------------
/**********************************	Взлёт
 * **********************************/
void COPA::Copa_Take_Off(float doAltitude) {
  //Высота относительно точки взлёта, на которую должен подняться коптер.
  doAltitude = e_float(doAltitude);
  CopaPacketMake(CMD_NAV_TAKE_OFF, &doAltitude, 0);
  CopaPacketSend();
}
/**********************************	Посадка.
 * **********************************/
void COPA::Copa_To_Land(void *body) {
  // float groundAlt_, float speedDecAltitude_, float altVel_, float endVel_
  // groundAlt	        Высота точки ПОСАДКИ относительно точки взлета
  // (AltLandPoint=AltMap-AltMap_Pstart). AltLandPoint=0 если это точка
  // Взлета. speedDecAltitude	Размер ЗОНЫ ПОСАДКИ. Это не точка посадки, а
  // высота компенсирующая ошибки определения истинной высоты. В пределах этой
  // зоны коптер существенно снижает скорость и готов к посадке. altVel
  // Скорость снижения (скорость до зоны посадки).
  // endVel			Скорость при посадке (скорость в зоне посадки).
  CopaPacketMake(CMD_NAV_TO_LAND, body, 16);
  CopaPacketSend();
}
//-------------------------------------------
/*Запрос параметров установленного пресета #N (номер пресета: от 1 до 5 в
 * текущей версии ПО САУ).*/
void COPA::Copa_telem_read_config(uint8_t count) {}
//-------------------------------------------
/*Остановиться и Зафиксировать текущую позицию.
 * Если выполнялось задание или какая-то команда
 * ее выполнение приостанавливается, чтобы их
 * продолжить требуется выполнить команду продолжить
 * выполнение команды или приход новой команды управления.*/
void COPA::Copa_StopMotion() {
  CopaPacketMake(CMD_NAV_STOP_MOTION, 0, 0);
  CopaPacketSend();
}
//-------------------------------------------
/*Продолжить выполнение активной задачи.*/
void COPA::Copa_ContinueMotion() {
  CopaPacketMake(CMD_NAV_CONTINUE_MOTION, 0, 0);
  CopaPacketSend();
}
//-------------------------------------------
/**********************************Лететь в точку
 * "Дом".**********************************/
void COPA::Copa_GoTo_Home() {
  CopaPacketMake(CMD_NAV_GOTO_HOME, 0, 0);
  CopaPacketSend();
}
//-------------------------------------------
/**********************************открыть сервомашинки
 * крышки.**********************************/
void COPA::CoveringOpen() {
  uint16_t serv = e_uint16(1);
  CopaPacketMake(CMD_WFF_SERVO_CMD, &serv, sizeof(serv));
  CopaPacketSend();
}
//-------------------------------------------
/**********************************закрыть сервомашинки
 * крышки.**********************************/
void COPA::CoveringClose() {
  uint16_t serv = 0;
  CopaPacketMake(CMD_WFF_SERVO_CMD, &serv, sizeof(serv));
  CopaPacketSend();
}
//-------------------------------------------
/**********************************Оборудование
 * включить**********************************/
void COPA::EquipmentOn() {             //включить радиомаяк.
  if ((CopterState == COPTSTATE_NC) || //если нет связи
      (CopterState == COPTSTATE_BUSYINIT) || //если занят, инициализация
      (CopterState ==
       COPTSTATE_BUSYTARGET)) { //если  занят, загрузка полетного задания
    return; //то игнорирую
  }
  uint8_t phot = 253;
  CopaPacketMake(CMD_PHOTO_CMD, &phot, sizeof(phot)); //сформировать ответ
  CopaPacketSend(); //отправить ответ
}
/*********************************Оборудование
 * отключить**********************************/
void COPA::EquipmentOff() {            //выключить радиомаяк.
  if ((CopterState == COPTSTATE_NC) || //если нет связи
      (CopterState == COPTSTATE_BUSYINIT) || //если занят, инициализация
      (CopterState ==
       COPTSTATE_BUSYTARGET)) { //если  занят, загрузка полетного задания
    return; //то игнорирую
  }
  uint8_t phot = 254;
  CopaPacketMake(CMD_PHOTO_CMD, &phot, sizeof(phot)); //сформировать ответ
  CopaPacketSend(); //отправить ответ
}
//-------------------------------------------
/********************************** взлёт.**********************************/
void COPA::TakeOffCmd() {
  // TakeoffFlag = 1;
}
//-------------------------------------------
/**********************************собираю пакет для
 * отправки**********************************/
void COPA::CopaPacketMake(uint16_t cmd, void *body, uint16_t bodylen) {
  WaitCmdAck = cmd;
  sCoptHdr *Hdr = (sCoptHdr *)OutBuff;
  Hdr->magic0 = 0x41;
  Hdr->magic1 = 0x43;
  Hdr->magic2 = 0x35;
  Hdr->packetLen =
      e_uint16(bodylen + 16); //размер полезной нагрузки плюс заголовок кроме
                              // 3 байт идентификатора
  Hdr->packetNumber = e_uint16(sPN++);
  Hdr->senderID = e_uint16(GCS_ID);     // id наземки
  Hdr->recipientID = e_uint16(FC_ID);   // id коптера
  Hdr->packetTime = e_uint32(1);        //мсек актуальные
  Hdr->commandNumber = e_uint16(sCN++); //номер команды
  Hdr->commandType = e_uint16(cmd);     //тип команды 19
  if (bodylen) //если полезная нагрузка не пустая то присоединяю полезную
               //нагрузку
    memcpy(&(OutBuff[sizeof(sCoptHdr)]), body, bodylen);
  OutSize = sizeof(sCoptHdr) + bodylen +
            2; //длинна всего пакета плюс 2 байта контрольной суммы
  CopaPacketCRCRecount(); //контрольная сумма
}
//-------------------------------------------
/**********************************контрольная
 * сумма**********************************/
void COPA::CopaPacketCRCRecount() {
  uint8_t CRC_A = 0;
  uint8_t CRC_B = 0;
  for (int i = 3; i < (OutSize - 2); i++) {
    CRC_A += OutBuff[i];
    CRC_B += CRC_A;
  }
  OutBuff[OutSize - 2] = CRC_A;
  OutBuff[OutSize - 1] = CRC_B;
}
//-------------------------------------------
/**********************************отправить пакет в
 * UART**********************************/
void COPA::CopaPacketSend() { link_->sendData(OutBuff, OutSize); }
/**********************************повторная отправка-увеличит порядковый


/**********************************Размерность и тип
 * данных**********************************/

/**********************************обнулить все биты в пресете под номером
 * presetN**********************************/
void COPA::preset_bits_clear(uint8_t presetN) {
  for (uint8_t i = 0; i < TELEMETRY_PRESET_BITMASK_LENGTH; i++)
    preset_param_key[presetN][i] = '\0';
}
/**********************************выставить бит параметра в
 * пресете**********************************/
void COPA::preset_bit_set(uint16_t comand, uint8_t preset_) {
  uint16_t count; //элементов в массиве
  uint16_t byteN; //байт где хранится ключ к параметру
  uint8_t pos_; //позиция Бита для ключа параметра
  uint8_t bit_; //байт для включения в нём бита
  // preset_param_key[preset_][0] = preset_param_key[preset_][0] |  (1 <<
  // comand);
  count = sizeof(telemetryFields) /
          sizeof(telemetryFields_t); //количество параметров в массиве шаблона

  for (uint16_t i = 0; i < count; i++) {
    if (telemetryFields[i].TELEM_NUM == comand) {
      byteN = i / 8; //байт где хранится ключ к параметру
      pos_ = i - (byteN * 8); //позиция Бита для ключа параметра
      //включаю бит
      bit_ = preset_param_key[preset_][byteN];
      preset_param_key[preset_][byteN] = bit_ | (1 << pos_);
      break;
    }
  }
}
/**********************************Узнать номер параметра в
 * пресете**********************************/
uint8_t COPA::Telem_Get_Param_preset(uint16_t comand, //какую команду ищу
                                     uint8_t preset_ //какой пресет проверяю
) {
  uint16_t count;  //элементов в массиве
  uint8_t num = 0; //позиция параметра в персете
  uint16_t byteN; //байт где хранится ключ к параметру
  uint8_t pos_; //позиция Бита для ключа параметра
  uint8_t bit_; //байт для сравнения

  count = sizeof(telemetryFields) /
          sizeof(telemetryFields_t); //количество параметров в массиве шаблона
  for (uint16_t i = 0; i < count; i++) {
    byteN = i / 8; //байт где хранится ключ к параметру
    pos_ = i - (byteN * 8); //позиция Бита для ключа параметра
    bit_ = preset_param_key[preset_][byteN];
    if ((bit_ >> pos_) & 1) {
      num++; //порядковый номер включенного параметра
      if (telemetryFields[i].TELEM_NUM == comand) {
        return num;
      }
    }
  }
  return 0;
}

void COPA::Miss_Clear() { //очистить миссию в коптере
  CopaPacketMake(CMD_NAV_CLEAR_WP, 0, 0);
  Mission_stat = (uint8_t)MISS_UPLOAD;
  CopaPacketSend();
}

void COPA::Mission_Up(sMissionPoint *Point) {
  // //
  TPM_t *TPM = (TPM_t *)OutBuff;
  TPM->PointNum = e_uint16(Mission_Nup);
  TPM->PointsCount = e_uint16(1);
  memcpy(&TPM->pointData, Point, sizeof(sMissionPoint));
  uint8_t Body[CMD_CONTROL_MAX_PACKET_SIZE];
  CopaPacketSend();
}

void COPA::Mission_Dn() {
  if (Mission_stat == MISS_DOWNLOAD_P) {
    TZP_t TZP;
    TZP.pointStartN =
        e_uint16(Mission_Ndn); //	Номер точки, с которой начать считывание
    TZP.pointStopN = e_uint16(
        Mission_Ndn); //	Номер точки, которой закончить считывание
    CopaPacketMake(CMD_NAV_LOAD_POINT, &TZP, sizeof(TZP));
    CopaPacketSend();
  }
}
/************запросить количество загруженных точек
 * миссии********************/
void COPA::MissGetPointCount() {
  CopaPacketMake(CMD_NAV_GET_WP_COUNT, 0,
                 0); //запросить количество точек в загруженной миссии в коптере
  CopaPacketSend();
}
/**************  сравнивает загруженную точку  ******************/
uint8_t COPA::Mission_compare_p(Point_info_t *infoP) {
  // uint8_t rr = 1;
  // if (Target_Mission[infoP->pointN - 1].flags != infoP->MP.flags)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].loiterTime != infoP->MP.loiterTime)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].maxHorizSpeed !=
  //     infoP->MP.maxHorizSpeed)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].maxVertSpeed !=
  // infoP->MP.maxVertSpeed)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].panoDeltaAngle !=
  //     infoP->MP.panoDeltaAngle)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].panoSectorsCount !=
  //     infoP->MP.panoSectorsCount)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].photo != infoP->MP.photo)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].poiAltitude != infoP->MP.poiAltitude)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].poiHeading != infoP->MP.poiHeading)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].poiLat != infoP->MP.poiLat)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].poiLon != infoP->MP.poiLon)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].poiPitch != infoP->MP.poiPitch)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].poiRoll != infoP->MP.poiRoll)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].targetAlt != infoP->MP.targetAlt)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].targetLat != infoP->MP.targetLat)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].targetLon != infoP->MP.targetLon)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].targetRadius !=
  // infoP->MP.targetRadius)
  //   return 0;
  // if (Target_Mission[infoP->pointN - 1].type != infoP->MP.type)
  //   return 0;
  return 1;
}

//включить бит
void COPA::CopaActual_flag_on(uint16_t fl) { Actual_Status_Flags |= fl; }
//отключить бит
void COPA::CopaActual_flag_off(uint16_t fl) {
  if ((Actual_Status_Flags & fl) > 0)
    Actual_Status_Flags &= ~fl;
}
//прочитать
uint8_t COPA::CopaActual_flag_get(uint16_t fl) {
  return Actual_Status_Flags & fl;
}
