/*
 * command_handler.h
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */
#ifndef COPA_H_
#define COPA_H_
#include "abstract_link.h"
#include "copa_types.h"
#include <coparos/Command.h>
#include <coparos/MissionPoint.h>
#include <cstdint>
#include <geometry_msgs/Vector3.h>
#include <list>
#include <memory>

#include <math.h>
#include <ros/ros.h>

/*############# defines ###########*/
#define ACTUAL_STAT_COPA 0x0001 //АКТИВНОСТЬ КОПТЕРА -
#define ACTUAL_STAT_TELEM 0x0002 //телеметрия с коптера пришла
#define ACTUAL_SPEED_UART 0x0004 //определена скорость UART Коптера или нет
#define ACTUAL_SET_PRESET 0x0008 // 0= пресет не настроен 1= пресет настроен

class COPA {
  AbstractLink *link_;
  ros::Publisher ack_pub_;
  ros::Publisher status_pub_;
  ros::Publisher drone_info_pub_;
  ros::Publisher mission_point_request_pub_;
  ros::Publisher mission_point_responce_pub_;

  ros::Publisher byteArray_pub_;
  ros::Subscriber command_sub_;
  ros::Subscriber mission_point_sub_;
  ros::Subscriber angles_sub_;
  ros::NodeHandle *nh_;

  /*############# types ###########*/
  uint16_t rPN; //порядковый номер получаемого пакета
  uint16_t sPN; //порядковый номер отправляемого пакета
  uint16_t rCN; //порядковый номер получаемой команды
  uint16_t sCN; //порядковый номер отправляемой команды
  uint16_t sTC; //тип отправленой команды команды
  volatile uint16_t ParseInSize; //количество байт входящего пакета для парсинга
  uint8_t CopterState;           //статус коптера
  // uint8_t TakeoffFlag;                                  // взлёт.
  // uint32_t IntercomTick;
  uint8_t BusyStep; //статус обработки команды
  uint8_t BusyStep_out; //статус обработки команды в коптер
  volatile uint8_t MissionState; //статус обработки команды Миссии по кнопке

  uint8_t
      preset_param_key[3][TELEMETRY_PRESET_BITMASK_LENGTH]; //битовая маска
                                                            //параметров пресета

  telemetryParam_t_ presetParam[3]; //параметры трёх пресетов
  uint8_t ParamTelemStat; //статус чтения параметров телеметрии с коптера
  uint16_t telemCount; //Сколько всего параметров телеметрии в САУ. Это число
                       //может меняться от версии к версии ПО САУ.

  uint32_t BusyRouteTick;
  uint16_t PackLen;

  uint8_t BigEndian;

  uint32_t LastTransmittedTick;
  uint16_t WaitCmdAck; //тут хранится отправленая коптеру команда
  uint8_t NewTarget;
  uint8_t InCRC_A; //Данные контрольной суммы
  uint8_t InCRC_B;
  uint8_t copa_st;
  uint32_t devVers;      //	Версия устройства
  uint32_t ProtocolVers; //	Версия протокола
  uint8_t devName[32]; //	Имя устройства (текст 32 символа)
  uint16_t FC_ID; // ID - САУ (полётный контроллер, FlightController)
  uint16_t
      GCS_ID; // ID - НСУ получателя наземная станция управления, ПО оператора,
              // внешняя система управления, GroundControlStation
  uint8_t CMD_ACK_OK; //ключ подтверждения получения пакета 0-не получен или
                      //ожидает 1-получен

  //подтверждение команды

  /*****************приём UART коптер*****************/
  typedef struct {
    uint16_t devType; //	идентификатор устройства (4 старших бита = тип
                      //устройства, см. здесь)
    uint32_t devVers; //	Версия устройства
    uint32_t ProtocolVers; //	Версия протокола
    uint8_t devName[32]; //	Имя устройства (текст 32 символа)
  } __attribute__((packed)) DeviceInfo_t;
  DeviceInfo_t DeviceInfo;
  /*************************  ****************************/
  typedef struct {
    uint16_t packetNumber;
    uint16_t senderID;
    uint16_t recipientID;
    uint32_t packetTime;
    uint16_t commandNumber;
    uint16_t commandType;
    union {
      uint8_t data[256];
      float paramsFloat[64];
      double paramsDouble[32];
      int16_t paramsInt16[128];
      int32_t paramsInt32[64];
    } u;
    uint8_t chkA;
    uint8_t chkB;
  } __attribute__((packed)) cmdControlBufStruct_t;

  /*************************  ****************************/
  typedef struct {
    uint8_t ChkA_;
    uint8_t ChkB_;
  } crcAB;
  crcAB crcab;
  /*************************  ****************************/
  typedef struct {
    uint16_t uint16_;
  } __attribute__((packed)) u16_t_;
  /*************************  ****************************/
  typedef struct {
    uint32_t uint32_;
  } __attribute__((packed)) u32_t_;
  /*************************  ****************************/
  typedef struct {
    float val_float1;
    float val_float2;
    float val_float3;
    uint32_t val_u32;
  } __attribute__((packed)) mov_t_;

  /*************************  ****************************/
  /*
  typedef struct {
      uint8_t state;//статус при чтении пакета
      uint16_t inPacketSize;//количество байт в принимаемом пакете включая 2
  байта CRC без 3-х байт начала и 2-х байт размера(не более 500 байт) uint16_t
  pathBufCnt;//счётчик читаемого байта всего пакета uint8_t i;//счётчик
  читаемого байта полезной нагрузки uint8_t n;//для чтения количества байт в
  пакете - для inPacketSize uint8_t *pathBuf;//буфер для записи принимаемого
  пакета cmdControlBufStruct_t buf;//структура для принятого пакета void
  (*ccParser)(void);//проседура парсинга uint8_t checksumErrors;//счётчик ошибок
  при приёме uint8_t inChkA;//контрольная сумма uint8_t inChkB;//контрольная
  сумма } __attribute__((packed)) cmdControlDeviceStruct_t;
  cmdControlDeviceStruct_t cmdControlDeviceStruct;*/

  /****** Первый байт пакета (STX) — это символ начала пакета: ******/
  /* Mavlink
   * 0xFD для версии v2.0
   * 0xFE для версии v1.0
   * 0x55 для версии v0.9
   * Viktor
   * 0x41 Пакет Виктора
   */
  enum Device_STX {
    FIRST_BYTE_MAV_V0_9 = 0x55,
    FIRST_BYTE_MAV_V1_0 = 0xFE,
    FIRST_BYTE_MAV_V2_0 = 0xFD,
    FIRST_BYTE_VIKTOR = 0x41,
  };

  /*************************  ****************************/
  enum cctlParseStates {
    CCTL_STATE_SYNC1 = 0, //проверка 1-й байта
    CCTL_STATE_SYNC2,     //проверка 2-й байта
    CCTL_STATE_SYNC3,     //проверка 3-й байта
    CCTL_STATE_SIZE1, //чтение количества байт пакета
    CCTL_STATE_SIZE2, //
    CCTL_STATE_PAYLOAD,
    CCTL_STATE_CHK_SUM_A,
    CCTL_STATE_CHK_SUM_B,
  };

  /************************* Типы устройств ****************************/
  /*
   * Для адресации ко всем устройствам одновременно используется адрес
   * получателя 0xFFFF (broadcast) Для адресации ко всем устройствам только
   * одного класса используется адрес вида 0xNFFF, где N - Номер класса
   * Например, для отправки сообщения всем устройствам типа "коптер"
   * используется адрес получателя 0x0FFF
   */

  enum deviceClasses {
    DEVICE_ID_COPTER_FC = 0x0000, //САУ коптера (полётный контроллер). Возможны
                                  //адреса от 1 до 4094.
    DEVICE_ID_COPTER_ALL = 0x0FFF, //
    DEVICE_ID_GIMBAL = 0x1000, //Гиростабилизированная платформа камеры
    DEVICE_ID_GIMBAL_ALL = 0x1FFF,          //
    DEVICE_ID_VIDEO_PROCESSOR = 0x2000,     //Видеопроцессор
    DEVICE_ID_VIDEO_PROCESSOR_ALL = 0x2FFF, //
    DEVICE_ID_VIDEO_OPERATOR = 0x3000, //рабочее место оператора видеокамеры
    DEVICE_ID_VIDEO_OPERATOR_ALL = 0x3FFF, //
    DEVICE_ID_GROUND_STATION = 0x4000, //рабочее место оператора коптера
    DEVICE_ID_GROUND_STATION_ALL = 0x4FFF, //
    DEVICE_ID_GROUND_RADIO_CONTROLLER =
        0x5000, //наземный блок контроллера радиоканала телеметрии (контроль
                //питания модуля, RSSI, поворотная система наведения антенны)
    DEVICE_ID_GROUND_RADIO_CONTROLLER_ALL = 0x5FFF, //
    DEVICE_ID_ALL = 0xFFFF, //Для всех (такой идентификатор можно использовать
                            //только для получателя)
  };

  /************************* команды ****************************/
  /*############# union ###########*/
  union Cmd_u_ {
    uint8_t data_[256];
    float paramsFloat_[64];
    double paramsDouble_[32];
    int16_t paramsInt16_[128];
    int32_t paramsInt32_[64];
  } Cmd_u_;

private:
  void ConfigTelemetry(); //получены новые данные телеметрии с коптера. *
                          //Широта/долгота/высота/скорость/курс/к-во *
                          //спутников/напряжение АКБ.
  void PacketReceived(sCoptHdr *header, void *body); //парсинг входящего пакета
  void SetParamPreset(); //Заполнить битовую маску для пресета
  void Copa_Cmd_Ack(sCoptHdr *header); //команда CMD_AC подтверждения
  void GetTelemConfig(); //запросить параметры телеметрии.
  void copa_reset(); //перезагрузить коптер

  void CopaParseByte(uint8_t byte); //Побайтное чтение пакета с коптера

  void Copa_Pool(); //Тестовый пакет, по которому САУ делает выводы (например,
                    //активизировать режим "FailSafe", если за последнюю минуту
                    //не пришло ни одного пакета) о наличии и качестве канала
                    //управления от НСУ.	Частота пакетом выбирается
                    //исходя из возможностей канала связи.
  void
  Copa_PreStartCheck(uint32_t AltMap_Pstart_); //Предстартовая проверка
                                               //готовности системы к взлёту.

  void CopaSetHeading(float yaw,
                      float vel); //Угол, на который следует довернуть коптер
  void CopaSetAngles(float dx, float dy); //Установить вектор скорости движения
  void CopaSetNavMode(uint8_t mode);
  void Copa_Go_To(double lat, double lon, float alt, float hspeed,
                  float vspeed);
  void Copa_Get_info(); //Проверка коптера - чтение ID коптера.
  void Copa_Motors_On();                //Запуск двигателей.
  void Copa_Motors_Off();               //Стоп Двигателей
  void Copa_Take_Off(float doAltitude); //Взлёт
  void Copa_To_Land(void *body);        //Посадка
  void Copa_telem_read_config(
      uint8_t count); //Запрос параметров установленного пресета #N (номер
                      //пресета: от 1 до 5 в текущей версии ПО САУ).
  void Copa_StopMotion(); //Остановиться и Зафиксировать текущую позицию.
  void Copa_ContinueMotion(); //Продолжить выполнение активной задачи.
  void Copa_GoTo_Home(); //Лететь в точку "Дом"
  void CoveringOpen();   //открыть сервомашинки крышки.
  void SetGnss(uint8_t i);

  void CoveringClose(); //закрыть сервомашинки крышки.
  void CopaPacketMake(uint16_t cmd, void *body,
                      uint16_t bodylen); //Сборка пакета для отправки
  void CopaPacketSend();                 //отправить пакет в UART
  void CopaPacketCRCRecount(); //контрольная сумма

  void EquipmentOn();  //Оборудование включить
  void EquipmentOff(); //Оборудование отключить
  void TakeOffCmd();   //Взлёт.
  uint8_t Telem_Get_Param_preset(uint16_t comand, //Команда в пресете
                                 uint8_t preset_ //Номер пресета
  ); //Узнать номер параметра в пресете

  void CopaActual_flag_on(uint16_t fl);  //включить бит
  void CopaActual_flag_off(uint16_t fl); //отключить бит

  void preset_bits_clear(
      uint8_t presetN); //обнулить все биты в пресете под номером presetN
  void preset_bit_set(uint16_t comand,
                      uint8_t preset_); //выставить бит параметра в пресете
public:
  ~COPA();

  //пакет телеметрии
  void callback_command(const coparos::Command &msg);
  void callback_mission_point(const coparos::MissionPoint &msg);
  void callback_angles(const geometry_msgs::Vector3 &msg);
  uint8_t presetStatSet =
      1; //статус инициализации (настройки пресета) если меньше
         // 4 то ещё не настроен
         //статус отправляемого пакета
  uint8_t OutBuff[CMD_CONTROL_MAX_PACKET_SIZE]; //буфер исходящего пакета
  uint16_t OutSize; //количество байт исходящего пакета
  uint8_t InBuff[CMD_CONTROL_MAX_PACKET_SIZE];
  uint16_t InSize; //количество байт входящего пакета

  uint8_t Mission_stat = 0; //статус отправляемого пакета

  uint8_t Mission_Nup = 0; //Номер загружаемой точки миссии (грузится)
  // uint8_t Mission_Nup_  = 0;                              //Номер загруженной
  // точки миссии

  uint8_t Mission_Ndn = 0; //Номер выгружаемой точки миссии (выгружается)
  // uint8_t Mission_Ndn_ = 0;                               //Номер выгруженной
  // точки миссии
  uint16_t Mission_count = 0; //сколько точек в миссии в коптере
  uint16_t
      Actual_Status_Flags; //флаги состояния например определина ли скорость
                           // UART или получена телеметрия
  uint32_t
      Mission_timer; //таймер для повторения загрузки или выгрузки точек задания

  COPA(AbstractLink *link,
       ros::NodeHandle *nh); //задать начальные параметры при первой загрузке
  void parseFunc();
  float e_float(float t);        //перевернуть байты
  double e_double(double t);     //перевернуть байты
  uint16_t e_uint16(uint16_t t); //перевернуть байты
  uint32_t e_uint32(uint32_t t); //перевернуть байты

  void Preset_Set_Param(); //настроить пресет 1 для телеметрии
  void CopaParseBUF(std::shared_ptr<std::list<unsigned char>> list_ptr);
  void Copa_Ping(); //Тестовая посылка для проверки канала связи и общего
                    //рабочего состояния устройств.
  uint8_t CopaActual_flag_get(uint16_t fl); //прочитать
  void Miss_Clear(); //очистить миссию в коптере
  void Mission_Up(sMissionPoint *Point); // загрузка очередной точки в коптер
  void Mission_Dn(); // Выгрузка очередной точки из коптера
  void MissGetPointCount(); //запросить количество загруженных точек миссии
  uint8_t Mission_compare_p(Point_info_t *infoP); //сравнивает загруженную точку
  void CopaExecSwitch_N(uint8_t count); //выполнить миссию номер
};

#endif /* COPA_H_ */
