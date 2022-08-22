/*
 * copa_types.h
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */

#ifndef COPA_TYPES_H_
#define COPA_TYPES_H_

#include <cstdint>
#include <stdint.h>
/*############# ROS defines ###########*/

/*############# ROS defines ###########*/
/*############# defines ###########*/
//#define WGS84_X_RAD 6378137.0000
//#define WGS84_Y_RAD 6356752.3142
//#define DEG_TO_RAD 3.1415926/180.0
//#define RAD_TO_DEG 180.0/3.1415926
//#define PI 3.1415926
#define TELEMETRY_PRESET_BITMASK_LENGTH 80
/*############# defines ###########*/
#define CMD_CONTROL_PROTOCOL_VERSION 1
#define CMD_CONTROL_MAX_PACKET_SIZE 256
//#define TELEMETRY_PRESET_BITMASK_LENGTH 80 уже есть в pilottype.h

#define COPTSTATE_NC 0x00       // нет связи
#define COPTSTATE_BUSYINIT 0x01 // занят, инициализация
#define COPTSTATE_NOTARGET 0x02 // ожидание полетного задания
#define COPTSTATE_BUSYTARGET 0x03 // занят, загрузка полетного задания
#define COPTSTATE_CORRECTING                                                   \
  0x04 // Калибровка датчиков, сброс не рекомендуется.
#define COPTSTATE_OK 0x05 // Ok, ожидание команды готовности к сбросу

#define COPTSTATE_READY 0x06 // Готов к сбросу

#define COPTSTATE_INCORRECT_SETTINGS                                           \
  0x10 // Неправильные настройки квадрокоптера (или версия ПО).
#define COPTSTATE_AP_ERROR 0x20 // Ошибка автопилота

//#define ACTUAL_STAT_COPA           0x01 //АКТИВНОСТЬ КОПТЕРА
//#define ACTUAL_STAT_TELEM          0x02 //телеметрия с коптера пришла
//#define ACTUAL_SPEED_UART          0x04 //определена скорость UART Коптера или
//нет
#define TEXT_BUFF_LENGHT 254
#define MISS_DEF 0 //никаких операций не происходит
#define MISS_NEW_MISS 1 //получена координата создаю новую миссию
#define MISS_UPLOAD 2   //загружаются точки в коптер
#define MISS_DOWNLOAD 3 //выгружаются точки из коптера
#define MISS_DOWNLOAD_P 4 //выгружаются точки из коптера для проверки
#define MISS_GET_POINT_COUNT 5 //Запрашивается количество точек из коптера
#define MISS_GET_POINT_COUNT_P 6 //Запрашивается количество точек из коптера
#define MISS_POINT_CLEAR 7 //стереть задание в коптере
#define MISS_START 8 //миссия загружена и проверена и коптер готов к взлёту
#define MISS_ARM 9 //миссия загружена и проверена и коптер готов к взлёту
#define MISS_SET_HOME 10 //Сохранить точку дом
//#define  OTI_NOGPS_VIRT_POLE   11          // лететь на цель в виртуальном
//поле

#pragma pack(push, 1)
typedef struct { //заголовок пакета
  uint8_t magic0;
  uint8_t magic1;
  uint8_t magic2;
  uint16_t packetLen;     //длина пакета с этого места
  uint16_t packetNumber;  //номер пакета
  uint16_t senderID;      // id отправителя
  uint16_t recipientID;   // id получателя
  uint32_t packetTime;    //время пакета
  uint16_t commandNumber; //номер отправления
  uint16_t commandType;   //тип команды
} sCoptHdr;
/*параметры к CMD_ACK*/
typedef struct {
  uint16_t packetNumber; //номер пакета (по порядку отправления)
  uint16_t commandNumber; //номер команды (последовательный, изменяется с каждой
                          //новой командой, если команда дублируется от потерь
                          //ее номер не изменяется)
  uint16_t commandType;   //тип команды
} sAckBody;
typedef struct {
  uint16_t packetNumber; //номер пакета (по порядку отправления)
  uint16_t commandNumber; //номер команды (последовательный, изменяется с каждой
                          //новой командой, если команда дублируется от потерь
                          //ее номер не изменяется)
  uint16_t commandType;
  char text[TEXT_BUFF_LENGHT - 26]; //тип команды
} sNAckBody;
/* точка дом */
typedef struct {
  double homeLat; //номер пакета (по порядку отправления)
  double homeLon; //номер команды (последовательный, изменяется с каждой новой
                  //командой, если команда дублируется от потерь ее номер не
                  //изменяется)
  float homeAlt;  //тип команды
} Home_Point_t;
/*параметр пресета*/
typedef struct {
  uint8_t presetNumber; // presetN	Номер пресета, параметры которого
                        // следует изменить.
  uint16_t rate; //Значение, определяющее как часто отправлять телеметрию этого
                 //пресета.
  /*Примеры rate:
200 - раз в секунду
100 - 2 раза в секунду
50 - 4 раза в секунду
1000 - раз в 5 секунд и т.п.
   */
  uint16_t offset; //Значение, определяющее с каким сдвигом отправлять
                   //телеметрию этого пресета.
  /*Смысл параметра: равномерно распределить в времени разные пресеты.
Пример:
Пусть определено 3 пресета с rate = 200.
В этом случае, чтобы равномерно во времени распределить отправку пакетов следует
указать offset так: Первый с offset=0, второй с offset=66 третий с offset=133
   */
} sTelemPresetHdr;
/*параметр заголовка телеметрии*/
typedef struct {
  uint16_t paramNum;
  uint8_t paramType;
} sTelemParamHdr;
/*пакет телеметрии*/
//структура телеметрии Preset1
typedef struct {
  uint8_t preset_number;  //номер пресета 1
  float VBAT;             //напряжение 5
  uint16_t STATE;         //статус 7
  uint32_t FLAGS;         //флаг 9
  float ABSOLUTE_HEADING; //
  float ALTITUDE;         //высота 13
  float ALTITUDE_HOLD_SPEED;
  float GPS_ALT;  //высота GPS 17
  double GPS_LAT; // GPS КООРДИНАТА 25
  double GPS_LON; // GPS КООРДИНАТА 33
  float GPS_HACC; //точность 37
  float GPS_HDOP;
  uint8_t GPS_SATELLITES_FIX;
  uint8_t GPS_NUMBER_OF_SATELLITES; //количество спутников 42
  uint8_t DRONE_MODE;
  int16_t rc6_channel;
  int16_t rc11_channel;
  uint16_t CURRENT_WP;

} LowSpeedTelemetry;

/*для тчоки миссии*/
typedef struct {
  double
      targetLat; // 4 широта точки, градусы.
                 // http://man.copterworks.ru/index.php/%D0%A4%D0%BE%D1%80%D0%BC%D0%B0%D1%82_%D0%B7%D0%B0%D0%BF%D0%B8%D1%81%D0%B8_%D0%BE_%D1%82%D0%BE%D1%87%D0%BA%D0%B5#C-code
  double targetLon; //долгота точки, градусы.
  float targetAlt; //высота точки над уровнем моря. Если требуется посадка
                   //(landingFlag), то подразумевается что это – высота
                   //поверхности, на которую он должен сесть (высота рельефа в
                   //данной точке, высота поверхности крыши и т.п). Если посадка
                   //не задана – высота, на которой он должен прийти на точку и
                   //зависнуть.
  float targetRadius; //Радиус цилиндра вокруг целевой координаты, попадание
                      //аппарата в который считается фактом достижения целевой
                      //точки
  uint32_t loiterTime; //Время в миллисекундах, сколько нужно подождать аппарату
                       //после фиксации факта достижения целевой точки
                       //(попадания в целевой цилиндр)
  float maxHorizSpeed; //скорость подлета к точке, м/с, в диапазоне от 1 до 12
                       //м/с.
  float maxVertSpeed; //Максимальная вертикальная скорость, которой
                      //придерживаться при полёте к целевой точке
  /*Используется для вычисления курса аппарата, с целью наведения оптической оси
   * камеры на цель*/
  double poiLat; //Координата в градусах по долготе точки наблюдения (Point Of
                 // Interest)
  /*Используется для вычисления курса аппарата, с целью наведения оптической оси
   * камеры на цель*/
  double poiLon; //Координата в градусах по широте точки наблюдения (Point Of
                 // Interest)
  float poiHeading; //курс, с которым коптер должен удерживать при подходе к
                    //точке (и в самой точке).(0 - на север, 90 - на восток, 180
                    //- на юг, 270 - на запад)
  /*Координата в метрах относительно точки взлёта по высоте точки
     наблюдения(Point Of Interest)
       Используется для понимания наклона камеры, с целью наведения оптической
     оси камеры на цель*/
  float poiAltitude; //высота подлета к точке (также над уровнем моря. Не может
                     //быть ниже alt_msl).
  uint32_t flags; //флаг посадки. 1 – посадка в точке, 0 – без посадки.
  /*Управление затвором фотоаппарата.
   * 0 - остановить съемку.
   * 1..127 - число кадров.
   * 253 - открыть фотоаппарат
   * 254 - закрыть фотоаппарат
   * 255 - начать снимать постоянно*/
  uint8_t photo; //Управление затвором фотоаппарата.
  uint8_t panoSectorsCount; //Число секторов для панорамной съемки.
  float panoDeltaAngle; //Приращение угла при панорамной съемке (в градусах).
  float poiPitch; //Угол наклона (начальный) камеры по Pitch.
  float poiRoll; //Угол наклона (начальный) камеры по Roll
  uint8_t type; //Тип точки. enum missPointTypes
} sMissionPoint;

/* точка миссии для загрузки или выгрузки */
typedef struct {
  uint16_t PointsCount;
  uint16_t PointNum;
  sMissionPoint pointData;
} TPM_t;

/*запросить точку миссии*/
typedef struct {
  uint16_t pointStartN; //	Номер точки, с которой начать считывание (=0 с
                        //начала, = другому значению с указанной точки, = 0xFFFF
                        //только последнюю точку)
  uint16_t pointStopN; //	Номер точки, которой закончить считывание (Если
                       // 0xFFFF то до конца, если другое значение то до точки с
                       //этим номером)
} TZP_t;

/*Информация о точке (в блоке точек) полётного задания. */
typedef struct {
  uint16_t totalPoints; //	Сколько точек всего высылается в блоке.
  uint16_t pointID; //	Номер точки в этом блоке.
  uint16_t pointN; //	Номер точки задания (нумерация в большом общем
                   //массиве/таблице)
  sMissionPoint MP;
} Point_info_t;

/*высота и флаг*/
typedef struct {
  float altitude;
  uint8_t flag;
} sIntercom;

/*************************** Дла отработки макроса полётного задания полёт
 * импульсным методом ***********/
typedef struct {
  uint8_t Line_n;   //счётчик выполнения плеча
  uint8_t Line_St;  //статус выполнения плеча
                    // 0=начало выполнения
                    // 1=выполняется угол
                    // 2=выполняется одно плечё
  uint32_t Timer_U; //Таймер выполнения угла по которому меняю статус Line_St
  uint32_t Timer_i; //Таймер интервала. импульса в плече
  uint16_t V_nn; //счётчик векторов в плече по наполнению счётчика Line_St = 3
} MacrosR_t;

/****** Структура для команды MD_SET_MAN_TARGET_ANGLES*********/
typedef struct { //для управление углами Pitch Roll в полёте
  float Pitch; //Тангаж
  float Roll;  //Крен
} ManTargetAngeles_t;

/****************************************************************************************************/
#define NAVI_MISS_FLAG_NONE 0 //Сброс всех флагов (или их отсутствие)
#define NAVI_MISS_FLAG_RELATIVE_ALT                                            \
  (1 << 0) // 0 - указанная высота исчисляется относительно точки взлёта 1 -
           // указанная цифра является дельтой к предыдущей целевой высоте
#define NAVI_MISS_FLAG_PANO_PITCH                                              \
  (1 << 1) // 1 - панорамная съемка будет происходить с приращением угла по оси
           // Pitch Используется с точками типа Панорама
#define NAVI_MISS_FLAG_PANO_ROLL                                               \
  (1 << 2) // 1 - панорамная съемка будет происходить с приращением угла по оси
           // Roll Используется с точками типа Панорама
#define NAVI_MISS_FLAG_PANO_YAW                                                \
  (1 << 3) // 1 - панорамная съемка будет происходить с приращением угла по оси
           // Yaw (изменение курса) Используется с точками типа Панорама

#define NAVI_MISS_FLAG_FRRE_AND_RETURN_HEADING                                 \
  (1 << 4) // NAVI_MISS_FLAG_FRRE_AND_RETURN_HEADING
           // NAVI_MISS_FLAG_FIXED_HEADING 00 - Свободное управление курсом при
           // выполнении мисси 01 - Разрешать управлять курсом, но возвращать к
           // указанному в миссии 10 - Запретить ручное управление курсом в
           // миссии 11 - Не изменять курс (игнорировать команду смены курса при
           // загрузке)
#define NAVI_MISS_FLAG_FIXED_HEADING (1 << 5) //

#define NAVI_MISS_FLAG_RELATIVE_COORD                                          \
  (1 << 6) // 1 - означает исчисление координат целевой точки относительно ранее
           // заданной в миссии базовой точки
#define NAVI_MISS_FLAG_SET_AUTO_HEADING_TO_TARGET                              \
  (1 << 7) // 1 - означает, что при загрузке нового задания (целевой точки)
           // аппарат развернётся курсом на цель
#define NAVI_MISS_FLAG_SYNC_TIME                                               \
  (1 << 8) // 1 - означает, что через время, указанное в loiterTime, в любом
           // случае будет произведён переход на следующую точку миссии. При
           // этом не важно, достигнута уже цель или нет. Актуально для
           // сихромиссий (шоу дронов и т.п.)
/*Флаги состояния САУ
Флаги в TELEM_FLAGS:*/
enum systemFlags {
  FLAG_SD_CARD_INSERTED = 0x0001,
  FLAG_HEATING_ON = 0x0002,
  FLAG_SD_CARD_WORKED = 0x0004,
  FLAG_NAV_ALT_HOLD_MODE = 0x0008,
  FLAG_TAKEOFF_ACTIVE = 0x0010,
  FLAG_TO_LAND_ACTIVE = 0x0020,
  FLAG_MISSION_CORRECT_MODE = 0x0040,
  FLAG_MSC_ACTIVE = 0x0080,
  FLAG_GUARD_ON = 0x0100,
  FLAG_DOSIMETER_ACTIVE = 0x0200,
};

/*Флаги в TELEM_STATE:*/
enum systemStateFlags {
  SYS_STATE_FLAG_INITIALIZING = 0x0000,
  SYS_STATE_FLAG_DISARMED = 0x0001,
  SYS_STATE_FLAG_ARMED = 0x0002,
  SYS_STATE_FLAG_FLYING = 0x0004,
  SYS_STATE_FLAG_RADIO_LOSS_STEP1 = 0x0008,
  SYS_STATE_FLAG_RADIO_LOSS_STEP2 = 0x0010,
  SYS_STATE_FLAG_BATTERY_WARNING = 0x0020,
  SYS_STATE_FLAG_BATTERY_ALERT = 0x0040,
  SYS_STATE_FLAG_MAG_CALIBRATE = 0x0080,
  SYS_STATE_FLAG_DATALINK_LOSS_STEP1 = 0x0100,
  SYS_STATE_FLAG_DATALINK_LOSS_STEP2 = 0x0200,
  SYS_STATE_FLAG_FOLLOW_ME_LOSS_STEP1 = 0x0400,
  SYS_STATE_FLAG_FOLLOW_ME_LOSS_STEP2 = 0x0800,
};

/************************************************/
enum missPointTypes {
  NAVI_POINT_NOT_DEFINED = 0,
  NAVI_POINT_HOME,    //Лететь домой.
  NAVI_POINT_TAKEOFF, //Автоматический взлёт. В случае, если аппарат находится в
                      //состоянии DISARMED (в покое на земле), и по команде
                      //загрузки полётного задания будет обнаружена точка типа
                      //"Взлёт", то произойдёт постепенный набор оборотов
                      //двигателей аппарата, плавный отрыв от земли, и выход на
                      //указанную в параметрах точки высоту, в которой он
                      //зависнет на время, указанное в поле loiterTime
  NAVI_POINT_SIMPLE, //Обычная точка миссии.	Наиболее популярный тип точки.
  NAVI_POINT_CIRCLE, //Облёт целевой точки по окружности с радиусом, указанным в
                     //поле targetRadius
  NAVI_POINT_TO_LAND, //Автоматическая посадка.
  NAVI_POINT_PANORAMA, //Автоматическая съемка панорам.
  NAVI_POINT_SET_BASE, //Формальная точка определения начала системы координат,
                       //"нулевой" точкой которой является текущее расположение
                       //аппарата в пространстве (его долгота, широта, высота,
                       //курс).
  NAVI_POINT_NON_STOP, //точка, при проходе которой, аппарат не замедляется, а
                       //пролетает по траектории, аналогичной на картинке:
                       // http://man.copterworks.ru/index.php/%D0%A4%D0%B0%D0%B9%D0%BB:Wp_non_stop.png
  NAVI_POINT_WAIT_FREE_FALL, //Автоматический запуск моторов (и всей миссии),
                             //когда аппарат свободно падает с высоты.
  NAVI_POINT_TYPES_COUNT //
};

/*команды и ответы на команды*/
enum cmdControlTypes {
  CMD_ACK = 0x0000,
  CMD_GET_DEVICE_INFO = 0x0001,
  CMD_DEVICE_INFO = 0x0002,
  CMD_TEXT = 0x0003,
  CMD_NACK = 0x0004,
  CMD_PING = 0x0005,
  CMD_SYSTEM_RESET = 0x0006,
  CMD_CALIBRATE_MAG = 0x0007,
  CMD_CALIBRATE_ACC = 0x0008,
  CMD_MOTORS_ACTIVE_STATE = 0x0009,
  CMD_TEST_MOTOR_VALUE = 0x000A,
  CMD_OSD_WRITE_ALL_CHARS = 0x000B,
  CMD_OSD_SHOW_ALL_CHARS = 0x000C,
  CMD_SD_FORMAT = 0x000D,
  CMD_SD_INFO = 0x000E,
  CMD_SD_FILES_LIST = 0x000F,
  CMD_GPS_PASSTHROUGH = 0x0010,
  CMD_RETRAC_ON = 0x0011,
  CMD_RETRAC_OFF = 0x0012,
  CMD_MAG_ACCEPT_NEW_DECL = 0x0013,
  CMD_MAG_ACCEPT_NEW_INCL = 0x0014,
  CMD_TRANSP_UART_TO_DEVICE = 0x0015,
  CMD_TRANSP_UART_FROM_DEVICE = 0x0016,
  CMD_POOL = 0x0017,
  CMD_SET_PWM_VALUE = 0x0018,
  CMD_GUARD_SET_STATE = 0x0019,
  /*выбрать полётное задание*/
  CMD_EXEC_SWITCH_N = 0x001B, //в парамерах 1 байт от 1 до 10 это количество
                              //кликов кнопки, 255 это длинное нажатие
  CMD_GNSS_USE = 0x001C, //в парамерах 1 байт 0=не использовать внутренний GPS
                         // 1=Использовать GPS
  /**************CMD_SET_NAV_MODE***************
   *    0 - мануальный (пульт или просто держать нулевой угол)
   *    1 - только высоту
   *    2 - позицию и высоту
   *    3 - миссия
   *    остальные не юзать
   */
  CMD_SET_NAV_MODE = 0x001D,
  /******MD_SET_MAN_TARGET_ANGLES*********
   * параметры - два флоата питч и ролл на которые надо наклониться
   * перед их юзанием нужно установить режим 1 по команде ,которую выше описывал
   */
  CMD_SET_MAN_TARGET_ANGLES = 0x001E,
  CMD_PHOTO_CMD = 0x0040,
  CMD_PHOTO_SET_DELTA_TIME = 0x0041,
  CMD_PHOTO_SET_PITCH_ABS_ANGLE = 0x0042,
  CMD_PHOTO_SET_ROLL_ABS_ANGLE = 0x0043,
  CMD_CAMERA_ZOOM_INC_DEC = 0x0045,
  CMD_CAMERA_ZOOM_INFO_GET = 0x0046,
  CMD_CAMERA_ZOOM_INFO = 0x0047,
  CMD_GBL_WRITE_CONFIG = 0x0080,
  CMD_GBL_SET_ABS_POS = 0x0081,
  CMD_GBL_ADD_DELTA_ANGLE = 0x0082,
  CMD_GBL_SCAN = 0x0083,
  CMD_GBL_SET_DELTA_POS = 0x0084,
  CMD_GBL_SET_POI = 0x0085,
  CMD_GBL_PAUSE = 0x0086,
  CMD_GBL_CONTINUE = 0x0087,
  CMD_GBL_SWITCHES = 0x0088,
  CMD_GBL_CALIBRATE = 0x0089,
  CMD_TELEM_WRITE_CONFIG = 0x0100,
  CMD_FC_TELEM_READ_CONFIG = 0x0101,
  CMD_FC_TELEM_PRESET_BITMASK = 0x0102,
  /*Телеметрия от САУ. Шлётся в режиме "флуда" без подтверждения о доставке.*/
  TELEMETRY_DATA =
      0x0103, //Телеметрия от коптера. Номер пресета + массив данных (передаются
              //по битовой маске и настройке команды передаваемых параметров,
              //последовательность не меняется).
  CMD_TELEM_SAVE_CONFIG = 0x0104,
  CMD_NAV_PRE_START_CHECK = 0x0105,
  CMD_NAV_PRE_START_ANSWER = 0x0106,
  CMD_NAV_MOTORS_ON = 0x0107,
  CMD_NAV_MOTORS_ON_ANSWER = 0x0108,
  CMD_NAV_MOTORS_OFF = 0x0109,
  CMD_NAV_TAKE_OFF = 0x010A,
  CMD_NAV_TO_LAND = 0x010B,
  CMD_NAV_STOP_MOTION = 0x010C,
  CMD_NAV_CONTINUE_MOTION = 0x010D,
  CMD_NAV_SET_ABS_HEADING = 0x010E,
  CMD_NAV_ADD_HEADING = 0x010F,
  CMD_NAV_SET_TARGET_ABS = 0x0110,
  CMD_NAV_SET_MOVE = 0x0111,
  CMD_NAV_SET_MOVE_ABS = 0x0112,
  CMD_NAV_SET_CONTROL_MODE = 0x0113,
  CMD_NAV_SET_HOME = 0x0114,
  CMD_NAV_GOTO_HOME = 0x0115,
  CMD_TELEM_ENABLE = 0x0116, //Включить или выключить (зависит от параметров)
                             //передачу пресетов телеметрии
  CMD_TELEM_DISABLE = 0x0117, //Выключить передачу телеметрии
  CMD_TELEM_GET_PARAMS_COUNT = 0x0118,
  CMD_TELEM_PARAMS_COUNT = 0x0119,
  CMD_TELEM_GET_PARAM_NAME = 0x011A,
  CMD_TELEM_PARAM_NAME = 0x011B,
  CMD_CONFIG_GET_PARAMS_COUNT = 0x011C,
  CMD_CONFIG_PARAMS_COUNT = 0x011D,
  CMD_CONFIG_GET_PARAM_NAME = 0x011E,
  CMD_CONFIG_PARAM_NAME = 0x011F,
  CMD_CONFIG_GET_PARAMS_VALUE = 0x0120,
  CMD_CONFIG_PARAMS_VALUE = 0x0121,
  CMD_CONFIG_SET_PARAMS_VALUE = 0x0122,
  CMD_CONFIG_SAVE_TO_EEPROM = 0x0123,
  CMD_CONFIG_FACTORY_RELOAD = 0x0124,
  CMD_NAV_WRITE_POINT = 0x0180, // CMD_NAV_SAVE_POINT = 0x0180
  CMD_NAV_LOAD_POINT = 0x0181,
  CMD_NAV_POINT_INFO = 0x0182,
  CMD_NAV_EXEC_POINT = 0x0183,
  CMD_NAV_WAIT_POINT_AND_STOP = 0x0185,
  CMD_NAV_CLEAR_WP = 0x0186,
  CMD_NAV_DELETE_WP = 0x0187,
  CMD_NAV_INSERT_WP = 0x0188,
  CMD_NAV_GET_WP_COUNT = 0x0189,
  CMD_NAV_WP_COUNT = 0x018A,
  CMD_NAV_GET_WP_TOTAL_COUNT = 0x018D,
  CMD_NAV_WP_TOTAL_COUNT = 0x018E,
  CMD_FC_NAV_LOAD_MISS_FROM_uSD = 0x018B,
  CMD_FC_NAV_SAVE_MISS_TO_uSD = 0x018C,
  CMD_NAV_FSP_CLEAR = 0x0190,
  CMD_FSP_ADD_CURRENT = 0x0191,
  CMD_FSP_LOAD = 0x0192,
  CMD_FSP_DATA = 0x0193,
  CMD_FSP_WRITE = 0x0194,
  CMD_FSP_EXEC = 0x0195,
  CMD_NAV_FOLLOW_ME_ACTIVE = 0x01A0,
  CMD_NAV_FOLLOW_ME_PARAMS = 0x01A1,
  CMD_NAV_FOLLOW_ME_SET_HOME = 0x01A2,
  CMD_OPTIC_FLOW_INFO = 0x01C0,
  CMD_RTK_RTCM_DATA = 0x01D0,
  CMD_NMEA_CMD_STR = 0x01D1,
  CMD_HALT_TEST = 0x01D2,
  CMD_SET_VIDEO_IN = 0x01D3,
  CMD_SET_LEVEL1 = 0x01D4,
  CMD_UART6_PASSTHROUGH = 0x01D5,
  CMD_UART6_AUX_1 = 0x01D6,
  CMD_TO_GOES = 0x01D7,
  CMD_FROM_GOES = 0x01D8,
  CMD_WFF_SERVO_SET = 0x01D9,
  CMD_WFF_INTERCOM = 0x01DA,
  CMD_WFF_SERVO_CMD = 0x01DB,
  CMD_LORA_READ_CFG = 0x01DC,
  CMD_LORA_CFG_DATA = 0x01DD,
  CMD_LORA_WRITE_CFG = 0x01DE,
  CMD_TO_NRL = 0x01DF,
  CMD_LOGBOOK_ERASE = 0x01E0,
  CMD_LOGBOOK_DATA_REQ = 0x01E1,
  CMD_LOGBOOK_DATA = 0x01E2,
  CMD_LOGBOOK_STAT_REQ = 0x01E3,
  CMD_LOGBOOK_STAT = 0x01E3,
  CMD_LOGBOOK_AGGREGATE = 0x01E4,
  CMD_LOGBOOK_SAVE_TO_FILE = 0x01E5,

  CMD_FC_SET_HOLD_HEADING = 0x01F0,
  CMD_FC_ADD_HOLD_HEADING = 0x01F1,
  CMD_FC_SET_HOLD_ALTITUDE = 0x01F2,
  CMD_FC_ADD_HOLD_ALTITUDE = 0x01F3,
  CMD_FC_SET_HOLD_ORBIT = 0x01F4,

};
enum errorsPreCheckBitmask {
  ERR_PCBM_NONE = 0,
  ERR_PCBM_RADIO_QUALITY = (1 << 0),
  ERR_PCBM_THRO_STATE = (1 << 1),
  ERR_PCBM_NAVI_SWITCH = (1 << 2),
  ERR_PCBM_ALT_SWITCH = (1 << 3),
  ERR_PCBM_BATTERY_LOW = (1 << 4),
  ERR_PCBM_IMU_SENSORS = (1 << 5),
  ERR_PCBM_PITCH_ROLL = (1 << 6),
  ERR_PCBM_YAW = (1 << 7),
  ERR_PCBM_MAG_X = (1 << 8),
  ERR_PCBM_MAG_Y = (1 << 9),
  ERR_PCBM_MAG_Z = (1 << 10),
  ERR_PCBM_BARO = (1 << 11),
  ERR_PCBM_CD_CARD = (1 << 12),
  ERR_PCBM_DATA_LINK_LOST = (1 << 13),
};
/*Параметры*/
enum telemetryParameters {
  TELEM_VBAT = 0,
  TELEM_CURRENT_A,
  TELEM_POWER_WATT,
  TELEM_CAPACITY_MAH,
  TELEM_ENERGY_WH,
  TELEM_RADIO_QUALITY,
  TELEM_CPU_LOAD,
  TELEM_FLIGHT_TIME,
  TELEM_FLIGHT_TIME_REMAINING,
  TELEM_FLIGHT_TIME_SOC,
  TELEM_BAT_CELL_COUNT,
  TELEM_ANGLE_PITCH,
  TELEM_ANGLE_ROLL,
  TELEM_EXT_TEMP,
  TELEM_TEMP,
  TELEM_STATE,
  TELEM_FLAGS,
  TELEM_GYRO_RAW_X_ROLL,
  TELEM_GYRO_RAW_Y_PITCH,
  TELEM_GYRO_RAW_Z_Yaw,
  TELEM_GYRO_PITCH,
  TELEM_GYRO_ROLL,
  TELEM_GYRO_YAW,
  TELEM_GYRO_TEMP1,
  TELEM_GYRO_TEMP2,
  TELEM_ACC_RAW_X,
  TELEM_ACC_RAW_Y,
  TELEM_ACC_RAW_Z,
  TELEM_ACC_X,
  TELEM_ACC_Y,
  TELEM_ACC_Z,
  TELEM_ACC_BIAS_X,
  TELEM_ACC_BIAS_Y,
  TELEM_ACC_BIAS_Z,
  TELEM_ACC_STD_Z,
  TELEM_MAG_X,
  TELEM_MAG_Y,
  TELEM_MAG_Z,
  TELEM_ABSOLUTE_HEADING,
  TELEM_HOLD_HEADING,
  TELEM_ACTUAL_HEADING,
  TELEM_BARO1_ADC,
  TELEM_BARO2_ADC,
  TELEM_BARO1_ALT,
  TELEM_BARO2_ALT,
  TELEM_BARO_MS5611_ALT,
  TELEM_BARO_MS5611_TEMP,
  TELEM_ALTITUDE,
  TELEM_ALTITUDE_ABS,
  TELEM_ALTITUDE_HOLD,
  TELEM_GPS_ALT,
  TELEM_ALTITUDE_HOLD_SPEED,
  TELEM_ALTITUDE_VELOCITY,
  TELEM_ACTUAL_ALT_VELOCITY,
  TELEM_BARO_ALT_VELOCITY,
  TELEM_RANGE_SENSOR1,
  TELEM_RANGE_STD1,
  TELEM_RANGE_SENSOR2,
  TELEM_RANGE_STD2,
  TELEM_MILEAGE,
  TELEM_MILEAGE_TOTAL,
  TELEM_ENERGY_EFFICIENT,
  TELEM_AMP_EFFICIENT,
  TELEM_POSITION_NORTH,
  TELEM_POSITION_EAST,
  TELEM_VELOCITY_NORTH,
  TELEM_VELOCITY_EAST,
  TELEM_GPS_LAT,
  TELEM_GPS_LON,
  TELEM_GPS_GOOD_FLAG,
  TELEM_GPS_HACC,
  TELEM_GPS_VACC,
  TELEM_GPS_VELD,
  TELEM_GPS_HEADING,
  TELEM_GPS_PDOP,
  TELEM_GPS_HDOP,
  TELEM_GPS_VDOP,
  TELEM_GPS_TDOP,
  TELEM_GPS_NDOP,
  TELEM_GPS_EDOP,
  TELEM_GPS_GDOP,
  TELEM_HOLD_COURSE,
  TELEM_HOLD_DISTANCE,
  TELEM_HOME_LAT,
  TELEM_HOME_LON,
  TELEM_HOME_ALT,
  TELEM_DISTANCE_TO_HOME,
  TELEM_HOLD_LAT,
  TELEM_HOLD_LON,
  TELEM_POI_LAT,
  TELEM_POI_LON,
  TELEM_GPS_SATELLITES_FLAGS,
  TELEM_GPS_SATELLITES_FIX,
  TELEM_GPS_NUMBER_OF_SATELLITES,
  TELEM_NAV_MODE,
  TELEM_MICROSPERSECOND,
  TELEM_MIXER_THROTTLE,
  TELEM_MIXER_AVG_THROTTLE,
  TELEM_MIXER_PITCH,
  TELEM_MIXER_ROLL,
  TELEM_MIXER_RUDDER,
  TELEM_MOTORS_PWM1,
  TELEM_MOTORS_PWM2,
  TELEM_MOTORS_PWM3,
  TELEM_MOTORS_PWM4,
  TELEM_MOTORS_PWM5,
  TELEM_MOTORS_PWM6,
  TELEM_MOTORS_PWM7,
  TELEM_MOTORS_PWM8,
  TELEM_MOTORS_PWM9,
  TELEM_MOTORS_PWM10,
  TELEM_MOTORS_PWM11,
  TELEM_MOTORS_PWM12,
  TELEM_MOTORS_PWM13,
  TELEM_MOTORS_PWM14,
  TELEM_MOTORS_PWM15,
  TELEM_MOTORS_PWM16,
  TELEM_MOTORS_PWM17,
  TELEM_MOTORS_PWM18,
  TELEM_MOTORS_RPM1,
  TELEM_MOTORS_RPM2,
  TELEM_MOTORS_RPM3,
  TELEM_MOTORS_RPM4,
  TELEM_MOTORS_RPM5,
  TELEM_MOTORS_RPM6,
  TELEM_MOTORS_RPM7,
  TELEM_MOTORS_RPM8,
  TELEM_MOTORS_RPM9,
  TELEM_MOTORS_RPM10,
  TELEM_MOTORS_RPM11,
  TELEM_MOTORS_RPM12,
  TELEM_MOTORS_RPM13,
  TELEM_MOTORS_RPM14,
  TELEM_MOTORS_RPM15,
  TELEM_MOTORS_RPM16,
  TELEM_MOTORS_RPM17,
  TELEM_MOTORS_RPM18,
  TELEM_MOTORS_CURRENT1,
  TELEM_MOTORS_CURRENT2,
  TELEM_MOTORS_CURRENT3,
  TELEM_MOTORS_CURRENT4,
  TELEM_MOTORS_CURRENT5,
  TELEM_MOTORS_CURRENT6,
  TELEM_MOTORS_CURRENT7,
  TELEM_MOTORS_CURRENT8,
  TELEM_MOTORS_CURRENT9,
  TELEM_MOTORS_CURRENT10,
  TELEM_MOTORS_CURRENT11,
  TELEM_MOTORS_CURRENT12,
  TELEM_MOTORS_CURRENT13,
  TELEM_MOTORS_CURRENT14,
  TELEM_MOTORS_CURRENT15,
  TELEM_MOTORS_CURRENT16,
  TELEM_MOTORS_CURRENT17,
  TELEM_MOTORS_CURRENT18,
  TELEM_MOTORS_TEMP1,
  TELEM_MOTORS_TEMP2,
  TELEM_MOTORS_TEMP3,
  TELEM_MOTORS_TEMP4,
  TELEM_MOTORS_TEMP5,
  TELEM_MOTORS_TEMP6,
  TELEM_MOTORS_TEMP7,
  TELEM_MOTORS_TEMP8,
  TELEM_MOTORS_TEMP9,
  TELEM_MOTORS_TEMP10,
  TELEM_MOTORS_TEMP11,
  TELEM_MOTORS_TEMP12,
  TELEM_MOTORS_TEMP13,
  TELEM_MOTORS_TEMP14,
  TELEM_MOTORS_TEMP15,
  TELEM_MOTORS_TEMP16,
  TELEM_MOTORS_TEMP17,
  TELEM_MOTORS_TEMP18,
  TELEM_RC1,
  TELEM_RC2,
  TELEM_RC3,
  TELEM_RC4,
  TELEM_RC5,
  TELEM_RC6,
  TELEM_RC7,
  TELEM_RC8,
  TELEM_RC9,
  TELEM_RC10,
  TELEM_RC11,
  TELEM_RC12,
  TELEM_RC13,
  TELEM_RC14,
  TELEM_RC15,
  TELEM_RC16,
  TELEM_RC17,
  TELEM_RC18,
  TELEM_RAW_ADC0,
  TELEM_RAW_ADC1,
  TELEM_RAW_ADC2,
  TELEM_RAW_ADC3,
  TELEM_RAW_ADC4,
  TELEM_RAW_ADC5,
  TELEM_RAW_ADC6,
  TELEM_RAW_ADC7,
  TELEM_RAW_ADC8,
  TELEM_RAW_ADC9,
  TELEM_RAW_ADC10,
  TELEM_RAW_ADC11,
  TELEM_RAW_ADC12,
  TELEM_RAW_ADC13,
  TELEM_RAW_ADC14,
  TELEM_RAW_ADC15,
  TELEM_RAW_ADC16,
  TELEM_RAW_ADC17,
  TELEM_RAW_ADC18,
  TELEM_RAW_ADC19,
  TELEM_RAW_ADC20,
  TELEM_RAW_ADC21,
  TELEM_RAW_ADC22,
  TELEM_RAW_ADC23,
  TELEM_DGYRO_X,
  TELEM_DGYRO_Y,
  TELEM_DGYRO_Z,
  TELEM_STDGYROX,
  TELEM_STDGYROY,
  TELEM_STDGYROZ,
  TELEM_STDGYROSUM,
  TELEM_GIMBAL_PWM_PITCH,
  TELEM_GIMBAL_PWM_ROLL,
  TELEM_GIMBAL_ANGLE_ROLL,
  TELEM_GIMBAL_ANGLE_PITCH,
  TELEM_GIMBAL_ANGLE_YAW,
  TELEM_DEBUG1,
  TELEM_DEBUG2,
  TELEM_DEBUG3,
  TELEM_DEBUG4,
  TELEM_DEBUG5,
  TELEM_DEBUG6,
  TELEM_DEBUG7,
  TELEM_DEBUG8,
  TELEM_ALT_PSTART_MAP,
  TELEM_PHOTO_FLAGS,
  TELEM_PHOTO_SHOT_CNT,
  TELEM_NAV_ACTIVE_WP,
  TELEM_NAV_SAT_TOTAL_GPS,
  TELEM_NAV_SAT_TOTAL_GLONASS,
  TELEM_NAV_SAT_USED_GPS,
  TELEM_NAV_SAT_USED_GLONASS,
  TELEM_NAV_HOLD_MAX_HSPEED,
  TELEM_RTK_DIFF_AGE,
  TELEM_FFT_DIV,
  TELEM_MARKER_LAT,
  TELEM_MARKER_LON,
  TELEM_MARKER_ALT,
  TELEM_MARKER_HACC,
  TELEM_MARKER_AGE,
  TELEM_IN_DATA_AGE,
  TELEM_RTKA_PRJ_LAT,
  TELEM_RTKA_PRJ_LON,
  TELEM_RTKA_PRJ_ALT,
  TELEM_RTKA_DIST,
  TELEM_RTKA_HEADING,
  TELEM_RTKA_ANGLE,
  TELEM_RTKA_MODE,
  TELEM_MUX_TEMPERATURE1,
  TELEM_MUX_TEMPERATURE2,
  TELEM_MUX_TEMPERATURE3,
  TELEM_MUX_TEMPERATURE4,
  TELEM_MUX_TEMPERATURE5,
  TELEM_MUX_TEMPERATURE6,
  TELEM_MUX_TEMPERATURE7,
  TELEM_MUX_TEMPERATURE8,
  TELEM_ADC_TEMPERATURE1,
  TELEM_ADC_TEMPERATURE2,
  TELEM_CUR_SENS1_A,
  TELEM_CUR_SENS2_A,
  TELEM_CUR_SENS3_A,
  TELEM_CUR_SENS4_A,
  TELEM_ENERGY_EFFICIENT_ALT,
  TELEM_AMP_EFFICIENT_ALT,
  TELEM_VOLT_SENS1,
  TELEM_VOLT_SENS2,
  TELEM_VOLT_SENS3,
  TELEM_VOLT_SENS4,
  TELEM_CAMERA_ENCODER1,
  TELEM_CAMERA_ENCODER2,
  TELEM_CAMERA_ENCODER3,
  TELEM_CAMERA_EULER1,
  TELEM_CAMERA_EULER2,
  TELEM_CAMERA_EULER3,
  TELEM_CAMERA_ZOOM,
  TELEM_CAMERA_ZOOM_STATE,
  TELEM_CAMERA_OPTICAL_AXIS_LAT,
  TELEM_CAMERA_OPTICAL_AXIS_LON,
  TELEM_CAMERA_OPTICAL_AXIS_DIST,
  TELEM_LIDAR_RANGE,
  TELEM_DOSIMETER_DATA1,
  TELEM_DOSIMETER_DATA2,
  TELEM_DOSIMETER_DATA_TYPE,
  TELEM_DOSIMETER_DATA_STATUS,
  TELEM_DOSIMETER_VIN,
  TELEM_DOSIMETER_DATA3,
  TELEM_DOSIMETER_DATA_SEC,
  TELEM_NAV_AXIS_DIST,
  TELEM_LIDAR_VELOCITY,
  TELEM_GPS_VEL_N,
  TELEM_GPS_VEL_E,
  TELEM_LPS_LAT,
  TELEM_LPS_LON,
  TELEM_LPS_X,
  TELEM_LPS_Y,
  TELEM_LPS_Z,
  TELEM_LPS_VELX,
  TELEM_LPS_VELY,
  TELEM_LPS_VELZ,
  TELEM_LPS_TAGSTATE,
  TELEM_LPS_GOODINDATAFREQ,
  TELEM_LPS_ANCHOR1_DISTANCE,
  TELEM_LPS_ANCHOR1_POWERRX,
  TELEM_LPS_ANCHOR1_VIN,
  TELEM_LPS_ANCHOR2_DISTANCE,
  TELEM_LPS_ANCHOR2_POWERRX,
  TELEM_LPS_ANCHOR2_VIN,
  TELEM_LPS_ANCHOR3_DISTANCE,
  TELEM_LPS_ANCHOR3_POWERRX,
  TELEM_LPS_ANCHOR3_VIN,
  TELEM_LPS_ANCHOR4_DISTANCE,
  TELEM_LPS_ANCHOR4_POWERRX,
  TELEM_LPS_ANCHOR4_VIN,
  TELEM_LPS_ANCHOR5_DISTANCE,
  TELEM_LPS_ANCHOR5_POWERRX,
  TELEM_LPS_ANCHOR5_VIN,
  TELEM_LPS_ANCHOR6_DISTANCE,
  TELEM_LPS_ANCHOR6_POWERRX,
  TELEM_LPS_ANCHOR6_VIN,
  TELEM_LPS_ANCHOR7_DISTANCE,
  TELEM_LPS_ANCHOR7_POWERRX,
  TELEM_LPS_ANCHOR7_VIN,
  TELEM_LPS_ANCHOR8_DISTANCE,
  TELEM_LPS_ANCHOR8_POWERRX,
  TELEM_LPS_ANCHOR8_VIN,
  TELEM_CAMERA_TEMPERATURE1,
  TELEM_CAMERA_TEMPERATURE2,
  TELEM_CAMERA_TEMPERATURE3,
  TELEM_CAMERA_TEMPERATURE4,
  TELEM_MOTORS_STATE1,
  TELEM_MOTORS_STATE2,
  TELEM_MOTORS_STATE3,
  TELEM_MOTORS_STATE4,
  TELEM_MOTORS_STATE5,
  TELEM_MOTORS_STATE6,
  TELEM_MOTORS_STATE7,
  TELEM_MOTORS_STATE8,
  TELEM_MOTORS_STATE9,
  TELEM_MOTORS_STATE10,
  TELEM_MOTORS_STATE11,
  TELEM_MOTORS_STATE12,
  TELEM_MOTORS_STATE13,
  TELEM_MOTORS_STATE14,
  TELEM_MOTORS_STATE15,
  TELEM_MOTORS_STATE16,
  TELEM_MOTORS_STATE17,
  TELEM_MOTORS_STATE18,
  TELEM_RADIO_FPS,
  TELEM_RADIO_SYNC_ERRORS,
  TELEM_OPTIC_FLOW_DELTA_X,
  TELEM_OPTIC_FLOW_DELTA_Y,
  TELEM_LIDAR_LIDAR_QUALITY,
  TELEM_TOF_RANGE,
  TELEM_ACC_MAG,
  TELEM_GYRO_TOTAL_SPEED,
  TELEM_SIDESLIP_DISTURB,
  TELEM_ACC_SIDESLIP,
  TELEM_ANGLE_TARGET_PITCH,
  TELEM_ANGLE_TARGET_ROLL,
  TELEM_PID_PITCH_ANGLE_P,
  TELEM_PID_PITCH_ANGLE_I,
  TELEM_PID_PITCH_ANGLE_D,
  TELEM_PID_PITCH_RATE_P,
  TELEM_PID_PITCH_RATE_I,
  TELEM_PID_PITCH_RATE_D,
  TELEM_PID_ROLL_ANGLE_P,
  TELEM_PID_ROLL_ANGLE_I,
  TELEM_PID_ROLL_ANGLE_D,
  TELEM_PID_ROLL_RATE_P,
  TELEM_PID_ROLL_RATE_I,
  TELEM_PID_ROLL_RATE_D,
  TELEM_PID_YAW_ANGLE_P,
  TELEM_PID_YAW_ANGLE_I,
  TELEM_PID_YAW_ANGLE_D,
  TELEM_PID_YAW_RATE_P,
  TELEM_PID_YAW_RATE_I,
  TELEM_PID_YAW_RATE_D,
  TELEM_PID_RUDD_ANGLE_P,
  TELEM_PID_RUDD_ANGLE_I,
  TELEM_PID_RUDD_ANGLE_D,
  TELEM_PID_SIDE_SLIP_P,
  TELEM_PID_SIDE_SLIP_I,
  TELEM_PID_SIDE_SLIP_D,
  TELEM_PID_ALT_DIST_P,
  TELEM_PID_ALT_DIST_I,
  TELEM_PID_ALT_DIST_D,
  TELEM_PID_ALT_VEL_P,
  TELEM_PID_ALT_VEL_I,
  TELEM_PID_ALT_VEL_D,
  TELEM_PID_NAV_DIST_N_P,
  TELEM_PID_NAV_DIST_N_I,
  TELEM_PID_NAV_DIST_N_D,
  TELEM_PID_NAV_DIST_E_P,
  TELEM_PID_NAV_DIST_E_I,
  TELEM_PID_NAV_DIST_E_D,
  TELEM_PID_NAV_SPEED_N_P,
  TELEM_PID_NAV_SPEED_N_I,
  TELEM_PID_NAV_SPEED_N_D,
  TELEM_PID_NAV_SPEED_E_P,
  TELEM_PID_NAV_SPEED_E_I,
  TELEM_PID_NAV_SPEED_E_D,
  TELEM_JOY_SW_1,
  TELEM_JOY_SW_2,
  TELEM_JOY_SW_3,
  TELEM_JOY_SW_4,
  TELEM_PID_TO_AXIS_P,
  TELEM_PID_TO_AXIS_I,
  TELEM_PID_TO_AXIS_D,
  TELEM_NAV_AXIS_ADD_ANG,
  TELEM_NRL_2G,
  TELEM_NRL_3G,
  TELEM_NRL_VIN,
  TELEM_NRL_POWER,
  TELEM_GYRO_BIAS_PITCH,
  TELEM_GYRO_BIAS_ROLL,
  TELEM_GYRO_BIAS_YAW,
  TELEM_FM24_RANGE,
  TELEM_NRA24_RANGE,
  TELEM_NRA24_RCS,
  TELEM_NRA24_NO_DATA_LOOPS,
  TELEM_RF0_STATUS,
  TELEM_RF0_NOISE_PER_MS,
  TELEM_RF0_AGC_CNT,
  TELEM_RF0_JAM_IND,
  TELEM_RF0_OFS_I,
  TELEM_RF0_MAG_I,
  TELEM_RF0_OFS_Q,
  TELEM_RF0_MAG_Q,
  TELEM_RF1_STATUS,
  TELEM_RF1_NOISE_PER_MS,
  TELEM_RF1_AGC_CNT,
  TELEM_RF1_JAM_IND,
  TELEM_RF1_OFS_I,
  TELEM_RF1_MAG_I,
  TELEM_RF1_OFS_Q,
  TELEM_RF1_MAG_Q,
  TELEM_NRA24_RANGE_HOLD,
  TELEM_LIDAR_RANGE_HOLD,
  TELEM_LIDAR_RANGE_LPF,
  TELEM_NRA24_RANGE_LPF,
  TELEM_LIDAR_RANGE_ALT,
  TELEM_LIDAR_RANGE_X,
  TELEM_LPS_ERR_CODE,
  TELEM_LPS_DATA_AGE,
  TELEM_LPS_TO_GNSS_DIST,
  TELEM_GNSS_M_DIST,
  TELEM_AOP_PWM_OUT_US,
  TELEM_NUM_PARAMS
};
/************************* Типы данных ****************************/
/*Размерность
    TELEM_TYPE_S8  - 1 байт  -128..127
    TELEM_TYPE_U8  - 1 байт   0..255
    TELEM_TYPE_S16 - 2 байта -32768..32767
    TELEM_TYPE_U16 - 2 байта  0..65535
    TELEM_TYPE_S32 - 4 байта -2147483648..2147483647
    TELEM_TYPE_U32 - 4 байта  0..4294967295
    TELEM_TYPE_FLOAT  - 4 байта
    TELEM_TYPE_DOUBLE - 8 байт
*/
/*Размерность*/
enum telemetryDataTypes {
  TELEM_TYPE_S8 = 0,
  TELEM_TYPE_U8,
  TELEM_TYPE_S16,
  TELEM_TYPE_U16,
  TELEM_TYPE_S32,
  TELEM_TYPE_U32,
  TELEM_TYPE_FLOAT,
  TELEM_TYPE_DOUBLE
};
typedef struct {
  uint8_t presetN; //	Номер пресета.
  uint16_t rate; //	Значение, определяющее как часто отправлять телеметрию
                 //этого пресета.
                 /*Примеры:
                 200 - раз в секунду
                 100 - 2 раза в секунду
                 50 - 4 раза в секунду
                 1000 - раз в 5 секунд и т.п.
                  */
  uint16_t offset; //	Значение, определяющее с каким сдвигом отправлять
                   //телеметрию этого пресета.
  /*Смысл параметра: равномерно распределить в времени разные пресеты.
  Пример:
  Пусть определено 3 пресета с rate = 200.
  В этом случае, чтобы равномерно во времени распределить отправку пакетов
  следует указать offset так: Первый с offset=0, второй с offset=66 третий с
  offset=133
   */
  uint8_t bitmask[TELEMETRY_PRESET_BITMASK_LENGTH]; //	Битовый массив, который
                                                    //определяет наличие
                                                    //параметров телеметрии в
                                                    //указанном пресете.
  /*Длина массива в версии САУ на 30.03.2017: TELEMETRY_PRESET_BITMASK_LENGTH =
  40 байт. Этого достаточно, чтобы определить 40*8=320 параметров телеметрии).
  В зависимости от версии ПО САУ эта цифра может быть больше.
  Начиная с 28.05.2017 TELEMETRY_PRESET_BITMASK_LENGTH = 80 байт. Этого
  достаточно, чтобы определить 80*8=640 параметров телеметрии.
*/
} telemetryParam_t_;

typedef struct {
  uint16_t TELEM_NUM; //
  uint8_t TELEM_TYPE; //
} telemetryFields_t;
typedef struct {
  float groundAlt; //
  float speedDecAltitude;
  float altVel;
  float endVel; //
} landBody_t;
typedef struct {
  double lat; //
  double lon;
  float alt;
  float hspeed;
  float vspeed; //
} pointGoTo_t;
#pragma pack(pop)

#ifdef telFld
#endif /*telFld*/

#endif /* COPA_TYPES_H_ */
