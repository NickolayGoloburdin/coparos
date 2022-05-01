#include "copa_types.h"
#include "coparos/Service_command.h"
#include "ros/ros.h"
#include <coparos/Ack.h>
#include <coparos/Command.h>
// Класс для работы сервисов передачи команд и миссий в коптер
class ServiceHandler {
public:
  ros::Publisher
      cmd_pub_; //Издатель для отправки команд в модуль коммуникации с коптером
  ros::NodeHandle *n; //Указатель на ноду РОС
  coparos::Command
      msg; //Сообщение РОС для хранения отпраленной на коптер команды
  coparos::Ack ack_msg; //Сообщения для храниния ответа на команду с коптера
  ros::Subscriber ack_processor; // Подписчик на ответы с коптера
  ros::ServiceServer service_arm; //Сервис для снятия коптера с предохранителя
  ros::ServiceServer service_disarm; //Сервис для выключения двигателей
  ros::ServiceServer service_takeoff; //Сервис взлета
  ros::ServiceServer service_land;    //Сервис посадки
  ros::ServiceServer service_rtl; //Сервис включения режима Возврат домой
  ros::ServiceServer service_stop; //Сервис остановки
  ros::ServiceServer service_continue_flight; //Сервис продолжения полета
  ros::ServiceServer service_start_mission; //Сервис запуска миссии
  ros::ServiceServer service_clear_mission; //Сервис очистки миссии
  ros::ServiceServer service_other_gps; //Сервис включения подмены gps координат
  bool useFakeGps = false;
  ServiceHandler(ros::NodeHandle *nh) : n(nh) {
    //Инициализация сервисо и подписчиков РОС
    cmd_pub_ = n->advertise<coparos::Command>("/command", 1000);
    service_arm = n->advertiseService("Arm", &ServiceHandler::arm, this);
    service_disarm =
        n->advertiseService("Disarm", &ServiceHandler::disarm, this);
    service_takeoff =
        n->advertiseService("Takeoff", &ServiceHandler::takeoff, this);
    service_land = n->advertiseService("Land", &ServiceHandler::land, this);
    service_rtl = n->advertiseService("RTL", &ServiceHandler::rtl, this);
    service_stop = n->advertiseService("STOP", &ServiceHandler::stop, this);
    service_continue_flight =
        n->advertiseService("Continue", &ServiceHandler::continue_flight, this);
    service_start_mission = n->advertiseService(
        "Start_mission", &ServiceHandler::start_mission, this);
    service_clear_mission = n->advertiseService(
        "Clear_mission", &ServiceHandler::clear_mission, this);
    service_other_gps = n->advertiseService(
        "Set_gps_mode", &ServiceHandler::set_gps_mode, this);
    ack_processor =
        n->subscribe("/ack", 1000, &ServiceHandler::callback_ack, this);
  }
  void callback_ack(const coparos::Ack msg) {
    ack_msg = msg;
  } //Функция обработчик ответа с коптера
  bool arm(coparos::Service_command::Request &req,
           coparos::Service_command::Response &res) {
    //Каждый сервис работает по принципу
    // 1. Отправил команду на коптер 2. Получил ответ об обработке команды 3.
    // Вывел ответ об успешности выполнения команды
    ROS_INFO("ARMING");
    msg.command = CMD_NAV_MOTORS_ON;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == CMD_NAV_MOTORS_ON) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
    //   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  }
  bool disarm(coparos::Service_command::Request &req,
              coparos::Service_command::Response &res) {
    ROS_INFO("DISARMING");
    msg.command = CMD_NAV_MOTORS_OFF;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_MOTORS_OFF)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool takeoff(coparos::Service_command::Request &req,
               coparos::Service_command::Response &res) {
    ROS_INFO("TAKING OFF");
    msg.command = CMD_NAV_TAKE_OFF;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_TAKE_OFF)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool land(coparos::Service_command::Request &req,
            coparos::Service_command::Response &res) {
    ROS_INFO("LANDING");
    msg.command = CMD_NAV_TO_LAND;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_TO_LAND)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }

  bool rtl(coparos::Service_command::Request &req,
           coparos::Service_command::Response &res) {
    ROS_INFO("Returning to launch");
    msg.command = CMD_NAV_GOTO_HOME;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_GOTO_HOME)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool stop(coparos::Service_command::Request &req,
            coparos::Service_command::Response &res) {
    ROS_INFO("Stopping");
    msg.command = CMD_NAV_STOP_MOTION;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_STOP_MOTION)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }

  bool set_gps_mode(coparos::Service_command::Request &req,
                    coparos::Service_command::Response &res) {

    msg.command = CMD_GPS_PASSTHROUGH;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_GPS_PASSTHROUGH)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          useFakeGps = !useFakeGps;
          ROS_INFO("Use fake gps:", useFakeGps);
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          ROS_INFO("Use fake gps:", useFakeGps);
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        ROS_INFO("Use fake gps:", useFakeGps);
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      ROS_INFO("Use fake gps:", useFakeGps);
      return true;
    }
  }
  bool continue_flight(coparos::Service_command::Request &req,
                       coparos::Service_command::Response &res) {
    ROS_INFO("Continue");
    msg.command = CMD_NAV_CONTINUE_MOTION;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_CONTINUE_MOTION)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool start_mission(coparos::Service_command::Request &req,
                     coparos::Service_command::Response &res) {
    ROS_INFO("Starting 1 click mission");
    msg.command = CMD_EXEC_SWITCH_N;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_EXEC_SWITCH_N)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool clear_mission(coparos::Service_command::Request &req,
                     coparos::Service_command::Response &res) {
    ROS_INFO("Cleaning mission");
    msg.command = CMD_NAV_CLEAR_WP;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_CLEAR_WP)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "services_server");
  ros::NodeHandle n;
  ServiceHandler sh(&n);
  ros::spin();

  return 0;
}