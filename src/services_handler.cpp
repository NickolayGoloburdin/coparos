#include "copa_types.h"
#include "coparos/Service_command.h"
#include "ros/ros.h"
#include <coparos/Ack.h>
#include <coparos/Command.h>
#include <std_msgs/String.h>
#include <string>
// Класс для работы сервисов передачи команд и миссий в коптер
class ServiceHandler {
public:
  ros::Publisher
      cmd_pub_; //Издатель для отправки команд в модуль коммуникации с коптером
  ros::Subscriber ack_sub_;
  ros::Publisher log_pub_;
  ros::NodeHandle *n; //Указатель на ноду РОС
                      //Сообщение РОС для хранения отпраленной на коптер команды
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
  ros::ServiceServer service_set_pry;
  ros::ServiceServer service_set_mode;
  ros::ServiceServer service_set_yaw;
  std_msgs::String log;
  coparos::Ack ack_;

  bool useFakeGps = false;
  ServiceHandler(ros::NodeHandle *nh) : n(nh) {
    //Инициализация сервисо и подписчиков РОС
    cmd_pub_ = n->advertise<coparos::Command>("/command", 1000);
    ack_sub_ = n->subscribe("/ack", 1000, &ServiceHandler::callback_ack, this);
    log_pub_ = n->advertise<std_msgs::String>("/logging_topic", 1000);
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
        "On_off_replace_gps", &ServiceHandler::set_gps_mode, this);
    service_set_pry = n->advertiseService("Set_pitch_roll_yaw",
                                          &ServiceHandler::set_pry, this);
    service_set_mode =
        n->advertiseService("Set_flight_mode", &ServiceHandler::set_mode, this);
    service_set_yaw =
        n->advertiseService("Set_yaw", &ServiceHandler::set_yaw, this);
  }
  void callback_ack(coparos::Ack &msg) { ack_ = msg; }

  bool arm(coparos::Service_command::Request &req,
           coparos::Service_command::Response &res) {
    //Каждый сервис работает по принципу
    // 1. Отправил команду на коптер 2. Получил ответ об обработке команды 3.
    // Вывел ответ об успешности выполнения команды
    log.data = "ARMING";
    log_pub_.publish(log);
    coparos::Command msg;
    msg.command = CMD_NAV_MOTORS_ON;
    cmd_pub_.publish(msg);
    ros::Duration(2).sleep();
    ros::spinOnce();
    if (ack.command == CMD_NAV_MOTORS_ON) {
      if (ack.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Other command recieved";
      res.result = false;
      return true;
    }

    //   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  }
  bool disarm(coparos::Service_command::Request &req,
              coparos::Service_command::Response &res) {
    coparos::Command msg;
    log.data = "DISARMING";
    log_pub_.publish(log);
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
  bool set_yaw(coparos::Service_command::Request &req,
               coparos::Service_command::Response &res) {
    log.data = "Setting yaw";
    log_pub_.publish(log);
    coparos::Command msg;
    msg.command = CMD_NAV_SET_ABS_HEADING;
    msg.data1 = req.param1;
    msg.data2 = req.param2;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_SET_ABS_HEADING)) {
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
    log.data = "TAKING OFF";
    log_pub_.publish(log);
    coparos::Command msg;
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
    log.data = "LANDING";
    log_pub_.publish(log);
    coparos::Command msg;
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
    log.data = "RTL";
    log_pub_.publish(log);
    coparos::Command msg;
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
    log.data = "STOPPING";
    log_pub_.publish(log);
    coparos::Command msg;
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
    coparos::Command msg;
    msg.command = CMD_GNSS_USE;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_GNSS_USE)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          useFakeGps = !useFakeGps;
          ROS_INFO("Use fake gps:", useFakeGps);
          n->setParam("/use_gps_from_video", useFakeGps);
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
    coparos::Command msg;
    log.data = "CONTINUE";
    log_pub_.publish(log);
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
    log.data = "Starting mission 1";
    log_pub_.publish(log);
    coparos::Command msg;
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
    log.data = "Clearing mission";
    log_pub_.publish(log);
    coparos::Command msg;
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

  bool set_pry(coparos::Service_command::Request &req,
               coparos::Service_command::Response &res) {
    log.data = "Set Pitch Roll Yaw:" + std::to_string(req.param1) +
               std::to_string(req.param2) + std::to_string(req.param3) +
               std::to_string(req.param4);
    log_pub_.publish(log);
    coparos::Command msg;
    msg.command = CMD_SET_MAN_TARGET_ANGLES;
    msg.data1 = req.param1; // pitch
    msg.data2 = req.param2; // roll
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_SET_MAN_TARGET_ANGLES)) {
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
  bool set_mode(coparos::Service_command::Request &req,
                coparos::Service_command::Response &res) {
    log.data = "Set mode: " + std::to_string(req.param1);
    log_pub_.publish(log);
    coparos::Command msg;
    msg.command = CMD_SET_NAV_MODE;
    msg.data1 = req.param1; // mode
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_SET_NAV_MODE)) {
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