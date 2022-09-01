#include "copa_types.h"
#include "coparos/Service_command.h"
#include "ros/ros.h"
#include <coparos/Ack.h>
#include <coparos/Command.h>
#include <coparos/GPS.h>
#include <cstdint>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <string>
// Класс для работы сервисов передачи команд и миссий в коптер
class ServiceHandler {
public:
  ros::Publisher
      cmd_pub_; //Издатель для отправки команд в модуль коммуникации с коптером
  ros::Subscriber ack_sub_;
  ros::Subscriber gps_sub_;
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
  ros::ServiceServer service_download_mission; //Сервис загрузки миссии с дрона
  ros::ServiceServer service_mission_count;
  ros::ServiceServer
      service_enable_gps; //Сервис включения подмены gps координат
  ros::ServiceServer
      service_disable_gps; //Сервис включения подмены gps координат
  ros::ServiceServer fly_to_;
  ros::ServiceServer service_set_pry;
  ros::ServiceServer service_set_mode;
  ros::ServiceServer service_set_yaw;
  ros::ServiceServer service_get_gps;
  std_msgs::String log;
  coparos::Ack ack_;
  sensor_msgs::NavSatFix saved_gps_;

  bool useFakeGps = false;
  ServiceHandler(ros::NodeHandle *nh) : n(nh) {
    //Инициализация сервисо и подписчиков РОС
    cmd_pub_ = n->advertise<coparos::Command>("/command", 1000);
    ack_sub_ = n->subscribe("/ack", 1, &ServiceHandler::callback_ack, this);
    gps_sub_ =
        n->subscribe("/gps", 1, &ServiceHandler::callback_save_gps, this);
    log_pub_ = n->advertise<std_msgs::String>("/logging_topic", 1000);
    service_arm = n->advertiseService("Arm", &ServiceHandler::arm, this);
    service_disarm =
        n->advertiseService("Disarm", &ServiceHandler::disarm, this);
    service_takeoff =
        n->advertiseService("Takeoff", &ServiceHandler::takeoff, this);
    fly_to_ = n->advertiseService("FlyTo", &ServiceHandler::fly_to, this);
    service_land = n->advertiseService("Land", &ServiceHandler::land, this);
    service_rtl = n->advertiseService("RTL", &ServiceHandler::rtl, this);
    service_stop = n->advertiseService("STOP", &ServiceHandler::stop, this);
    service_continue_flight =
        n->advertiseService("Continue", &ServiceHandler::continue_flight, this);
    service_start_mission = n->advertiseService(
        "Start_mission", &ServiceHandler::start_mission, this);
    service_clear_mission = n->advertiseService(
        "Clear_mission", &ServiceHandler::clear_mission, this);
    service_download_mission = n->advertiseService(
        "Download_mission", &ServiceHandler::download_mission, this);
    service_enable_gps =
        n->advertiseService("Enable_gps", &ServiceHandler::enable_gnss, this);
    service_disable_gps =
        n->advertiseService("Disable_gps", &ServiceHandler::disable_gnss, this);
    service_set_pry = n->advertiseService("Set_pitch_roll_yaw",
                                          &ServiceHandler::set_pry, this);
    service_set_mode =
        n->advertiseService("Set_flight_mode", &ServiceHandler::set_mode, this);
    service_set_yaw =
        n->advertiseService("Set_yaw", &ServiceHandler::set_yaw, this);
    service_get_gps =
        n->advertiseService("Get_gps", &ServiceHandler::get_gps, this);
    service_mission_count = n->advertiseService(
        "Get_mission_count", &ServiceHandler::get_mission_count, this);
  }
  void callback_ack(const coparos::Ack &msg) { ack_ = msg; }
  void callback_save_gps(const sensor_msgs::NavSatFix &msg) {
    saved_gps_ = msg;
  }

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
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    if (ack_.command == CMD_NAV_MOTORS_ON) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_MOTORS_OFF)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    if (ack_.command == uint16_t(CMD_NAV_SET_ABS_HEADING)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
      res.result = false;
      return true;
    }
  }
  bool get_gps(coparos::GPS::Request &req, coparos::GPS::Response &res) {
    res.lat = saved_gps_.latitude;
    res.lon = saved_gps_.longitude;
    return true;
  }
  bool takeoff(coparos::Service_command::Request &req,
               coparos::Service_command::Response &res) {
    log.data = "TAKING OFF";
    log_pub_.publish(log);
    coparos::Command msg;
    msg.command = CMD_NAV_TAKE_OFF;
    msg.data1 = req.param1;
    cmd_pub_.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_TAKE_OFF)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_TO_LAND)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_GOTO_HOME)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_STOP_MOTION)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
      res.result = false;
      return true;
    }
  }

  bool enable_gnss(coparos::Service_command::Request &req,
                   coparos::Service_command::Response &res) {
    coparos::Command msg;
    msg.data1 = 1;
    msg.command = CMD_GNSS_USE;
    cmd_pub_.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_GNSS_USE)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        useFakeGps = true;
        // ROS_INFO("Use fake gps:", useFakeGps);
        // n->setParam("/use_gps_from_video", useFakeGps);
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        // ROS_INFO("Use fake gps:", useFakeGps);
        return true;
      }
    } else {
      res.status = "Controller does not response";
      res.result = false;
      // ROS_INFO("Use fake gps:", useFakeGps);
      return true;
    }
  }
  bool disable_gnss(coparos::Service_command::Request &req,
                    coparos::Service_command::Response &res) {
    coparos::Command msg;
    msg.data1 = 0;
    msg.command = CMD_GNSS_USE;
    cmd_pub_.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_GNSS_USE)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        useFakeGps = false;
        // ROS_INFO("Use fake gps:", useFakeGps);
        // n->setParam("/use_gps_from_video", useFakeGps);
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        // ROS_INFO("Use fake gps:", useFakeGps);
        return true;
      }
    } else {
      res.status = "Controller does not response";
      res.result = false;
      // ROS_INFO("Use fake gps:", useFakeGps);
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_CONTINUE_MOTION)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_EXEC_SWITCH_N)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_CLEAR_WP)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
      res.result = false;
      return true;
    }
  }
  bool download_mission(coparos::Service_command::Request &req,
                        coparos::Service_command::Response &res) {

    log.data = "Geting points count";
    log_pub_.publish(log);
    coparos::Command msg_count;
    msg_count.command = CMD_NAV_GET_WP_COUNT;
    cmd_pub_.publish(msg_count);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    double count;
    if (ack_.command == uint16_t(CMD_NAV_WP_COUNT)) {
      if (ack_.result)
        count = ack_.data;
    }

    log.data = "Downloading mission from drone";
    log_pub_.publish(log);

    coparos::Command msg;
    msg.command = CMD_NAV_LOAD_POINT;
    msg.data1 = count;
    cmd_pub_.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_LOAD_POINT)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
      res.result = false;
      return true;
    }
  }
  bool get_mission_count(coparos::Service_command::Request &req,
                         coparos::Service_command::Response &res) {
    log.data = "Geting points count";
    log_pub_.publish(log);
    coparos::Command msg;
    msg.command = CMD_NAV_GET_WP_COUNT;
    cmd_pub_.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_WP_COUNT)) {
      if (ack_.result) {
        res.status = "Success";
        res.data = ack_.data;
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_SET_MAN_TARGET_ANGLES)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_SET_NAV_MODE)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
      res.result = false;
      return true;
    }
  }
  bool fly_to(coparos::Service_command::Request &req,
              coparos::Service_command::Response &res) {
    log.data = "Fly to point: " + std::to_string(req.param1) + " ; " +
               std::to_string(req.param1);
    log_pub_.publish(log);
    coparos::Command msg;
    msg.command = CMD_NAV_SET_TARGET_ABS;
    msg.data1 = req.param1;
    msg.data2 = req.param2;
    msg.data3 = req.param3;
    msg.data4 = req.param4;
    cmd_pub_.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (ack_.command == uint16_t(CMD_NAV_SET_TARGET_ABS)) {
      if (ack_.result) {
        res.status = "Success";
        res.result = true;
        return true;
      } else {
        res.status = ack_.status;
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller does not response";
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