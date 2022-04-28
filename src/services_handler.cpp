#include "coparos/Service_command.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandHome.h"
#include "mavros_msgs/CommandInt.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "ros/ros.h"
// Класс для работы сервисов передачи команд и миссий в коптер
class ServiceHandler {
public:
  ros::NodeHandle *n;            //Указатель на ноду РОС
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
  ServiceHandler(ros::NodeHandle *nh) : n(nh) {
    //Инициализация сервисо и подписчиков РОС
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
  }
  //Функция обработчик ответа с коптера
  bool arm(coparos::Service_command::Request &req,
           coparos::Service_command::Response &res) {

    ROS_INFO("ARMING");
    auto client_arm = n->serviceClient<mavros_msgs::CommandBool>("~cmd/arming");
    mavros_msgs::CommandBool cmd;
    cmd.request.value = true;
    if (client_arm.call(cmd)) {
      res.result = cmd.response.success;
      return true;

    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }

    //   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  }
  bool disarm(coparos::Service_command::Request &req,
              coparos::Service_command::Response &res) {
    ROS_INFO("DISARMING");
    auto client_disarm =
        n->serviceClient<mavros_msgs::CommandBool>("~cmd/arming");
    mavros_msgs::CommandBool cmd;
    cmd.request.value = false;
    if (client_disarm.call(cmd)) {
      res.result = cmd.response.success;
      return true;

    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }
  bool takeoff(coparos::Service_command::Request &req,
               coparos::Service_command::Response &res) {
    float takeoff_height;
    if (!(n->getParam("/takeoff_height", takeoff_height))) {
      res.status = "Cannot read takeoff height from rosparam";
      res.result = false;
      return true;
    }
    ROS_INFO("TAKING OFF");
    auto client_takeoff =
        n->serviceClient<mavros_msgs::CommandTOL>("~cmd/takeoff");
    mavros_msgs::CommandTOL cmd;

    cmd.request.altitude = takeoff_height;
    if (client_takeoff.call(cmd)) {
      res.result = cmd.response.success;
      return true;

    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }
  bool land(coparos::Service_command::Request &req,
            coparos::Service_command::Response &res) {
    ROS_INFO("LANDING");
    auto client_landing =
        n->serviceClient<mavros_msgs::CommandTOL>("~cmd/land");
    mavros_msgs::CommandTOL cmd;
    if (client_landing.call(cmd)) {
      res.result = cmd.response.success;
      return true;

    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }

  bool rtl(coparos::Service_command::Request &req,
           coparos::Service_command::Response &res) {
    ROS_INFO("Returning to launch");
    auto client_rtl =
        n->serviceClient<mavros_msgs::CommandLong>("~cmd/command");
    mavros_msgs::CommandLong cmd;
    cmd.request.broadcast = true;
    cmd.request.command = uint16_t(20);
    cmd.request.confirmation = 1;
    if (client_rtl.call(cmd)) {
      res.result = cmd.response.success;
      return true;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }
  bool stop(coparos::Service_command::Request &req,
            coparos::Service_command::Response &res) {
    ROS_INFO("Stopping");
    auto client_stop = n->serviceClient<mavros_msgs::SetMode>("~/set_mode");
    mavros_msgs::SetMode cmd;
    cmd.request.base_mode = 1;
    cmd.request.custom_mode = 4;
    if (client_stop.call(cmd)) {
      res.result = cmd.response.mode_sent;
      return true;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }
  bool continue_flight(coparos::Service_command::Request &req,
                       coparos::Service_command::Response &res) {
    ROS_INFO("Continue");
    auto client_continue = n->serviceClient<mavros_msgs::SetMode>("~/set_mode");
    mavros_msgs::SetMode cmd;
    cmd.request.base_mode = 1;
    cmd.request.custom_mode = 3;
    if (client_continue.call(cmd)) {
      res.result = cmd.response.mode_sent;
      return true;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }
  bool start_mission(coparos::Service_command::Request &req,
                     coparos::Service_command::Response &res) {
    ROS_INFO("Starting 1 click mission");

    float takeoff_height;
    if (!(n->getParam("/takeoff_height", takeoff_height))) {
      res.status = "Cannot read takeoff height from rosparam";
      res.result = false;
      return true;
    }
    ROS_INFO("TAKING OFF");
    auto client_takeoff =
        n->serviceClient<mavros_msgs::CommandTOL>("~cmd/takeoff");
    mavros_msgs::CommandTOL cmd;

    cmd.request.altitude = takeoff_height;
    if (client_takeoff.call(cmd)) {
      res.result = cmd.response.success;

    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
    auto client_continue = n->serviceClient<mavros_msgs::SetMode>("~/set_mode");
    mavros_msgs::SetMode cmd2;
    cmd2.request.base_mode = 1;
    cmd2.request.custom_mode = 3;
    if (client_continue.call(cmd2)) {
      res.result = cmd2.response.mode_sent;
      return true;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }
  bool clear_mission(coparos::Service_command::Request &req,
                     coparos::Service_command::Response &res) {
    ROS_INFO("Cleaning mission");
    return true;
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "services_server");
  ros::NodeHandle n;
  ServiceHandler sh(&n);
  ros::spin();

  return 0;
}