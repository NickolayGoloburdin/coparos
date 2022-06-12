#include "coparos/Service_command.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandHome.h"
#include "mavros_msgs/CommandInt.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/SetMode.h"
#include "ros/ros.h"
// Класс для работы сервисов передачи команд и миссий в коптер
class ServiceHandler {
public:
  ros::NodeHandle *n;            //Указатель на ноду РОС
  ros::Subscriber ack_processor; // Подписчик на ответы с коптера
  ros::ServiceServer service_arm; //Сервис для снятия коптера с предохранителя
  ros::ServiceServer service_disarm;
  ros::ServiceServer service_set_yaw;
  ros::ServiceServer service_set_speed; //Сервис для выключения двигателей
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
    service_set_yaw =
        n->advertiseService("Set_gps_mode", &ServiceHandler::set_yaw,
                            this); //Сервис для выключения двигателей
    service_set_speed =
        n->advertiseService("Set_speed", &ServiceHandler::set_speed,
                            this); //Сервис для выключения двигателей
  }
  //Функция обработчик ответа с коптера
  bool arm(coparos::Service_command::Request &req,
           coparos::Service_command::Response &res) {

    ROS_INFO("ARMING");
    auto client_guided =
        n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode cmd;
    cmd.request.base_mode = 1;
    cmd.request.custom_mode = "GUIDED";
    if (client_guided.call(cmd)) {
      res.result = cmd.response.mode_sent;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
    auto client_arm =
        n->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool cmd2;
    cmd2.request.value = true;
    if (client_arm.call(cmd2)) {
      res.result = cmd2.response.success;
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
        n->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
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
    auto client_guided =
        n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode cmd1;
    cmd1.request.base_mode = 1;
    cmd1.request.custom_mode = "GUIDED";
    if (client_guided.call(cmd1)) {
      res.result = cmd1.response.mode_sent;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
    ROS_INFO("TAKING OFF");
    auto client_takeoff =
        n->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
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
        n->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
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
        n->serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
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
    auto client_stop =
        n->serviceClient<mavros_msgs::SetMode>("/mavros/cmd/set_mode");
    mavros_msgs::SetMode cmd;
    cmd.request.base_mode = 1;
    cmd.request.custom_mode = "GUIDED";
    if (client_stop.call(cmd)) {
      res.result = cmd.response.mode_sent;
      return true;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }
  bool set_gps_mode(coparos::Service_command::Request &req,
                    coparos::Service_command::Response &res) {

    ROS_INFO("Stopping");
    auto client_stop =
        n->serviceClient<mavros_msgs::SetMode>("/mavros/param/set");
    mavros_msgs::ParamSet cmd;
    if (!useFakeGps) {
      cmd.request.param_id = "GPS_TYPE";
      cmd.request.value.integer = 14;
    } else {
      cmd.request.param_id = "GPS_TYPE";
      cmd.request.value.integer = 1;
    }
    if (client_stop.call(cmd)) {
      useFakeGps = !useFakeGps;
      n->setParam("/use_gps_from_video", useFakeGps);
      res.result = cmd.response.success;
      ROS_INFO("Use fake gps:", useFakeGps);
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
    auto client_continue =
        n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode cmd;
    cmd.request.base_mode = 1;
    cmd.request.custom_mode = "AUTO";
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
    auto client_guided =
        n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode cmd;
    cmd.request.base_mode = 1;
    cmd.request.custom_mode = "GUIDED";
    if (client_guided.call(cmd)) {
      res.result = cmd.response.mode_sent;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
    ROS_INFO("TAKING OFF");
    auto client_takeoff =
        n->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL cmd2;

    cmd2.request.altitude = takeoff_height;
    if (client_takeoff.call(cmd2)) {
      res.result = cmd2.response.success;

    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
    ros::Duration(1).sleep();
    auto client_continue =
        n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode cmd3;
    cmd3.request.base_mode = 1;
    cmd3.request.custom_mode = "AUTO";
    if (client_continue.call(cmd3)) {
      res.result = cmd3.response.mode_sent;
      return true;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }

  //   Param (:Label)	Description	Values	Units
  // 1: Angle	target angle, 0 is north		deg
  // 2: Angular Speed	angular speed		deg/s
  // 3: Direction	direction: -1: counter clockwise, 1: clockwise	min: -1
  // max:1 increment:2
  // 4: Relative	0: absolute angle, 1: relative offset	min:0 max:1
  // increment:1 5	Empty 6	Empty 7	Empty
  bool set_yaw(coparos::Service_command::Request &req,
               coparos::Service_command::Response &res) {
    ROS_INFO("Setting yaw:", req.param1);
    auto client_set_yaw =
        n->serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    mavros_msgs::CommandLong cmd;
    cmd.request.broadcast = true;
    cmd.request.command = uint16_t(115);
    cmd.request.confirmation = 1;
    cmd.request.param1 = req.param1;
    cmd.request.param2 = req.param2;
    cmd.request.param3 = req.param3;
    cmd.request.param4 = req.param4;
    if (client_set_yaw.call(cmd)) {
      res.result = cmd.response.success;
      return true;
    } else {
      res.status = "Cannot call mavros service";
      res.result = false;
      return true;
    }
  }
  bool set_speed(coparos::Service_command::Request &req,
                 coparos::Service_command::Response &res) {
    ROS_INFO("Setting speed:");
    auto client_set_yaw =
        n->serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    mavros_msgs::CommandLong cmd;
    cmd.request.broadcast = true;
    cmd.request.command = uint16_t(115);
    cmd.request.confirmation = 1;
    cmd.request.param1 = req.param1;
    cmd.request.param2 = req.param2;
    cmd.request.param3 = req.param3;
    cmd.request.param4 = req.param4;
    if (client_set_yaw.call(cmd)) {
      res.result = cmd.response.success;
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
