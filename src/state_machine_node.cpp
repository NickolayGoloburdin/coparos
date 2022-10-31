/*
 * telemetry_node.cpp
 *
 *  Created on: 5 августа 2022 г.
 *      Author: Nickolay
 */
// Конечный автомат системы для включения азимутального полета и измерения ветра
#include "copa_types.h"
#include <actionlib/client/simple_action_client.h> // action Library Header File
#include <actionlib/client/terminal_state.h> // Action Goal Status Header File
#include <coparos/AzimuthFlyAction.h>
#include <coparos/Download_mission.h>
#include <coparos/DroneInfo.h>
#include <coparos/GPS.h>
#include <coparos/MissionPoint.h>
#include <coparos/Service_command.h>
#include <cstddef>
#include <cstdint>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <string>

// void write_callback(const std_msgs::String::ConstPtr &msg) {
//   //   ROS_INFO_STREAM("Writing to serial port" << msg->data);
//   //   ser.write(msg->data);
// }
class StateMachine {
private:
  ros::NodeHandle *nh_;
  bool gnss_status = true;
  int channel11_, safety = 0;
  unsigned int drone_mode_, drone_prev_mode_, current_wp_;
  bool wind_is_measured = false;
  uint16_t state_ = 0;
  double baro_ = 0;
  bool mission_downloaded_ = false;
  int safe_misson_ = 0;
  // ros::Subscriber gnss_use_status_sub_;
  ros::Subscriber rc_channel_sub_;
  ros::Subscriber baro_sub_;
  ros::ServiceClient flight_mode_service_client;
  ros::ServiceClient get_gps_service_client;
  ros::ServiceClient gnss_on;
  ros::ServiceClient req_dwnld_mission_client;
  ros::ServiceClient missions_service_client;
  ros::ServiceClient exec_point_service_client;
  ros::ServiceClient measure_wind_service_client;
  std::vector<coparos::MissionPoint> mission_;
  ros::Publisher log_pub_;
  std_msgs::String log;
  bool azimuth_fly = false;
  bool posadka = false;
  // actionlib::SimpleActionClient<coparos::AzimuthFlyAction> ac;
  //Считывание данных с радиоканала для обработки команды измерения ветра и
  // gnss_use с пульта
  void callback_drone_info(const coparos::DroneInfo &msg) {
    channel11_ = msg.rc11_channel;
    drone_mode_ = msg.DRONE_MODE;
    ROS_INFO("Drone mode %d", drone_mode_);
    if (gnss_status == true) {
      current_wp_ = msg.current_wp;
    }
    ROS_INFO("current wp: %d", current_wp_);
    gnss_status = msg.rc6_channel > 200 ? true : false;
    state_ = msg.STATE;
  }
  // void callback_gnss(const std_msgs::Bool &msg) { gnss_status = msg.data; }

public:
  //вызывается модуль измерения ветра когда тумблер на 11 канале джойстика
  //выдает пвм больше 900
  void mes_wind() {
    if (channel11_ > 900) {
      log.data = "Channel 11 = 1000; Starting measuring wind";
      log_pub_.publish(log);
      coparos::Service_command cmd;
      measure_wind_service_client.call(cmd);
      wind_is_measured = true;
      log.data = "Service " + std::to_string(cmd.response.result) +
                 cmd.response.status;
      log_pub_.publish(log);
    }
  }
  //Выбор режима полета: 1 азимутальный; 0 никакой; 4 автоматический
  unsigned int create_target_flight_mode() {
    if (gnss_status == true)
      return 4;
    else if (!wind_is_measured)
      return 0;
    else
      return 1;
  }
  //Запрос текущего режима в дроне
  unsigned int current_mode() { return drone_mode_; }

  StateMachine(ros::NodeHandle *nh) : nh_(nh) {
    // gnss_use_status_sub_ = nh_->subscribe("/gnss_use_status", 1,
    //                                       &StateMachine::callback_gnss,
    //                                       this);
    rc_channel_sub_ = nh_->subscribe("/droneInfo", 1,
                                     &StateMachine::callback_drone_info, this);
    baro_sub_ = nh_->subscribe("/baro_relative", 1,
                               &StateMachine::callback_baro_, this);
    flight_mode_service_client =
        nh_->serviceClient<coparos::Service_command>("/Set_flight_mode");
    get_gps_service_client = nh_->serviceClient<coparos::GPS>("/Get_gps");
    req_dwnld_mission_client =
        nh_->serviceClient<coparos::Service_command>("/Download_mission");
    exec_point_service_client =
        nh_->serviceClient<coparos::Service_command>("/Exec_Point");
    missions_service_client =
        nh_->serviceClient<coparos::Download_mission>("/GetMissionPointsList");
    measure_wind_service_client =
        nh_->serviceClient<coparos::Service_command>("/MeasureWind");
    gnss_on = nh_->serviceClient<coparos::Service_command>("/Enable_gps");
    log_pub_ = nh_->advertise<std_msgs::String>("/logging_topic", 1000);
  }

  void callback_baro_(const std_msgs::Float64 &msg) { baro_ = msg.data; }
  void logging(std::string log_str) {
    log.data = log_str;
    log_pub_.publish(log);
  }
  void set_target_mode( //Выставление необходимого режима полета
      actionlib::SimpleActionClient<coparos::AzimuthFlyAction> &ac) {
    if (baro_ < 10 || current_mode() == 3)
      return;

    unsigned int target = create_target_flight_mode();
    // Проверка происходт ли в данный момент азимутальный полет
    if (ac.getState() == actionlib::SimpleClientGoalState::PENDING) {
      azimuth_fly = true;

    } else if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
      azimuth_fly = true;

    } else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      azimuth_fly = false;

    } else {
      azimuth_fly = false;
    }

    coparos::Service_command cmd;
    //Если целевой режим автоматический, выставляется автоматический и если дрон
    //в азимутальном режиме, поисходит выход из азимутального режима
    if (target == 4 && target != current_mode() && safe_misson_ < 10) {
      posadka = false;
      safe_misson_++;
      if (azimuth_fly) {
        ac.cancelGoal();
        logging("Cancelling from action");
        azimuth_fly = false;
      }

      cmd.request.param1 = 4;
      flight_mode_service_client.call(cmd);
      logging("Current mode " + std::to_string(current_mode()));
      logging("Set mission mode");
    } else if (target == 1 && !azimuth_fly && !posadka) {
      //Если целевой полет азимутальный, то из полетного задания получаются
      //целевые точки  и отправляются в модуль азимутального полета
      safe_misson_ = 0;
      logging("Starting action azimuth client ");
      coparos::AzimuthFlyGoal goal;
      if (mission_.size() == 0) {
        logging("Mission is empty Set althold mode");
        cmd.request.param1 = 1;
        flight_mode_service_client.call(cmd);
        return;
      }
      if (current_wp_ < mission_.size()) {
        logging("c_wp size accepted, mission size " +
                std::to_string(mission_.size()));
        logging("fly to point num " + std::to_string(current_wp_));
        goal.target.targetLat = mission_[current_wp_].targetLat;
        goal.target.targetLon = mission_[current_wp_].targetLon;
        ROS_INFO("Target lat %f", goal.target.targetLat);
        ROS_INFO("Target lon %f", goal.target.targetLon);
        goal.target.maxHorizSpeed = mission_[current_wp_].maxHorizSpeed;
        if (goal.target.targetLat < 5.0 || goal.target.targetLon < 5.0) {
          logging("target coordinates empty");
          gnss_on.call(cmd);
          logging("Finishing mission");
          cmd.request.param1 = current_wp_;
          exec_point_service_client.call(cmd);
          cmd.request.param1 = 4;
          flight_mode_service_client.call(cmd);
          posadka = true;
          return;
        }

        cmd.request.param1 = current_wp_;
        exec_point_service_client.call(cmd);
        //Перед вызовом азимутального полета включается режим удержания высоты
        //для фиксации дрона
        cmd.request.param1 = 1;
        flight_mode_service_client.call(cmd);
        current_wp_++;
        logging("Set althold mode");
        logging("start azimuth fly");
        logging("sending goal");

        ac.sendGoal(goal);

        logging("waiting result");

      } else {
        logging("c_wp size declined, mission size" +
                std::to_string(mission_.size()));
        return;
      }

      // ros::Timer timer = nh_->createTimer(
      //     ros::Duration(40),
      //     [&](const ros::TimerEvent &event) { ac.cancelGoal(); });
      // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

      // log.data = "result accepted";
      // log_pub_.publish(log);
      // Process when action results are received within the time limit for
      // achieving the action goal

      // if (finished_before_timeout) {
      //   // Receive action target status value and display on screen
      //   actionlib::SimpleClientGoalState state = ac.getState();
      //   log.data = "Action finished";
      //   log_pub_.publish(log);
      //   ROS_INFO("Action finished: %s", state.toString().c_str());
      // }
    }
  }
  //При включениии автоматического режима в первый раз с дрона скачивается
  //текущее полетное задания для азимутального полета

  void check_mission() {
    if (mission_downloaded_)
      return;
    if (current_mode() == 4) {
      logging("Download mission from drone");
      coparos::Service_command srv;
      req_dwnld_mission_client.call(srv);
      coparos::Download_mission srvmission;
      missions_service_client.call(srvmission);

      logging("Downloaded " +
              std::to_string(srvmission.response.points.size()) + " points");
      if (srvmission.response.points.size() > 1)
        mission_downloaded_ = true;
      for (auto i : srvmission.response.points)
        mission_.push_back(i);
      for (size_t i = 0; i < mission_.size(); i++) {
        logging("Point " + std::to_string(i) +
                "lat = " + std::to_string(mission_[i].targetLat) +
                " lon = " + std::to_string(mission_[i].targetLon));
      }
    }
  }
};
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;
  StateMachine smach(&nh);
  actionlib::SimpleActionClient<coparos::AzimuthFlyAction> ac("azimuth", true);
  ac.waitForServer();
  ros::Rate loop_rate(5);
  while (ros::ok()) {
    smach.mes_wind();
    smach.check_mission();
    smach.set_target_mode(ac);

    ros::spinOnce();

    loop_rate.sleep();
  }
}