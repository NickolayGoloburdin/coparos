/*
 * telemetry_node.cpp
 *
 *  Created on: 5 августа 2022 г.
 *      Author: Nickolay
 */

#include "copa_types.h"
#include <actionlib/client/simple_action_client.h> // action Library Header File
#include <actionlib/client/terminal_state.h> // Action Goal Status Header File
#include <coparos/AzimuthFlyAction.h>
#include <coparos/Download_mission.h>
#include <coparos/DroneInfo.h>
#include <coparos/GPS.h>
#include <coparos/MissionPoint.h>
#include <coparos/Service_command.h>
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
  // ros::Subscriber gnss_use_status_sub_;
  ros::Subscriber rc_channel_sub_;
  ros::Subscriber baro_sub_;
  ros::ServiceClient flight_mode_service_client;
  ros::ServiceClient get_gps_service_client;
  ros::ServiceClient req_dwnld_mission_client;
  ros::ServiceClient missions_service_client;
  ros::ServiceClient measure_wind_service_client;
  std::vector<coparos::MissionPoint> mission_;
  ros::Publisher log_pub_;
  std_msgs::String log;
  bool azimuth_fly = false;
  // actionlib::SimpleActionClient<coparos::AzimuthFlyAction> ac;

  void callback_drone_info(const coparos::DroneInfo &msg) {
    channel11_ = msg.rc11_channel;
    drone_mode_ = msg.DRONE_MODE;
    current_wp_ = msg.current_wp;
    gnss_status = msg.rc6_channel > 200 ? true : false;
    state_ = msg.STATE;
  }
  // void callback_gnss(const std_msgs::Bool &msg) { gnss_status = msg.data; }

public:
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
  unsigned int create_target_flight_mode() {
    if (gnss_status == true)
      return 4;
    else if (!wind_is_measured)
      return 0;
    else
      return 1;
  }
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
    missions_service_client =
        nh_->serviceClient<coparos::Download_mission>("/GetMissionPointsList");
    measure_wind_service_client =
        nh_->serviceClient<coparos::Service_command>("/MeasureWind");
    log_pub_ = nh_->advertise<std_msgs::String>("/logging_topic", 1000);
  }

  void callback_baro_(const std_msgs::Float64 &msg) { baro_ = msg.data; }
  void set_target_mode() {
    if (baro_ < 10 || current_mode() == 3)
      return;
    actionlib::SimpleActionClient<coparos::AzimuthFlyAction> ac("azimuth",
                                                                true);
    ac.waitForServer();
    // log.data = "set_target_mode check acccepted";
    // log_pub_.publish(log);
    unsigned int target = create_target_flight_mode();
    // log.data = "target_mode = " + std::to_string(target) +
    //            "current mode = " + std::to_string(current_mode());
    // log_pub_.publish(log);
    if (ac.getState() == actionlib::SimpleClientGoalState::PENDING) {
      azimuth_fly = true;
    } else if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
      azimuth_fly = true;
    } else {
      azimuth_fly = false;
    }

    coparos::Service_command cmd;
    if (target == 4 && target != current_mode()) {
      if (azimuth_fly) {
        ac.cancelGoal();
        log.data = "Cancelling from action";
        log_pub_.publish(log);
        azimuth_fly = false;
      }

      cmd.request.param1 = 4;
      flight_mode_service_client.call(cmd);
      log.data = "Set mission mode";
      log_pub_.publish(log);
    } else if (target == 1 && target != current_mode() && !azimuth_fly) {
      cmd.request.param1 = 1;
      flight_mode_service_client.call(cmd);
      log.data = "Set althold mode";
      log_pub_.publish(log);

      log.data = "Starting action azimuth client ";
      log_pub_.publish(log);

      log.data = "start azimuth fly";
      log_pub_.publish(log);
      coparos::AzimuthFlyGoal goal;
      if (mission_.size() == 0) {
        log.data = "Mission is empty";
        log_pub_.publish(log);
        return;
      }
      if (current_wp_ + 1 <= mission_.size()) {
        log.data = "c_wp size accepted, mission size" +
                   std::to_string(mission_.size());
        log_pub_.publish(log);
        goal.target.targetLat = mission_[current_wp_ + 1].targetLat;
        goal.target.targetLat = mission_[current_wp_ + 1].targetLon;
      } else {
        log.data = "c_wp size declined, mission size" +
                   std::to_string(mission_.size());
        log_pub_.publish(log);
        goal.target.targetLat = mission_[current_wp_ + 1].targetLat;
        goal.target.targetLon = mission_[current_wp_ + 1].targetLon;
      }
      log.data = "sending goal";
      log_pub_.publish(log);
      ac.sendGoal(goal);
      log.data = "waiting result";
      log_pub_.publish(log);
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
  void check_mission() {
    if (mission_downloaded_)
      return;
    if (current_mode() == 4) {
      log.data = "Download mission from drone";
      log_pub_.publish(log);
      coparos::Service_command srv;
      req_dwnld_mission_client.call(srv);
      coparos::Download_mission srvmission;
      missions_service_client.call(srvmission);

      log.data = "Downloaded " +
                 std::to_string(srvmission.response.points.size()) + "points";
      log_pub_.publish(log);
      if (srvmission.response.points.size() > 1)
        mission_downloaded_ = true;
      for (auto i : srvmission.response.points)
        mission_.push_back(i);
    }
  }
};
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;
  StateMachine smach(&nh);
  ros::Rate loop_rate(5);
  while (ros::ok()) {
    smach.mes_wind();
    smach.check_mission();
    smach.set_target_mode();

    ros::spinOnce();

    loop_rate.sleep();
  }
}