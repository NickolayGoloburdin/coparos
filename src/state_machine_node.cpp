/*
 * telemetry_node.cpp
 *
 *  Created on: 5 августа 2022 г.
 *      Author: Nickolay
 */

#include <actionlib/client/simple_action_client.h> // action Library Header File
#include <actionlib/client/terminal_state.h> // Action Goal Status Header File
#include <coparos/AzimuthFlyAction.h>
#include <coparos/Download_mission.h>
#include <coparos/DroneInfo.h>
#include <coparos/GPS.h>
#include <coparos/MissionPoint.h>
#include <coparos/Service_command.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// void write_callback(const std_msgs::String::ConstPtr &msg) {
//   //   ROS_INFO_STREAM("Writing to serial port" << msg->data);
//   //   ser.write(msg->data);
// }
class StateMachine {
private:
  ros::NodeHandle *nh_;
  bool gnss_status = true;
  int channel11_;
  unsigned int drone_mode_, drone_prev_mode_, current_wp_;
  // ros::Subscriber gnss_use_status_sub_;
  ros::Subscriber rc_channel_sub_;
  ros::ServiceClient flight_mode_service_client;
  ros::ServiceClient get_gps_service_client;
  ros::ServiceClient req_dwnld_mission_client;
  ros::ServiceClient missions_service_client;
  ros::ServiceClient measure_wind_service_client;
  std::vector<coparos::MissionPoint> mission_;
  // actionlib::SimpleActionClient<coparos::AzimuthFlyAction> ac;

  void callback_drone_info(const coparos::DroneInfo &msg) {
    channel11_ = msg.rc11_channel;
    drone_mode_ = msg.DRONE_MODE;
    current_wp_ = msg.current_wp;
    gnss_status = msg.rc6_channel > 200 ? false : true;
  }
  // void callback_gnss(const std_msgs::Bool &msg) { gnss_status = msg.data; }

public:
  void mes_wind() {
    if (channel11_ > 900) {
      coparos::Service_command cmd;
      measure_wind_service_client.call(cmd);
    }
  }
  unsigned int create_target_flight_mode() {
    if (gnss_status == false)
      return 1;
    else
      return 4;
  }
  unsigned int current_mode() { return drone_mode_; }

  StateMachine(ros::NodeHandle *nh) : nh_(nh) {
    // gnss_use_status_sub_ = nh_->subscribe("/gnss_use_status", 1,
    //                                       &StateMachine::callback_gnss,
    //                                       this);
    rc_channel_sub_ = nh_->subscribe("/droneInfo", 1,
                                     &StateMachine::callback_drone_info, this);
    flight_mode_service_client =
        nh_->serviceClient<coparos::Service_command>("Set_flight_mode");
    get_gps_service_client = nh_->serviceClient<coparos::GPS>("Get_gps");
    req_dwnld_mission_client =
        nh_->serviceClient<coparos::Service_command>("Download_mission");
    missions_service_client =
        nh_->serviceClient<coparos::Download_mission>("GetMissionPointsList");
    measure_wind_service_client =
        nh_->serviceClient<coparos::Service_command>("MeasureWind");
  }
  void set_target_mode() {
    coparos::Service_command cmd;
    unsigned int target = create_target_flight_mode();
    if (target == 4 && target != current_mode()) {
      cmd.request.param1 = 4;
      flight_mode_service_client.call(cmd);
    } else if (target == 1 && target != current_mode()) {
      cmd.request.param1 = 1;
      flight_mode_service_client.call(cmd);
      actionlib::SimpleActionClient<coparos::AzimuthFlyAction> ac("azimuth",
                                                                  true);
      ac.waitForServer();
      coparos::AzimuthFlyGoal goal;
      if (current_wp_ + 1 <= mission_.size()) {
        goal.target.targetLat = mission_[current_wp_ + 1].targetLat;
        goal.target.targetLat = mission_[current_wp_ + 1].targetLon;
      } else {
        goal.target.targetLat = mission_[current_wp_ + 1].targetLat;
        goal.target.targetLat = mission_[current_wp_ + 1].targetLon;
      }
      ac.sendGoal(goal);
      bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
      ros::Timer timer = nh_->createTimer(
          ros::Duration(30),
          [&](const ros::TimerEvent &event) { ac.cancelGoal(); });

      // Process when action results are received within the time limit for
      // achieving the action goal
      if (finished_before_timeout) {
        // Receive action target status value and display on screen
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
      }
    }
  }
  void check_mission() {
    if (current_mode() == 4 && current_mode() != drone_prev_mode_) {
      coparos::Service_command srv;
      req_dwnld_mission_client.call(srv);
      coparos::Download_mission srvmission;
      missions_service_client.call(srvmission);
      for (auto i : srvmission.response.points)
        mission_.push_back(i);
    } else {
      drone_prev_mode_ = current_mode();
    }
  }
};
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;
  StateMachine smach(&nh);
  ros::Rate loop_rate(20);
  while (ros::ok()) {
    smach.mes_wind();
    smach.check_mission();

    ros::spinOnce();

    loop_rate.sleep();
  }
}