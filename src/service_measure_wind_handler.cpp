#include "coparos/Service_command.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <copa_msgs/WindSpeedAction.h>
#include <copa_msgs/WindSpeedFeedback.h>
#include <copa_msgs/WindSpeedGoal.h>
#include <copa_msgs/WindSpeedResult.h>
#include <coparos/DroneInfo.h>
#include <std_msgs/String.h>
#include <string>
#define PI 3.14159265358979323846
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / PI)
class Service {
public:
  ros::NodeHandle *n;
  ros::ServiceClient client_stop;
  ros::ServiceClient client_start;
  ros::Publisher log_pub_;
  ros::Subscriber compass_sub_;
  float heading_;
  std_msgs::String log;
  ros::ServiceServer service_measure_wind;
  actionlib::SimpleActionClient<copa_msgs::WindSpeedAction> ac;
  Service(ros::NodeHandle *nh) : n(nh), ac("/mes_wind", true) {
    log_pub_ = n->advertise<std_msgs::String>("/logging_topic", 1000);
    client_stop = n->serviceClient<coparos::Service_command>("Set_flight_mode");
    client_start =
        n->serviceClient<coparos::Service_command>("Set_flight_mode");
    compass_sub_ =
        n->subscribe("/droneInfo", 1, &Service::callback_heading, this);
    service_measure_wind =
        n->advertiseService("/MeasureWind", &Service::measure_wind, this);
    ac.waitForServer();
  }
  bool measure_wind(coparos::Service_command::Request &req,
                    coparos::Service_command::Response &res) {

    coparos::Service_command cmd;
    cmd.request.param1 = 1;
    if (client_stop.call(cmd)) {
      if (cmd.response.result) {
        log.data = "Drone is stopped, start measuring wind";
        log_pub_.publish(log);
      } else {
        log.data = "Cannot stop drone";
        log_pub_.publish(log);
        res.status = "Drone doesn't responce on command";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Failed to call service stop drone";
      res.result = false;
      return true;
    }
    copa_msgs::WindSpeedGoal goal;
    goal.start = true;
    // ac.sendGoal(goal);
    bool finished_before_timeout =
        true; // ac.waitForResult(ros::Duration(20.0));
    if (finished_before_timeout) {
      // actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished,starting drone");
    } else {
      res.status = "Action Server doesnt responce";
      res.result = false;
      return false;
    }
    auto action_result = 0; // ac.getResult();
    float speed = 10;
    // action_result->speed;
    float angle = 90.0 - radToDeg(1) + heading_; // action_result->angle
    n->setParam("/wind_speed", speed);
    n->setParam("/wind_angle", angle);
    cmd.request.param1 = 4;
    if (client_start.call(cmd)) {
      if (cmd.response.result) {
        log.data = "Wind is measured, wind speed =" + std::to_string(speed) +
                   " wind course = " + std::to_string(angle) + "Continue fly";
        log_pub_.publish(log);
      } else {
        log.data = "Cannot start drone";
        log_pub_.publish(log);
        res.status = "Drone doesn't responce on command";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Failed to call service continue";
      res.result = false;
      return true;
    }
    res.status = "wind has been measured";
    res.result = true;
    return true;
  }
  void callback_heading(const coparos::DroneInfo &msg) {
    heading_ = msg.ABSOLUTE_HEADING;
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "measure_wind_service");
  ros::NodeHandle n;
  Service service(&n);
  ROS_INFO("Ready to measure wind.");
  ros::spin();

  return 0;
}