#include "coparos/Service_command.h"
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <copa_msgs/WindSpeedAction.h>
#include <copa_msgs/WindSpeedFeedback.h>
#include <copa_msgs/WindSpeedGoal.h>
#include <copa_msgs/WindSpeedResult.h>
ros::NodeHandle n;
bool measure_wind(coparos::Service_command::Request &req,
                  coparos::Service_command::Response &res) {
  ros::ServiceClient client =
      n->serviceClient<coparos::Service_command>("STOP");
  coparos::Service_command cmd;
  if (client.call(cmd)) {
    if (cmd.response.result)
      ROS_INFO("Drone is stopped, start measuring wind");
    else {
      ROS_INFO("Cannot stop drone");
      res.status = "Drone doesn't responce on command";
      res.result = false;
      return true;
    }
  } else {
    res.status = "Failed to call service stop drone";
    res.result = false;
    return true;
  }
  actionlib::SimpleActionClient<coparos_msgs::WindSpeedAction> ac("mes_wind",
                                                                  true);
  coparos_msgs::WindSpeedGoal goal;
  goal.order = true;
  ac.sendGoal(goal);
  bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));
  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished,starting drone");
  } else {
    res.status = "Action Server doesnt responce";
    res.result = false;
    return true;
  }
  ros::ServiceClient client =
      n->serviceClient<coparos::Service_command>("Continue");
  coparos::Service_command cmd;
  if (client.call(cmd)) {
    if (cmd.response.result)
      ROS_INFO("Wind is measured Continue fly");
    else {
      ROS_INFO("Cannot start drone");
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
int main(int argc, char **argv) {
  ros::init(argc, argv, "measure_wind_service");
  n = new ros::NodeHandle;

  ros::ServiceServer service_measure_wind =
      n->advertiseService("MeasureWind", measure_wind);

  ROS_INFO("Ready to measure wind.");
  ros::spin();

  return 0;
}