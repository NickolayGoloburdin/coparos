#include "coparos/Service_command.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <copa_msgs/WindSpeedAction.h>
#include <copa_msgs/WindSpeedFeedback.h>
#include <copa_msgs/WindSpeedGoal.h>
#include <copa_msgs/WindSpeedResult.h>
class Service {
public:
  ros::NodeHandle *n;
  ros::ServiceClient client_stop;
  ros::ServiceClient client_start;
  actionlib::SimpleActionClient<copa_msgs::WindSpeedAction> ac;
  Service(ros::NodeHandle *nh) : n(nh), ac("mes_wind", true) {
    client_stop = n->serviceClient<coparos::Service_command>("STOP");
    client_start = n->serviceClient<coparos::Service_command>("Continue");
    ac.waitForServer();
    ros::ServiceServer service_measure_wind =
        n->advertiseService("MeasureWind", &Service::measure_wind, this);
  }

private:
  bool measure_wind(coparos::Service_command::Request &req,
                    coparos::Service_command::Response &res) {

    coparos::Service_command cmd;
    if (client_stop.call(cmd)) {
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
    copa_msgs::WindSpeedGoal goal;
    goal.start = true;
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
    auto action_result = ac.getResult();
    float speed = action_result->speed;
    float angle = action_result->angle;
    n->setParam("/wind_speed", speed);
    n->setParam("/wind_angle", angle);

    n->serviceClient<coparos::Service_command>("Continue");
    if (client_start.call(cmd)) {
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
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "measure_wind_service");
  ros::NodeHandle n;
  Service service(&n);

  ROS_INFO("Ready to measure wind.");
  ros::spin();

  return 0;
}