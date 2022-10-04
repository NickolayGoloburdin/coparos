#include "coparos/Service_command.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <copa_msgs/WindSpeedAction.h>
#include <copa_msgs/WindSpeedFeedback.h>
#include <copa_msgs/WindSpeedGoal.h>
#include <copa_msgs/WindSpeedResult.h>
#include <coparos/DroneInfo.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <string>
#define PI 3.14159265358979323846
std::string rounded(double a) {
  std::string num_text = std::to_string(a);
  return num_text.substr(0, num_text.find(".") + 2);
}
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / PI)
#define degToRad(angleInDegrees) ((angleInDegrees)*PI / 180.0)
const double koeff_speed_angle = 1.4;
class Service {
public:
  ros::NodeHandle *n;
  ros::ServiceClient client_stop;
  ros::ServiceClient client_start;
  ros::Publisher log_pub_;
  ros::Subscriber compass_sub_;
  ros::Publisher angles_pub_;
  ros::ServiceClient client_yaw;
  double heading_;
  std_msgs::String log;
  ros::ServiceServer service_measure_wind;
  actionlib::SimpleActionClient<copa_msgs::WindSpeedAction> ac;
  Service(ros::NodeHandle *nh) : n(nh), ac("/mes_wind", true) {
    log_pub_ = n->advertise<std_msgs::String>("/logging_topic", 1000);
    client_stop = n->serviceClient<coparos::Service_command>("Set_flight_mode");
    client_start =
        n->serviceClient<coparos::Service_command>("Set_flight_mode");
    client_yaw = n->serviceClient<coparos::Service_command>("Set_yaw");
    compass_sub_ =
        n->subscribe("/droneInfo", 1, &Service::callback_heading, this);
    service_measure_wind =
        n->advertiseService("/MeasureWind", &Service::measure_wind, this);
    ac.waitForServer();
    angles_pub_ = n->advertise<geometry_msgs::Vector3>("/manualAngles", 1000);
  }
  void logging(std::string log_str) {
    log.data = log_str;
    log_pub_.publish(log);
  }
  bool measure_wind(coparos::Service_command::Request &req,
                    coparos::Service_command::Response &res) {

    coparos::Service_command cmd;
    cmd.request.param1 = 1;
    if (client_stop.call(cmd)) {
      if (cmd.response.result) {
        logging("Drone is stopped, start measuring wind");

      } else {
        logging("Cannot stop drone");
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
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout) {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished,starting drone");
      logging("Action finished,starting drone");
    } else {
      res.status = "Action Server doesnt responce";
      logging("Action Server doesnt responce");
      res.result = false;
      return true;
    }
    auto action_result = ac.getResult();
    double speed = action_result->speed;
    double angle = action_result->angle;
    n->setParam("/wind_speed", speed);
    n->setParam("/wind_angle", angle);
    logging("wind speed =" + rounded(speed) +
            " wind course = " + rounded(angle));
    logging("Setting stop mode for 5 sec");

    geometry_msgs::Vector3 angles;
    cmd.request.param1 = angle < 0.0 ? angle + 180.0 : angle - 180.0;
    cmd.request.param2 = 60;
    if (client_yaw.call(cmd)) {
      logging("Setting course " + rounded(cmd.request.param1));
    }
    // ros::Duration(4).sleep();
    angles.x = -speed * koeff_speed_angle;
    logging("Setting pitch = " + rounded(angles.x) +
            ", roll = " + rounded(angles.y));

    logging("Stopping drone for 5 seconds");
    angles_pub_.publish(angles);
    ros::Duration(5).sleep();
    ros::spinOnce();
    cmd.request.param1 = 4;
    if (client_start.call(cmd)) {
      if (cmd.response.result) {

      } else {
        logging("Cannot start drone");
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
    heading_ = double(msg.ABSOLUTE_HEADING);
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