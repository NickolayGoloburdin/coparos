#include "coparos/Azimuth.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "ros/ros.h"
#include <math.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define PI 3.14159265358979323846
#define RADIO_TERRESTRE 6372797.56085
#define GRADOS_RADIANES PI / 180
#define RADIANES_GRADOS 180 / PI
#define degToRad(angleInDegrees) ((angleInDegrees)*PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / PI)
const double koeff_speed_angle = 1.0;
const float a = 6378137.0;
const float b = 6371000.0;

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double longitude1 = lon1;
  double longitude2 = lon2;
  double latitude1 = lat1 * GRADOS_RADIANES;
  double latitude2 = lat2 * GRADOS_RADIANES;
  double longDiff = (longitude2 - longitude1) * GRADOS_RADIANES;
  double y = sin(longDiff) * cos(latitude2);
  double x = cos(latitude1) * sin(latitude2) -
             sin(latitude1) * cos(latitude2) * cos(longDiff);

  return fmod(((RADIANES_GRADOS * (atan2(y, x))) + 360), 360);
}

double distanceEarth(double lat1, double lon1, double lat2, double lon2) {
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = degToRad(lat1);
  lon1r = degToRad(lon1);
  lat2r = degToRad(lat2);
  lon2r = degToRad(lon2);
  u = std::sin((lat2r - lat1r) / 2);
  v = std::sin((lon2r - lon1r) / 2);
  return 2.0 * b *
         std::asin(
             std::sqrt(u * u + std::cos(lat1r) * std::cos(lat2r) * v * v));
}

class AzimuthFly {
public:
  ros::NodeHandle *n;
  ros::Subscriber heading_sub_;
  ros::Subscriber gps_sub_;
  ros::Publisher control_pub_;
  ros::ServiceServer service_azimuth;
  double current_lat_;
  double current_lon_;
  double heading_;
  AzimuthFly(ros::NodeHandle *nh) : n(nh) {
    service_azimuth = n->advertiseService("AzimuthFly", &AzimuthFly::fly, this);
    gps_sub_ = n->subscribe("/mavros/global_position/global", 1,
                            &AzimuthFly::callback_gps, this);
    heading_sub_ = n->subscribe("/mavros/global_position/compass_hdg", 1,
                                &AzimuthFly::callback_compass, this);
    control_pub_ = n->advertise<mavros_msgs::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 10);
  }

private:
  void callback_gps(const sensor_msgs::NavSatFix &msg) {
    current_lat_ = msg.latitude;
    current_lon_ = msg.longitude;
  }
  void callback_compass(const std_msgs::Float64 &msg) { heading_ = msg.data; }
  bool fly(coparos::Azimuth::Request &req, coparos::Azimuth::Response &res) {
    ros::Rate r(5);
    double wind_angle, wind_speed = 0;
    n->getParam("/wind_speed", wind_speed);
    n->getParam("/wind_angle", wind_angle);
    mavros_msgs::AttitudeTarget control;
    control.thrust = 100;
    auto azimuth =
        calculateBearing(current_lat_, current_lon_, req.lat2, req.lon2);
    auto distance =
        distanceEarth(current_lat_, current_lon_, req.lat2, req.lon2);
    auto client_set_mode =
        n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode cmd;
    cmd.request.base_mode = 1;
    cmd.request.custom_mode = "Guided_NoGPS";
    client_set_mode.call(cmd);

    auto delta = degToRad(heading_ - azimuth);
    control.body_rate.z = delta;
    control_pub_.publish(control);
    control.body_rate.z = 0;
    ros::Duration(4).sleep();
    double diff_angle = degToRad(wind_angle - azimuth);
    double wind_pitch = std::sin(diff_angle);
    double wind_roll = std::cos(diff_angle);
    int sign = (wind_pitch > 0) - (wind_pitch < 0);
    double set_pitch =
        std::abs(wind_speed * wind_pitch * koeff_speed_angle) > 15
            ? -sign * 15
            : -sign * wind_speed * wind_pitch * koeff_speed_angle;
    double set_additional_roll = wind_speed * wind_roll * koeff_speed_angle;
    double set_roll = (10 * koeff_speed_angle - set_additional_roll) > 15
                          ? 15
                          : 10 * koeff_speed_angle - set_additional_roll;
    double stop_time = 5;
    double time = distance / 10.0 - stop_time;
    ROS_INFO("Distance and course are calculated: course = %f", azimuth);
    ROS_INFO(", distance = %f", distance);
    ROS_INFO("Setting pitch = %f", set_pitch);
    ROS_INFO(", roll = %f", set_roll);
    ROS_INFO("Time of flight = %f", time);
    tf2::Quaternion q;
    q.setRPY(degToRad(set_pitch), degToRad(25), 0);
    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(q);
    // control.type_mask = 64;
    // control.orientation = quat_msg;
    control.orientation = quat_msg;
    control_pub_.publish(control);

    bool success = false;
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(time); // Timeout of 2 seconds
    while (ros::Time::now() - start_time < timeout) {

      q.setRPY(degToRad(set_pitch), degToRad(set_roll), 0);
      geometry_msgs::Quaternion quat_msg;
      quat_msg = tf2::toMsg(q);
      ROS_INFO("Setting pitch = %f", set_pitch);
      ROS_INFO("Setting roll = %f", set_roll);
      // control.type_mask = 64;
      // control.orientation = quat_msg;
      control.orientation = quat_msg;
      control_pub_.publish(control);
      r.sleep();
    }
    success = false;
    ROS_INFO("STOPPING");
    start_time = ros::Time::now();
    timeout = ros::Duration(stop_time);
    while (ros::Time::now() - start_time < timeout) {
      q.setRPY(degToRad(set_pitch), -degToRad(set_roll), 0);
      geometry_msgs::Quaternion quat_msg;
      quat_msg = tf2::toMsg(q);
      ROS_INFO("Setting pitch = %f", set_pitch);
      ROS_INFO("Setting roll = %f", -set_roll);
      // control.type_mask = 64;
      // control.orientation = quat_msg;
      control.orientation = quat_msg;
      control_pub_.publish(control);
      r.sleep();
    }
    cmd.request.base_mode = 1;
    cmd.request.custom_mode = "GUIDED";
    client_set_mode.call(cmd);
    // auto time = ros::Time::now().toSec();
    // tf2::Quaternion q;
    // q.setRPY(req.amax, 0, 0);
    // geometry_msgs::Quaternion quat_msg;
    // quat_msg = tf2::toMsg(q);
    // control_pub_.publish(quat_msg);
    return true;
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "az");
  ros::NodeHandle n;
  AzimuthFly service(&n);

  ROS_INFO("Ready to measure wind.");
  ros::spin();

  return 0;
}