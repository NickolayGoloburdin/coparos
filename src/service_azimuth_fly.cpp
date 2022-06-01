#include "coparos/Azimuth.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "ros/ros.h"
#include <math.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define PI 3.14159265358979323846
#define RADIO_TERRESTRE 6372797.56085
#define GRADOS_RADIANES PI / 180
#define RADIANES_GRADOS 180 / PI
#define degToRad(angleInDegrees) ((angleInDegrees)*PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / PI)

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
    mavros_msgs::AttitudeTarget control;
    auto azimuth = calculateBearing(req.lat1, req.lon1, req.lat2, req.lon2);
    auto distance = distanceEarth(req.lat1, req.lon1, req.lat2, req.lon2);
    // auto client_set_mode =
    //     n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // mavros_msgs::SetMode cmd;
    // cmd.request.base_mode = 1;
    // cmd.request.custom_mode = "Guided_NoGPS";
    // client_set_mode.call(cmd);
    auto delta = degToRad(req.heading - azimuth);
    // tf2::Quaternion q;
    //  q.setRPY(0, 0, degToRad(delta));
    //  geometry_msgs::Quaternion quat_msg;
    //  quat_msg = tf2::toMsg(q);
    //  control.type_mask = 64;
    //  control.orientation = quat_msg;
    control.body_rate.z = delta;
    control_pub_.publish(control);
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