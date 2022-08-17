#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <coparos/AzimuthFlyAction.h>
#include <coparos/GPS.h>
#include <coparos/Service_command.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
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

  // std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "

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

class AzimuthFlyActionServer {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<coparos::AzimuthFlyAction>
      as_; // NodeHandle instance must be created before this line. Otherwise
           // strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  coparos::AzimuthFlyFeedback feedback_;
  coparos::AzimuthFlyResult result_;

public:
  ros::Publisher log_pub_;
  ros::Publisher angles_pub_;

  AzimuthFlyActionServer(std::string name)
      : as_(nh_, name,
            boost::bind(&AzimuthFlyActionServer::executeCB, this, _1), false),
        action_name_(name) {
    as_.start();
    log_pub_ = nh_.advertise<std_msgs::String>("/logging_topic", 1000);
    angles_pub_ = nh_.advertise<geometry_msgs::Vector3>("/manualAngles", 1000);
  }

  ~AzimuthFlyActionServer(void) {}

  void executeCB(const coparos::AzimuthFlyGoalConstPtr &goal) {
    // helper variables
    //     ros::Rate r(1);
    std_msgs::String log;
    double lat1, lon1, lat2, lon2, wind_angle, wind_speed;
    nh_.getParam("/wind_speed", wind_speed);
    nh_.getParam("/wind_angle", wind_angle);
    coparos::GPS gps;
    coparos::Service_command cmd;
    ros::ServiceClient client_gps, client_yaw, client_flight_mode;
    client_gps = nh_.serviceClient<coparos::GPS>("Get_gps");
    client_yaw = nh_.serviceClient<coparos::Service_command>("Set_yaw");
    if (client_gps.call(gps)) {
      log.data = "Current coordinates, lat = " + std::to_string(lat1) +
                 ", lon = " + std::to_string(lon1);
      lat1 = gps.response.lat;
      lon1 = gps.response.lon;
      log_pub_.publish(log);
    }
    lat2 = goal->target.targetLat;
    lon2 = goal->target.targetLon;
    double azimuth = calculateBearing(lat1, lon1, lat2, lon2);
    double distance = distanceEarth(lat1, lon1, lat2, lon2);
    log.data = "Distance and course are calculated: course = " +
               std::to_string(azimuth) +
               ", distance = " + std::to_string(distance);
    log_pub_.publish(log);
    cmd.request.param1 = azimuth;
    if (client_yaw.call(cmd)) {
      log.data = "Setting course...";
      log_pub_.publish(log);
    }
    double diff_angle = degToRad(wind_angle - azimuth);
    double wind_pitch = std::sin(diff_angle);
    double wind_roll = std::cos(diff_angle);
    double set_pitch =
        wind_speed * wind_pitch * 1.4 > 15 ? 15 : wind_speed * wind_pitch * 1.4;
    double set_additional_roll = wind_speed * wind_roll * 1.4;
    double set_roll =
        (14 + set_additional_roll) > 15 ? 15 : 14 + set_additional_roll;
    double time = distance / 10.0;
    geometry_msgs::Vector3 angles;

    //     // push_back the seeds for the fibonacci sequence
    //     feedback_.sequence.clear();
    //     feedback_.sequence.push_back(0);
    //     feedback_.sequence.push_back(1);

    //     // publish info to the console for the user
    //     ROS_INFO("%s: Executing, creating fibonacci sequence of order %i
    //     with "
    //              "seeds %i, %i",
    //              action_name_.c_str(), goal->order, feedback_.sequence[0],
    //              feedback_.sequence[1]);

    //     // start executing the action
    //     for (int i = 1; i <= goal->order; i++) {
    //       // check that preempt has not been requested by the client
    //       if (as_.isPreemptRequested() || !ros::ok()) {
    //         ROS_INFO("%s: Preempted", action_name_.c_str());
    //         // set the action state to preempted
    //         as_.setPreempted();
    //         success = false;
    //         break;
    //       }
    //       feedback_.sequence.push_back(feedback_.sequence[i] +
    //                                    feedback_.sequence[i - 1]);
    //       // publish the feedback
    //       as_.publishFeedback(feedback_);
    //       // this sleep is not necessary, the sequence is computed at 1 Hz
    //       for
    //       // demonstration purposes
    //       r.sleep();
    //     }

    //     if (success) {
    //       result_.sequence = feedback_.sequence;
    //       ROS_INFO("%s: Succeeded", action_name_.c_str());
    //       // set the action state to succeeded
    //       as_.setSucceeded(result_);
    //     }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "azimuth");

  AzimuthFlyActionServer azimuth("azimuth");
  ros::spin();

  return 0;
}