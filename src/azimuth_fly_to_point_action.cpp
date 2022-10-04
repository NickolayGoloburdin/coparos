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
#include <tuple>
#define PI 3.14159265358979323846
#define RADIO_TERRESTRE 6372797.56085
#define GRADOS_RADIANES PI / 180
#define RADIANES_GRADOS 180 / PI
#define degToRad(angleInDegrees) ((angleInDegrees)*PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / PI)
const float a = 6378137.0;
const float b = 6371000.0;

const double start_way = 16;
const double stop_way = 20;
std::string rounded(double a) {
  std::string num_text = std::to_string(a);
  return num_text.substr(0, num_text.find(".") + 2);
}
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
  geometry_msgs::Vector3 angles;
  std_msgs::String log;

  AzimuthFlyActionServer(std::string name)
      : as_(nh_, name,
            boost::bind(&AzimuthFlyActionServer::executeCB, this, _1), false),
        action_name_(name) {
    as_.start();
    log_pub_ = nh_.advertise<std_msgs::String>("/logging_topic", 1000);
    angles_pub_ = nh_.advertise<geometry_msgs::Vector3>("/manualAngles", 1000);
  }

  ~AzimuthFlyActionServer(void) {}
  void set_pitch_roll(double pitch, double roll) {
    angles.x = pitch;
    angles.y = roll;
    angles_pub_.publish(angles);
    logging("pitch = " + rounded(angles.x) + ", roll = " + rounded(angles.y));
  }
  void set_course(double course, double speed) {
    ros::ServiceClient client_yaw =
        nh_.serviceClient<coparos::Service_command>("Set_yaw");
    coparos::Service_command cmd;
    cmd.request.param1 = course;
    cmd.request.param2 = speed;
    client_yaw.call(cmd);
    logging("Setting course " + rounded(course));
  }
  std::tuple<double, double> get_gps() {
    ros::ServiceClient client_gps = nh_.serviceClient<coparos::GPS>("Get_gps");
    coparos::GPS gps;

    client_gps.call(gps);
    return std::make_tuple(gps.response.lat, gps.response.lon);
  }
  double calculate_target_pitch(double azimuth, double wind_angle,
                                double wind_speed, double k_speed,
                                double max_const_angle) {
    double diff_angle = degToRad(azimuth - wind_angle);
    double wind_pitch = std::cos(diff_angle);
    double set_additional_pitch = wind_speed * wind_pitch * k_speed;
    int pitch_sign = (wind_pitch > 0) - (wind_pitch < 0);
    return std::abs(max_const_angle - set_additional_pitch) > 25
               ? pitch_sign * 25
               : -max_const_angle + set_additional_pitch;
  }
  void logging(std::string log_str) {
    log.data = log_str;
    log_pub_.publish(log);
  }
  // double calculate_stop_pitch(double azimuth, double distance, double
  // start_way,
  //                             double stop_way, double wind_angle,
  //                             double wind_speed double k_speed,
  //                             double max_const_angle) {}
  double calculate_target_roll(double azimuth, double wind_angle,
                               double wind_speed, double k_speed,
                               double max_const_angle) {
    double diff_angle = degToRad(azimuth - wind_angle);
    double wind_roll = std::sin(diff_angle);
    int sign = (wind_roll > 0) - (wind_roll < 0);
    double stop_roll = wind_roll * k_speed * wind_speed; // for stopping drone
    return std::abs(stop_roll) > 15 ? sign * 15 : stop_roll;
  }
  double calculate_stop_pitch(double azimuth, double wind_angle,
                              double wind_speed, double k_speed,
                              double max_const_angle) {
    double diff_angle = degToRad(azimuth - wind_angle);
    double wind_pitch = std::cos(diff_angle);
    return wind_speed * wind_pitch * k_speed;
  }
  double calculate_stop_roll(double azimuth, double wind_angle,
                             double wind_speed, double k_speed,
                             double max_const_angle) {
    double diff_angle = degToRad(azimuth - wind_angle);
    double wind_roll = std::sin(diff_angle);
    int sign = (wind_roll > 0) - (wind_roll < 0);
    return wind_roll * k_speed * wind_speed; // for stopping drone
  }
  void executeCB(const coparos::AzimuthFlyGoalConstPtr &goal) {
    // helper variables
    ros::Rate r(1);

    double lat1, lon1, lat2, lon2, wind_angle, wind_speed, additional_speed,
        k_speed_angle;

    nh_.getParam("/wind_speed", wind_speed);
    nh_.getParam("/wind_angle", wind_angle);
    nh_.getParam("/addition_angle", additional_speed);
    nh_.getParam("/koeff_speed_angle", k_speed_angle);
    ros::ServiceClient client_flight_mode;

    std::tie(lat1, lon1) = get_gps();
    lat2 = goal->target.targetLat;
    lon2 = goal->target.targetLon;
    logging("Current lat = " + std::to_string(lat1) +
            ", lon = " + std::to_string(lon1));
    logging("Target lat = " + std::to_string(lat2) +
            ", lon = " + std::to_string(lon2));
    double azimuth = calculateBearing(lat1, lon1, lat2, lon2);
    azimuth = azimuth > 180.0 ? azimuth - 360.0 : azimuth;
    double distance = distanceEarth(lat1, lon1, lat2, lon2);
    logging("course = " + rounded(azimuth) +
            ", distance = " + std::to_string(int(distance)));
    set_course(azimuth, 60);
    // ros::Duration(4).sleep();
    double set_pitch = calculate_target_pitch(azimuth, wind_angle, wind_speed,
                                              k_speed_angle, 15);
    double set_roll = calculate_target_roll(azimuth, wind_angle, wind_speed,
                                            k_speed_angle, 15);
    double stop_pitch = calculate_stop_pitch(azimuth, wind_angle, wind_speed,
                                             k_speed_angle, 15);
    double stop_roll =
        calculate_stop_roll(azimuth, wind_angle, wind_speed, k_speed_angle, 15);
    double horizontal_speed = 10.0;
    if (stop_pitch < 0) {
      horizontal_speed = (set_pitch - stop_pitch) / 1.4;
    }
    double stop_time = 3;

    double time = (distance - start_way - stop_way) / horizontal_speed;
    logging("Stop pitch = " + rounded(stop_pitch) + ", Stop roll = " +
            rounded(stop_roll) + "\n" + "Flight pitch = " + rounded(set_pitch) +
            ", Fligh roll = " + rounded(set_roll));

    logging("Start time = " + std::to_string(2) +
            ", Const time = " + std::to_string(int(time)) +
            ", Stop time = " + std::to_string(int(stop_time)));

    set_pitch_roll(-25, set_roll);
    ros::Duration(2).sleep();

    ros::Time start_time = ros::Time::now();

    ros::Duration timeout(time); // Timeout of 2 seconds
    while (ros::Time::now() - start_time < timeout) {
      set_pitch_roll(set_pitch - additional_speed, set_roll);
      if (as_.isPreemptRequested()) {
        as_.setPreempted();
        set_pitch_roll(0, 0);
        logging("Action stopped from client");
        return;
      }
      r.sleep();
    }
    start_time = ros::Time::now();
    timeout = ros::Duration(stop_time);
    while (ros::Time::now() - start_time < timeout) {
      set_pitch_roll(-set_pitch + additional_speed, set_roll);
      if (as_.isPreemptRequested()) {
        as_.setPreempted();
        set_pitch_roll(0, 0);
        logging("Action stopped from client");
        return;
      }
      r.sleep();
    }
    set_pitch_roll(stop_pitch, stop_roll);
    ros::Duration(2).sleep();
    if (as_.isPreemptRequested()) {
      as_.setPreempted();
      set_pitch_roll(0, 0);
      logging("Action stopped from client");
      return;
    }
    logging("Drone reach the point");
    result_.target_reached = true;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
    //      // push_back the seeds for the fibonacci sequence
    //      feedback_.sequence.clear();
    //      feedback_.sequence.push_back(0);
    //      feedback_.sequence.push_back(1);

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