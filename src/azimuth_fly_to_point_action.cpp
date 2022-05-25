#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <coparos/AzimuthFlyAction.h>
#include <math.h>
#include <ros/ros.h>
#include <tuple>
#define PI 3.141592653589793238463

#define degToRad(angleInDegrees) ((angleInDegrees)*PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / PI)
const float a = 6378137.0;
const float b = 6371000.0;
inline double to_degrees(double radians) { return radians * (180.0 / PI); }
double getAzimuth(double lat1, double lon1, double lat2, double lon2) {
  float azimuth;
  float d_lon;

  const float e = std::sqrt(1 - (b * b) / (a * a));
  if (lat1 == lat2 && lon1 == lon2) {
    azimuth = 180.0;
  } else if (lat1 == lat2 && lon1 > lon2) {
    azimuth = 270.0;
  } else if (lat1 == lat2 && lon1 < lon2) {
    azimuth = 90.0;
  } else if (lat1 < lat2 && lon1 == lon2) {
    azimuth = 0.0;
  } else if (lat1 > lat2 && lon1 == lon2) {
    azimuth = 180.0;
  }
  if (std::fabs(lon2 - lon1) <= 180) {
    d_lon = lon1 - lon2;
  } else if (lon2 - lon1 < -180) {
    d_lon = 360 + lon2 - lon1;
  } else if (lon2 - lon1 > 180) {
    d_lon = lon2 - lon1 - 360;
  }

  azimuth =
      (180 / PI) *
      std::atan(degToRad(d_lon) /
                (std::log(std::tan((PI / 4) + (degToRad(lat2) / 2)) *
                          std::pow((double)(1 - e * std::sin(degToRad(lat2))) /
                                       (1 + e * std::sin(degToRad(lat2))),
                                   (double)e / 2)) -
                 std::log(tan((PI / 4) + (degToRad(lat1) / 2)) *
                          std::pow((double)(1 - e * sin(degToRad(lat1))) /
                                       (1 + e * sin(degToRad(lat1))),
                                   (double)e / 2))));
  if (lat1 > lat2 && azimuth < 0) {
    azimuth += 180;
  } else if (lon1 > lon2 && lat1 > lat2) {
    azimuth += 180;
  } else if (lon1 < lon2 && (int)azimuth == -90) {
    azimuth += 180;
  } else if (lon1 > lon2 && (int)azimuth == 90) {
    azimuth += 180;
  }
  if (azimuth < 0) {
    azimuth += 360;
  }
  return azimuth;
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
  AzimuthFlyActionServer(std::string name)
      : as_(nh_, name,
            boost::bind(&AzimuthFlyActionServer::executeCB, this, _1), false),
        action_name_(name) {
    as_.start();
  }

  ~AzimuthFlyActionServer(void) {}

  void executeCB(const coparos::AzimuthFlyGoalConstPtr &goal) {
    // helper variables
    //     ros::Rate r(1);
    //     bool success = true;

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
  ros::init(argc, argv, "fibonacci");

  AzimuthFlyActionServer azimuth("fibonacci");
  ros::spin();

  return 0;
}