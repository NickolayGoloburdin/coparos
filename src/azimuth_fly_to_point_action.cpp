#include <actionlib/server/simple_action_server.h>
#include <coparos/AzimuthFlyAction.h>
#include <ros/ros.h>

float getAzimuth(double lat1, double lon1, double lat2, double lon2) {}
// function TForm1.CalcDirection(Lat1,Long1,Lat2,Long2: Extended) : Extended;
//  var e : Extended; //эксцентриситет сферойда
//  d_long : Extended; //числитель формулы
//  a,b : Extended; //постоянные Земли
// begin

//  //сперва для прямых углов:
//  if (lat1=lat2) and (long1<long2) then
//  begin
//   Result:=90;
//   exit;
//  end
//  else
//  if (lat1=lat2) and (long1>long2) then
//   begin
//   Result:=270;
//   exit;
//  end
//  else
//  if (lat1>lat2) and (long1=long2) then
//   begin
//   Result:=180;
//   exit;
//  end
//  else
//  if (lat1<lat2) and (long1=long2) then
//   begin
//   Result:=0;
//   exit;
//  end;

//  a:=6378137.0; //большая полуось Земли (экваториальный радиус)
//  b:=6371000.0; //малая полуось Земли (полярный радиус)
//  e := sqrt(1-(b*b)/(a*a));

//  if abs(long2-long1)<=180 then
//   d_long:=long2-long1
//  else
//  if long2-long1<-180 then
//   d_long:=360+long2-long1
//  else
//  if long2-long1>180 then
//   d_long:=long2-long1-360;

//  Result:=(180/pi)*ArcTan(DegToRad(d_long) / (
//  ln(tan((pi/4)+(DegToRad(lat2)/2)) * Power(
//  (1-e*sin(DegToRad(lat2)))/(1+e*sin(DegToRad(lat2))) , e/2 ))
//  - ln(tan((pi/4)+(DegToRad(lat1)/2)) * Power(
//  (1-e*sin(DegToRad(lat1)))/(1+e*sin(DegToRad(lat1))) , e/2 )) ) );

//  if (lat1>lat2) and (Result<0) then
//   Result := Result + 180
//  else
//  if (long1>long2) and (lat1>lat2) then
//   Result := Result + 180
//  else
//  if (long1<long2) and (Round(Result)=-90) then
//   Result:= Result+180
//  else
//  if (long1>long2) and (Round(Result)=90) then
//   Result:= Result+180;

//  if Result < 0 then
//   Result := 360 + Result;

// end;
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