#include "copa_types.h"
#include "coparos/Service_command.h"
#include "ros/ros.h"
#include <coparos/Ack.h>
#include <coparos/Command.h>
class ServiceHandler {
public:
  ros::Publisher cmd_pub_;
  ros::NodeHandle *n;
  coparos::Command msg;
  coparos::Ack ack_msg;
  ros::Subscriber ack_processor;
  ros::ServiceServer service_arm;
  ros::ServiceServer service_disarm;
  ros::ServiceServer service_takeoff;
  ros::ServiceServer service_land;
  ros::ServiceServer service_rtl;
  ros::ServiceServer service_stop;
  ros::ServiceServer service_continue_flight;
  ros::ServiceServer service_start_mission;
  ros::ServiceServer service_clear_mission;
  ServiceHandler(ros::NodeHandle *nh) : n(nh) {
    cmd_pub_ = n->advertise<coparos::Command>("/command", 1000);
    service_arm = n->advertiseService("Arm", &ServiceHandler::arm, this);
    service_disarm =
        n->advertiseService("Disarm", &ServiceHandler::disarm, this);
    service_takeoff =
        n->advertiseService("Takeoff", &ServiceHandler::takeoff, this);
    service_land = n->advertiseService("Land", &ServiceHandler::land, this);
    service_rtl = n->advertiseService("RTL", &ServiceHandler::rtl, this);
    service_stop = n->advertiseService("STOP", &ServiceHandler::stop, this);
    service_continue_flight =
        n->advertiseService("Continue", &ServiceHandler::continue_flight, this);
    service_start_mission = n->advertiseService(
        "Start_mission", &ServiceHandler::start_mission, this);
    service_clear_mission = n->advertiseService(
        "Clear_mission", &ServiceHandler::clear_mission, this);
    ack_processor =
        n->subscribe("/ack", 1000, &ServiceHandler::callback_ack, this);
  }
  void callback_ack(const coparos::Ack msg) { ack_msg = msg; }
  bool arm(coparos::Service_command::Request &req,
           coparos::Service_command::Response &res) {

    ROS_INFO("ARMING");
    msg.command = CMD_NAV_MOTORS_ON;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == CMD_NAV_MOTORS_ON) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
    //   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  }
  bool disarm(coparos::Service_command::Request &req,
              coparos::Service_command::Response &res) {
    ROS_INFO("DISARMING");
    msg.command = CMD_NAV_MOTORS_OFF;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_MOTORS_OFF)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool takeoff(coparos::Service_command::Request &req,
               coparos::Service_command::Response &res) {
    ROS_INFO("TAKING OFF");
    msg.command = CMD_NAV_TAKE_OFF;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_TAKE_OFF)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool land(coparos::Service_command::Request &req,
            coparos::Service_command::Response &res) {
    ROS_INFO("LANDING");
    msg.command = CMD_NAV_TO_LAND;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_TO_LAND)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }

  bool rtl(coparos::Service_command::Request &req,
           coparos::Service_command::Response &res) {
    ROS_INFO("Returning to launch");
    msg.command = CMD_NAV_GOTO_HOME;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_GOTO_HOME)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool stop(coparos::Service_command::Request &req,
            coparos::Service_command::Response &res) {
    ROS_INFO("Stopping");
    msg.command = CMD_NAV_STOP_MOTION;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_STOP_MOTION)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool continue_flight(coparos::Service_command::Request &req,
                       coparos::Service_command::Response &res) {
    ROS_INFO("Continue");
    msg.command = CMD_NAV_CONTINUE_MOTION;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_CONTINUE_MOTION)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool start_mission(coparos::Service_command::Request &req,
                     coparos::Service_command::Response &res) {
    ROS_INFO("Starting 1 click mission");
    msg.command = CMD_EXEC_SWITCH_N;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_EXEC_SWITCH_N)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
  bool clear_mission(coparos::Service_command::Request &req,
                     coparos::Service_command::Response &res) {
    ROS_INFO("Cleaning mission");
    msg.command = CMD_NAV_CLEAR_WP;
    cmd_pub_.publish(msg);
    auto ack = ros::topic::waitForMessage<coparos::Ack>("/ack", *n,
                                                        ros::Duration(0.1));
    if (ack) {
      if (ack->command == uint16_t(CMD_NAV_CLEAR_WP)) {
        if (ack->result) {
          res.status = "Success";
          res.result = true;
          return true;
        } else {
          res.status = ack->status;
          res.result = false;
          return true;
        }
      } else {
        res.status = "Other command recieved";
        res.result = false;
        return true;
      }
    } else {
      res.status = "Controller doesn't response";
      res.result = false;
      return true;
    }
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "services_server");
  ros::NodeHandle n;
  ServiceHandler sh(&n);
  ros::spin();

  return 0;
}