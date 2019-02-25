#pragma once

#include <ros/ros.h>
#include <string>

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Wrench.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/server.h"

class ForceController: public Service
{
private:
  std::atomic<bool> running_;

  ros::NodeHandle nh_;
  URCommander &commander_;
  URServer server_;
  ros::Subscriber force_cmd_sub_;
  RobotState state_;
  std::string program_;

  template <typename T>
  size_t append(uint8_t *buffer, T &val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

public:
  ForceController(URCommander &commander, std::string &reverse_ip, int reverse_port);
  void force_cmd_cb(const geometry_msgs::Wrench::ConstPtr& msg);
  void ping_cb(const std_msgs::Empty::ConstPtr& msg);
  void onRobotStateChange(RobotState state);

  bool start();
  void stop();
};
