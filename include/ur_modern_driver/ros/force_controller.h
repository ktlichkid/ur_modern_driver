#pragma once

#include <ros/ros.h>
#include <string>

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/server.h"

class ForceController: public Service
{
private:
  std::atomic<bool> running_;
  std::vector<double> default_orientation_, workspace_upper_limit_, workspace_lower_limit_;

  ros::NodeHandle nh_;
  URCommander &commander_;
  URServer server_;
  ros::Subscriber pose_cmd_sub_, position_cmd_sub_, wrench_cmd_sub_;
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
  void pose_cmd_cb(const geometry_msgs::Pose::ConstPtr& msg);
  void position_cmd_cb(const geometry_msgs::Point::ConstPtr& msg);
  void wrench_cmd_cb(const geometry_msgs::Wrench::ConstPtr& msg);
  void onRobotStateChange(RobotState state);

  bool start();
  void stop();

  virtual ~ForceController(){};
};
