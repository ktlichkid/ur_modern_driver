#include "ur_modern_driver/ros/force_controller.h"
#include "ur_modern_driver/log.h"


static const int32_t MULTIPLIER_ = 1000000;
static const std::string MULTIPIER_REPLACE("{{MULTIPIER_REPLACE}}");
static const std::string WATCHDOG_TIMEOUT_REPLACE("{{WATCHDOG_TIMEOUT_REPLACE}}");
static const std::string FORCE_MODE_REPLACE("{{FORCE_MODE_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string POSITION_PROGRAM = R"(
def force_mode():
  global cmd=[0,0,0,0,0,0]
  global multiplier={{MULTIPIER_REPLACE}}
  global wd_timer=0
  global wd_timer_is_counting=False
  thread Timer_Thread():
    while (True):
      if (wd_timer_is_counting):
        wd_timer = wd_timer + get_steptime()
      end
      sync()
    end
  end
  run Timer_Thread()
  zero_ftsensor()
  global connected=socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})
  wd_timer_is_counting = True
  thread read_Thread():
    while (True):
      global params=socket_read_binary_integer(6)
      if (params[0] > 0):
        global cmd=[params[1]/multiplier, params[2]/multiplier,params[3]/multiplier,params[4]/multiplier,params[5]/multiplier,params[6]/multiplier]
        force_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], [1,1,1,1,1,1], cmd, 2, {{FORCE_MODE_REPLACE}})
        wd_timer = 0
      end
      sleep(0.01)
    end
  end
  global read_thread_handler=run read_Thread()
  while (wd_timer<{{WATCHDOG_TIMEOUT_REPLACE}}):
    sync()
  end
  kill read_thread_handler
  end_force_mode()
  socket_close()
end
)";

ForceController::ForceController(URCommander &commander, std::string &reverse_ip, int reverse_port)
    : running_(false)
    , commander_(commander)
    , server_(reverse_port)
    , state_(RobotState::Error)
{
  double max_linear_speed, max_rotational_speed, wd_timeout;
  ros::param::get("~force_control_max_linear_speed", max_linear_speed);
  ros::param::get("~force_control_max_rotational_speed", max_rotational_speed);
  ros::param::get("~force_control_watchdog_timeout", wd_timeout);

  LOG_INFO("Initializing force controller subscriber");
  force_cmd_sub_ = nh_.subscribe("ur_driver/force_cmd", 1, &ForceController::force_cmd_cb, this);

  std::string res(POSITION_PROGRAM);

  std::ostringstream out;
  out << std::fixed << std::setprecision(4);
  out << '[' << max_linear_speed << ", " << max_linear_speed << ", " << max_linear_speed << ", ";
  out << max_rotational_speed << ", " << max_rotational_speed << ", " << max_rotational_speed << ']';

  res.replace(res.find(MULTIPIER_REPLACE), MULTIPIER_REPLACE.length(), std::to_string(MULTIPLIER_));
  res.replace(res.find(WATCHDOG_TIMEOUT_REPLACE), WATCHDOG_TIMEOUT_REPLACE.length(), std::to_string(wd_timeout));
  res.replace(res.find(FORCE_MODE_REPLACE), FORCE_MODE_REPLACE.length(), out.str());
  res.replace(res.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), reverse_ip);
  res.replace(res.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port));
  program_ = res;

  if (!server_.bind())
  {
    LOG_ERROR("Failed to bind server, the port %d is likely already in use", reverse_port);
    std::exit(-1);
  }

  LOG_INFO("Force controller initialized");
}

bool ForceController::start()
{
  if (running_)
    return true;

  LOG_INFO("Uploading force control program to robot");

  if (!commander_.uploadProg(program_))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }

  LOG_DEBUG("Awaiting incoming robot connection");

  if (!server_.accept())
  {
    LOG_ERROR("Failed to accept incoming robot connection");
    return false;
  }

  LOG_INFO("Robot successfully connected");
  return (running_ = true);
}

void ForceController::onRobotStateChange(RobotState state)
{
   state_ = state;
}

void ForceController::force_cmd_cb(const geometry_msgs::Wrench::ConstPtr& msg)
{
  if (!running_)
    return;

  uint8_t buf[sizeof(uint32_t) * 6];
  uint8_t *idx = buf;

  int32_t val = htobe32(static_cast<int32_t>(msg->force.x * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->force.y * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->force.z * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->torque.x * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->torque.y * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->torque.z * MULTIPLIER_));
  idx += append(idx, val);

  size_t written;
  server_.write(buf, sizeof(buf), written);
}

void ForceController::stop()
{
  if (!running_)
    return;

  server_.disconnectClient();
  running_ = false;
}
