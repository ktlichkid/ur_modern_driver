#include "ur_modern_driver/ros/force_controller.h"
#include "ur_modern_driver/log.h"

static const int32_t MULT_JOINTSTATE_ = 1000000;
static const std::string JOINT_STATE_REPLACE("{{JOINT_STATE_REPLACE}}");
static const std::string WATCHDOG_TIMEOUT_REPLACE("{{WATCHDOG_TIMEOUT_REPLACE}}");
static const std::string FORCE_MODE_REPLACE("{{FORCE_MODE_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string POSITION_PROGRAM = R"(
def driverProg():
	MULT_jointstate = {{JOINT_STATE_REPLACE}}

	watchdog_timeout = False
	watchdog_fed = True
	
	def feed_watchdog():
		enter_critical
  	watchdog_fed = True
		exit_critical
	end
  
	thread watchdogThread():
		while True:
      sleep({{WATCHDOG_TIMEOUT_REPLACE}})
      enter_critical
      if not watchdog_fed:
        watchdog_timeout = True
			watchdog_fed = False
			exit_critical
			end
		end
	end

  socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

  thread_watchdog = run watchdogThread()

  while not watchdog_timeout:
	  params_mult = socket_read_binary_integer(6)
	  if params_mult[0] > 0:
      feed_watchdog()
		  f = [params_mult[1] / MULT_jointstate, params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate]
      force_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], [1, 1, 1, 1, 1, 1], f, 2, {{FORCE_MODE_REPLACE}})
	  end
  end
  sleep(.05)
  socket_close()
  kill thread_watchdog
end
)";

ForceController::ForceController(URCommander &commander, std::string &reverse_ip, int reverse_port)
    : commander_(commander)
    , server_(reverse_port)
    , state_(RobotState::Error)
{
  double max_linear_speed, max_rotational_speed, watchdog_timeout;
  ros::param::get("~force_max_linear_speed", max_linear_speed);
  ros::param::get("~force_max_rotational_speed", max_rotational_speed);
  ros::param::get("~force_watchdog_timeout", watchdog_timeout);

  LOG_INFO("Initializing force controller subscriber");
  force_cmd_sub_ = nh_.subscribe("ur_driver/force_cmd", 1, &ForceController::force_cmd_cb, this);

  std::string res(POSITION_PROGRAM);

  std::ostringstream out;
  out << '[' << std::fixed << std::setprecision(4);
  for(int i = 0; i < 3; ++i) out << max_linear_speed << ", ";
  for(int i = 0; i < 3; ++i) out << max_rotational_speed << ", ";
  out << ']';

  res.replace(res.find(JOINT_STATE_REPLACE), JOINT_STATE_REPLACE.length(), std::to_string(MULT_JOINTSTATE_));
  res.replace(res.find(WATCHDOG_TIMEOUT_REPLACE), WATCHDOG_TIMEOUT_REPLACE.length(), std::to_string(watchdog_timeout));
  res.replace(res.find(FORCE_MODE_REPLACE), FORCE_MODE_REPLACE.length(), out.str());
  res.replace(res.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), reverse_ip);
  res.replace(res.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port));
  program_ = res;

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

  LOG_DEBUG("Robot successfully connected");
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

  uint8_t buf[sizeof(uint32_t) * 7];
  uint8_t *idx = buf;

  int32_t val = htobe32(static_cast<int32_t>(msg->force.x));
  append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->force.y));
  append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->force.z));
  append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->torque.x));
  append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->torque.y));
  append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->torque.z));
  append(idx, val);

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
