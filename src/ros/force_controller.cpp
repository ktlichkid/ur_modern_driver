#include "ur_modern_driver/ros/force_controller.h"
#include "ur_modern_driver/log.h"

static const int32_t MULTIPLIER_ = 1000000;
static const std::string MULTIPIER_REPLACE("{{MULTIPIER_REPLACE}}");
static const std::string F_MAX_REPLACE("{{F_MAX_REPLACE}}");
static const std::string T_MAX_REPLACE("{{T_MAX_REPLACE}}");
static const std::string K_P_REPLACE("{{K_P_REPLACE}}");
static const std::string K_Q_REPLACE("{{K_Q_REPLACE}}");
static const std::string FORCE_MODE_REPLACE("{{FORCE_MODE_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string FORCE_CONTROL_PROGRAM = R"(
def force_mode_p_control():
  def clip(l, low, high):
    local i = 0
    while i < get_list_length(l):
     if l[i] > high:
        l[i] = high
     elif l[i] < low:
        l[i] = low
     end
     i = i + 1
    end
    return l
  end

  def list_mul(l, k):
    local i = 0
    while i < get_list_length(l):
     l[i] = l[i] * k
     i = i + 1
    end
    return l
  end

  def vect_2_quaternion(vect):
    local angle = norm(vect)
    if angle < 0.0000000001:
      return [0,0,0,1]
    else:
      local axis = [vect[0]/angle, vect[1]/angle, vect[2]/angle]
      local s = sin(angle * 0.5) / norm(axis)
      local q = [axis[0]/s, axis[1]/s, axis[2]/s, cos(angle * 0.5)]
      return q
    end
  end

  def quaternion_conjugate(q):
    q_conj = [-q[0], -q[1], -q[2], q[3]]
    return q_conj
  end

  def quaternion_multiply(q1, q0):
    return [q1[0]*q0[3]+q1[1]*q0[2]-q1[2]*q0[1]+q1[3]*q0[0], -q1[0]*q0[2]+q1[1]*q0[3]+q1[2]*q0[0]+q1[3]*q0[1], q1[0]*q0[1]-q1[1]*q0[0]+q1[2]*q0[3]+q1[3]*q0[2], -q1[0]*q0[0]-q1[1]*q0[1]-q1[2]*q0[2]+q1[3]*q0[3]]
  end

  global multiplier = {{MULTIPIER_REPLACE}}
  global startup_pose = get_actual_tcp_pose()
  global targ_pos = [startup_pose[0],startup_pose[1],startup_pose[2]]
  global targ_rot = vect_2_quaternion([startup_pose[3],startup_pose[4],startup_pose[5]])
  global F_max = {{F_MAX_REPLACE}}
  global T_max = {{T_MAX_REPLACE}}
  global k_p = {{K_P_REPLACE}}
  global k_q = {{K_Q_REPLACE}}

  zero_ftsensor()

  global connected = socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

  thread Thread_force_control():
    while (True):
      local curr = get_actual_tcp_pose ()
      local curr_pos=[curr[0],curr[1],curr[2]]
      local curr_rot=vect_2_quaternion([curr[3],curr[4],curr[5]])
      local err_pos=[targ_pos[0]-curr_pos[0],targ_pos[1]-curr_pos[1],targ_pos[2]-curr_pos[2]]
      local err_rot=quaternion_multiply(targ_rot, quaternion_conjugate(curr_rot))
      if (err_rot[3] < 0):
        local err_rot=quaternion_conjugate(err_rot)
      end
      local force_cmd=clip(list_mul(err_pos, k_p),-F_max,F_max)
      local torque_cmd=clip(list_mul([err_rot[0], err_rot[1], err_rot[2]], k_q),-T_max,T_max)
      local cmd=[force_cmd[0], force_cmd[1], force_cmd[2], torque_cmd[0], torque_cmd[1], torque_cmd[2]]
      force_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], [1,1,1,1,1,1], cmd, 2, {{FORCE_MODE_REPLACE}})
      sync()
    end
  end

  threadId_Thread_force_control = run Thread_force_control()

  while (True):
    global params=socket_read_binary_integer(7)
    if (params[0] > 0):
      global targ_pos=[params[1]/multiplier, params[2]/multiplier,params[3]/multiplier]
      global targ_rot=[params[4]/multiplier,params[5]/multiplier,params[6]/multiplier,params[7]/multiplier]
    end
    sync()
  end
end
)";

ForceController::ForceController(URCommander &commander, std::string &reverse_ip, int reverse_port)
    : running_(false)
    , commander_(commander)
    , server_(reverse_port)
    , state_(RobotState::Error)
{
  double v_max, w_max, F_max, T_max, k_p, k_q;
  ros::param::get("~max_velocity_linear", v_max);
  ros::param::get("~max_velocity_rotation", w_max);
  ros::param::get("~max_force", F_max);
  ros::param::get("~max_torque", T_max);
  ros::param::get("~linear_gain", k_p);
  ros::param::get("~rotational_gain", k_q);
  ros::param::get("~default_orientation", default_orientation_);

  LOG_INFO("Initializing force controller subscriber");
  pose_cmd_sub_ = nh_.subscribe("ur_driver/pose_cmd", 1, &ForceController::pose_cmd_cb, this);
  position_cmd_sub_ = nh_.subscribe("ur_driver/position_cmd", 1, &ForceController::position_cmd_cb, this);

  std::string res(FORCE_CONTROL_PROGRAM);

  std::ostringstream out;
  out << std::fixed << std::setprecision(4);
  out << '[' << v_max << ", " << v_max << ", " << v_max << ", " << w_max << ", " << w_max << ", " << w_max << ']';

  res.replace(res.find(MULTIPIER_REPLACE), MULTIPIER_REPLACE.length(), std::to_string(MULTIPLIER_));
  res.replace(res.find(F_MAX_REPLACE), F_MAX_REPLACE.length(), std::to_string(F_max));
  res.replace(res.find(T_MAX_REPLACE), T_MAX_REPLACE.length(), std::to_string(T_max));
  res.replace(res.find(K_P_REPLACE), K_P_REPLACE.length(), std::to_string(k_p));
  res.replace(res.find(K_Q_REPLACE), K_Q_REPLACE.length(), std::to_string(k_q));
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

void ForceController::position_cmd_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  geometry_msgs::Pose *pose = new geometry_msgs::Pose();
  pose->position.x = msg->x;
  pose->position.y = msg->y;
  pose->position.z = msg->z;
  pose->orientation.x = default_orientation_[0];
  pose->orientation.y = default_orientation_[1];
  pose->orientation.z = default_orientation_[2];
  pose->orientation.w = default_orientation_[3];

  pose_cmd_cb(geometry_msgs::Pose::ConstPtr(pose));
}

void ForceController::pose_cmd_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (!running_)
    return;

  uint8_t buf[sizeof(uint32_t) * 7];
  uint8_t *idx = buf;

  int32_t val = htobe32(static_cast<int32_t>(msg->position.x * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->position.y * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->position.z * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->orientation.x * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->orientation.y * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->orientation.z * MULTIPLIER_));
  idx += append(idx, val);
  val = htobe32(static_cast<int32_t>(msg->orientation.w * MULTIPLIER_));
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
