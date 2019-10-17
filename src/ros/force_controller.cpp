#include "ur_modern_driver/ros/force_controller.h"
#include "ur_modern_driver/log.h"

static const int32_t MULTIPLIER_ = 1000000;
static const int32_t N_RETRIES = 3;
static const std::string MULTIPIER_REPLACE("{{MULTIPIER_REPLACE}}");
static const std::string F_MAX_REPLACE("{{F_MAX_REPLACE}}");
static const std::string T_MAX_REPLACE("{{T_MAX_REPLACE}}");
static const std::string K_P_REPLACE("{{K_P_REPLACE}}");
static const std::string K_Q_REPLACE("{{K_Q_REPLACE}}");
static const std::string FORCE_MODE_REPLACE("{{FORCE_MODE_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string FORCE_CONTROL_PROGRAM = R"(
def admittance_control():

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

  def multiply(l, k, len=-1):
    local i = 0
    if len == -1:
      len = get_list_length(l)
    end
    while i < len:
     l[i] = l[i] * k
     i = i + 1
    end
    return l
  end

  def add(l_1, l_2, len=-1):
    local i = 0
    if len == -1:
      len = get_list_length(l_1)
    end
    while i < len:
     l_1[i] = l_1[i] + l_2[i]
     i = i + 1
    end
    return l_1
  end

  def subtract(l_1, l_2, len=-1):
    local i = 0
    if len == -1:
      len = get_list_length(l_1)
    end
    while i < len:
     l_1[i] = l_1[i] - l_2[i]
     i = i + 1
    end
    return l_1
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

  global cmd = [0,0,0,0,0,0]
  global multiplier = {{MULTIPIER_REPLACE}}
  global k_p = {{K_P_REPLACE}}
  global k_q = {{K_Q_REPLACE}}

  global startup_pose = get_actual_tcp_pose()
  # startup_pose = p[0.550, -0.156, 0.028, 2.17, -2.22, 0.0607]
  global targ_pos = [startup_pose[0],startup_pose[1],startup_pose[2]]
  global targ_rot = vect_2_quaternion([startup_pose[3],startup_pose[4],startup_pose[5]])
  global targ_wrench = [0,0,0,0,0,0]

  zero_ftsensor()
  global connected = socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

  thread Thread_force_control():
    while (True):
      global pose = get_actual_tcp_pose()

      global curr_pos = [pose[0], pose[1], pose[2]]
      global curr_rot = vect_2_quaternion([pose[3], pose[4], pose[5]])

      global err_pos = [targ_pos[0]-curr_pos[0], targ_pos[1]-curr_pos[1], targ_pos[2]-curr_pos[2]]
      global err_rot = quaternion_multiply(targ_rot, quaternion_conjugate(curr_rot))

      if (err_rot[3] < 0):
        global err_rot = quaternion_conjugate(err_rot)
      end

      global force_cmd = multiply(err_pos, k_p)
      global torque_cmd = multiply([err_rot[0], err_rot[1], err_rot[2]], k_q)

      global cmd = [force_cmd[0], force_cmd[1], force_cmd[2], torque_cmd[0], torque_cmd[1], torque_cmd[2]]
      cmd = add(cmd, targ_wrench)
      # cmd = [0., 0., 3.0, 0., 0., 0.]
      force_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], [1,1,1,1,1,1], cmd, 2, {{FORCE_MODE_REPLACE}})
      sync()
    end
  end

  threadId_Thread_force_control = run Thread_force_control()

  while (True):
    global params=socket_read_binary_integer(6)
    if (params[0] > 0):
      global targ_wrench = [params[1]/multiplier, params[2]/multiplier, params[3]/multiplier, params[4]/multiplier, params[5]/multiplier, params[6]/multiplier]
    end
    sync()
  end
end
)";


ForceController::ForceController(URCommander &commander, std::string &reverse_ip, int reverse_port)
    : running_(false)
    , workspace_upper_limit_{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()}
    , workspace_lower_limit_{-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()}
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
  ros::param::get("~workspace_upper_limit", workspace_upper_limit_);
  ros::param::get("~workspace_lower_limit", workspace_lower_limit_);

  LOG_INFO("Initializing force controller subscriber");
  wrench_cmd_sub_ = nh_.subscribe("ur_driver/wrench_cmd", 1, &ForceController::wrench_cmd_cb, this);

  std::string res(FORCE_CONTROL_PROGRAM);

  std::ostringstream out;
  out << std::fixed << std::setprecision(4);
  out << '[' << v_max << ", " << v_max << ", " << v_max << ", " << w_max << ", " << w_max << ", " << w_max << ']';

  res.replace(res.find(MULTIPIER_REPLACE), MULTIPIER_REPLACE.length(), std::to_string(MULTIPLIER_));
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

  int32_t retry_i = 0;
  for(; retry_i < N_RETRIES && !commander_.uploadProg(program_); ++retry_i)
  {
    LOG_ERROR("Program upload failed, retrying");
  }

  if(retry_i == N_RETRIES)
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

void ForceController::wrench_cmd_cb(const geometry_msgs::Wrench::ConstPtr& msg)
{
  if (!running_)
    return;

  uint8_t buf[sizeof(uint32_t) * 7];
  uint8_t *idx = buf;
  int32_t val;

  val = htobe32(static_cast<int32_t>(msg->force.x * MULTIPLIER_));
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
