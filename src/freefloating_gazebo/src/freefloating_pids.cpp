#include <freefloating_gazebo/freefloating_pids.h>

using std::cout;
using std::endl;
using std::string;

FreeFloatingPids::FreeFloatingPids()
{
    setpoint_received_ = state_received_ = false;
    position_pids_.clear();
    velocity_pids_.clear();
}

void FreeFloatingPids::UpdatePositionPID()
{
    for(std::vector<pid_st>::iterator pid = position_pids_.begin(); pid!=position_pids_.end();++pid)
        *(pid->command_ptr) = pid->pid.computeCommand(*(pid->error_ptr), dt_);
}

void FreeFloatingPids::UpdateVelocityPID()
{
    for(std::vector<pid_st>::iterator pid = velocity_pids_.begin(); pid!=velocity_pids_.end();++pid)
        *(pid->command_ptr) = pid->pid.computeCommand(*(pid->error_ptr), dt_);
}

bool FreeFloatingPids::ToPositionControl(std_srvs::EmptyRequest &_req, std_srvs::EmptyResponse &_res)
{
    ROS_INFO("Switching to position control");
    control_type_ = POSITION_CONTROL;
    return true;
}

bool FreeFloatingPids::ToVelocityControl(std_srvs::EmptyRequest &_req, std_srvs::EmptyResponse &_res)
{
    ROS_INFO("Switching to velocity control");
    control_type_ = VELOCITY_CONTROL;
    return true;
}

void FreeFloatingPids::InitPID(control_toolbox::Pid &_pid, const ros::NodeHandle&_node, const bool &_use_dynamic_reconfig)
{
    if(_use_dynamic_reconfig)
    {
        // classical PID init
        _pid.init(_node);
    }
    else
    {
        control_toolbox::Pid::Gains gains;

        // Load PID gains from parameter server
        if (!_node.getParam("p", gains.p_gain_))
        {
          ROS_ERROR("No p gain specified for pid.  Namespace: %s", _node.getNamespace().c_str());
          return;
        }
        // Only the P gain is required, the I and D gains are optional and default to 0:
        _node.param("i", gains.i_gain_, 0.0);
        _node.param("d", gains.d_gain_, 0.0);

        // Load integral clamp from param server or default to 0
        double i_clamp;
        _node.param("i_clamp", i_clamp, 0.0);
        gains.i_max_ = std::abs(i_clamp);
        gains.i_min_ = -std::abs(i_clamp);
        _pid.setGains(gains);
    }
}
