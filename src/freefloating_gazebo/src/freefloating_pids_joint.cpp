#include <freefloating_gazebo/freefloating_pids_joint.h>

using std::cout;
using std::endl;
using std::string;


void FreeFloatingJointPids::Init(const ros::NodeHandle&_node, ros::Duration&_dt)
{
    // init dt from rate
    dt_ = _dt;
    pid_node_ = _node;

    // store dynamic reconfigure

    // filter param
    alpha_ = .5;

    // get params from server
    std::vector<std::string> joint_names;
    _node.getParam("config/joints/name", joint_names);

    // get whether or not we use a cascaded controller for position
    _node.param("config/joints/cascaded_position", cascaded_position_pid_, true);

    // get whether or not we use dynamic reconfigure
    bool use_dynamic_reconfig;
    _node.param("config/joints/dynamic_reconfigure", use_dynamic_reconfig, false);

    // resize vectors
    const unsigned int n = joint_names.size();
    position_pids_.resize(n);
    velocity_pids_.resize(n);
    position_error_.resize(n);
    velocity_error_.resize(n);
    position_filtered_measure_.resize(n);
    velocity_filtered_measure_.resize(n);

    joint_measure_.velocity.resize(n);
    joint_measure_.position.resize(n);
    joint_setpoint_.velocity.resize(n);
    joint_setpoint_.position.resize(n);
    joint_setpoint_.name = joint_names;
    joint_command_.name = joint_names;
    joint_command_.effort.resize(n);

    // store limits
    _node.getParam("config/joints/upper", joint_upper_);
    _node.getParam("config/joints/lower", joint_lower_);
    _node.getParam("config/joints/velocity", joint_max_velocity_);
    vmax_is_set_ = !cascaded_position_pid_;

    for(unsigned int i=0;i<n;++i)
    {
        position_pids_[i].error_ptr = &(position_error_[i]);
        velocity_pids_[i].error_ptr = &(velocity_error_[i]);
        velocity_pids_[i].command_ptr = &(joint_command_.effort[i]);

        // position PID output depends on control type
        if(cascaded_position_pid_)
            position_pids_[i].command_ptr = &(joint_setpoint_.velocity[i]);
        else
            position_pids_[i].command_ptr = &(joint_command_.effort[i]);

        position_filtered_measure_[i] = velocity_filtered_measure_[i] = 0;

        InitPID(position_pids_[i].pid, ros::NodeHandle(_node, joint_names[i] + "/position"), use_dynamic_reconfig);
        InitPID(velocity_pids_[i].pid, ros::NodeHandle(_node, joint_names[i] + "/velocity"), use_dynamic_reconfig);
    }

    InitSwitchServices("joint");

}



bool FreeFloatingJointPids::UpdatePID()
{
    if(!vmax_is_set_)
    {
        control_toolbox::Pid::Gains gains;
        bool antiwindup;

        for(unsigned int i=0;i<velocity_error_.size();++i)
        {
               gains = position_pids_[i].pid.getGains();
            //position_pids_[i].pid.getGains(dummy, dummy, dummy, joint_max_velocity_[i], dummy, antiwindup);
               joint_max_velocity_[i] = gains.i_max_;

        }
    }

    if(setpoint_received_ && state_received_)
    {
        if(control_type_ == POSITION_CONTROL)
        {
            //  cout << "Joint position error: ";
            // get position error
            for(unsigned int i=0;i<position_error_.size();++i)
            {
                position_filtered_measure_[i] = filters::exponentialSmoothing(joint_measure_.position[i], position_filtered_measure_[i], alpha_);
                position_error_[i] = filters::clamp(joint_setpoint_.position[i], joint_lower_[i], joint_upper_[i]) - position_filtered_measure_[i];
                //      cout << position_error_[i];
            }
            //   cout << endl;
            // update pid's
            UpdatePositionPID();
            // has written new velocity setpoint
        }

        if(control_type_ == VELOCITY_CONTROL || cascaded_position_pid_)
        {
            // get velocity error
            for(unsigned int i=0;i<velocity_error_.size();++i)
            {
                // if joint is a max bound, error is 0
                if((joint_measure_.position[i] >= joint_upper_[i] && joint_setpoint_.velocity[i] > 0) ||
                        (joint_measure_.position[i] <= joint_lower_[i] && joint_setpoint_.velocity[i] < 0))
                    velocity_error_[i] = 0;
                else
                {
                    velocity_filtered_measure_[i] = filters::exponentialSmoothing(joint_measure_.velocity[i], velocity_filtered_measure_[i], alpha_);
                    //  cout << "Joint " << i+1 << "vel: " << joint_setpoint_.velocity[i] << ", min: " << -joint_max_velocity_[i] << ", max: " << joint_max_velocity_[i] << endl;
                    velocity_error_[i] = filters::clamp(joint_setpoint_.velocity[i], -joint_max_velocity_[i], joint_max_velocity_[i]) - velocity_filtered_measure_[i];
                }
            }

            /*   for(unsigned int i=0;i<velocity_error_.size();++i)
            {
                cout << "Joint " << i+1 << ": setpoint clamped = " << filters::clamp(joint_setpoint_.velocity[i], -joint_max_velocity_[i], joint_max_velocity_[i]) << ", measure = " << velocity_filtered_measure_[i] << ", error = " << velocity_error_[i] << ", command = " << joint_command_.effort[i] << endl;
            }*/

            // update pid's
            UpdateVelocityPID();
        }

        return true;
    }

    return false;
}

void FreeFloatingJointPids::SetpointCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
    setpoint_received_ = true;

    // don't assume joints are ordered the same!
    //joint_setpoint_ = *_msg;


    // check for joint ordering
    unsigned int i,j;

    for(i=0;i<joint_setpoint_.name.size();++i)
    {
        for(j=0;j<_msg->name.size();++j)
        {
            if(joint_setpoint_.name[i] == _msg->name[j])
            {
                if(_msg->position.size() > j)
                    joint_setpoint_.position[i] = _msg->position[j];
                if(_msg->velocity.size() > j)
                    joint_setpoint_.velocity[i] = _msg->velocity[j];
            }
        }
    }
}



void FreeFloatingJointPids::MeasureCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
    state_received_ = true;
    // assume joints are ordered the same, should be the case when the measure comes from Gazebo
    joint_measure_ = *_msg;
}
