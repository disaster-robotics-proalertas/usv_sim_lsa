#ifndef FREEFLOATINGJOINTPID_H
#define FREEFLOATINGJOINTPID_H

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>
#include <freefloating_gazebo/freefloating_pids.h>

class FreeFloatingJointPids : public FreeFloatingPids
{

public:
    void Init(const ros::NodeHandle&_node, ros::Duration&_dt);
    unsigned int JointNb() {return joint_lower_.size();}

    // parse received body setpoint
    void SetpointCallBack(const sensor_msgs::JointStateConstPtr& _msg);
    // parse received body measure
    void MeasureCallBack(const sensor_msgs::JointStateConstPtr& _msg);

    // update PID's
    bool UpdatePID();

    // get wrench command
    inline sensor_msgs::JointState EffortCommand() {return joint_command_;}

private:
    std::vector<double> joint_lower_, joint_upper_, joint_max_velocity_;
    std::vector<double> position_error_, velocity_error_, position_filtered_measure_, velocity_filtered_measure_;
    sensor_msgs::JointState joint_setpoint_, joint_measure_, joint_command_;
    bool vmax_is_set_;
    bool cascaded_position_pid_;
    double alpha_;

};

#endif // FREEFLOATINGJOINTPID_H
