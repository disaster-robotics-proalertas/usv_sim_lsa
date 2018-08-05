#ifndef FREEFLOATINGPID_H
#define FREEFLOATINGPID_H

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <control_toolbox/filters.h>
#include <ros/duration.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>

class FreeFloatingPids
{
protected:
    typedef enum
    {
        EFFORT_CONTROL, VELOCITY_CONTROL, POSITION_CONTROL
    } control_type;

    struct pid_st
    {
        double* error_ptr;
        double* command_ptr;
        control_toolbox::Pid pid;
    };

public:
    FreeFloatingPids();
    ~FreeFloatingPids() {}

    // these updates do no check about the pointers
    void UpdatePositionPID();
    void UpdateVelocityPID();

protected:

    // service to switch controllers
    bool ToPositionControl(std_srvs::EmptyRequest &_req, std_srvs::EmptyResponse &_res);
    bool ToVelocityControl(std_srvs::EmptyRequest &_req, std_srvs::EmptyResponse &_res);

    void InitSwitchServices(const std::string &_name)
    {
        control_type_ = POSITION_CONTROL;
        position_service_ = pid_node_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(_name + "_position_control", boost::bind(&FreeFloatingPids::ToPositionControl, this, _1, _2));
        velocity_service_ = pid_node_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(_name + "_velocity_control", boost::bind(&FreeFloatingPids::ToVelocityControl, this, _1, _2));
    }

    void InitPID(control_toolbox::Pid& _pid, const ros::NodeHandle&_node, const bool &_use_dynamic_reconfig);

    // Basic PID's update: compute command from error, needed customized
    virtual bool UpdatePID() = 0;

protected:
    ros::NodeHandle pid_node_;
    ros::ServiceServer position_service_, velocity_service_;
    ros::Duration dt_;
    std::vector<pid_st> position_pids_;
    std::vector<pid_st> velocity_pids_;
    control_type control_type_;
    double error_filter;
    bool setpoint_received_;
    bool state_received_;

};

#endif // FREEFLOATINGPID_H
