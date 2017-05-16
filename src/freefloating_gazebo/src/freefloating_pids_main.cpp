#include <freefloating_gazebo/freefloating_pids_body.h>
#include <freefloating_gazebo/freefloating_pids_joint.h>

using std::cout;
using std::endl;

int main(int argc, char ** argv)
{

    // init ROS node
    ros::init(argc, argv, "freefloating_pid_control");
    ros::NodeHandle rosnode;
    ros::NodeHandle control_node(rosnode, "controllers");

    // wait for body or joint param
    bool control_body = false;
    bool control_joints = false;
    while(!(control_body || control_joints))
    {
        sleep(5);
        control_body = control_node.hasParam("config/body");
        control_joints = control_node.hasParam("config/joints/name");
    }

    // recheck control_body against wrench vs thruster control (this basic PID can only do wrench)
    if(control_node.hasParam("config/body/control_type"))
    {
        std::string control_type;
        control_node.getParam("config/body/control_type", control_type);        
        if(control_type == "thruster")
            // check for a flag that tells us to use PID however
            control_body = control_node.param("config/body/use_pid", false);
    }
    // -- Parse body data if needed ---------

    // body setpoint and state topics
    std::string body_position_sp_topic, body_velocity_sp_topic, body_state_topic, body_command_topic;
    std::vector<std::string> controlled_axes;

    if(control_body)
    {
        control_node.param("config/body/position_setpoint", body_position_sp_topic, std::string("body_position_setpoint"));
        control_node.param("config/body/velocity_setpoint", body_velocity_sp_topic, std::string("body_velocity_setpoint"));
        control_node.param("config/body/state", body_state_topic, std::string("state"));
        control_node.param("config/body/command", body_command_topic, std::string("body_command"));
        // controlled body axes
        control_node.getParam("config/body/axes", controlled_axes);
    }

    // -- Parse joint data if needed ---------
    std::string joint_setpoint_topic, joint_state_topic, joint_command_topic;
    if(control_joints)
    {
        control_joints = true;
        // joint setpoint and state topics
        control_node.param("config/joints/setpoint", joint_setpoint_topic, std::string("joint_setpoint"));
        control_node.param("config/joints/state", joint_state_topic, std::string("joint_states"));
        control_node.param("config/joints/command", joint_command_topic, std::string("joint_command"));
    }
    // -- end parsing parameter server

    // loop rate
    ros::Rate loop(100);
    ros::Duration dt(.01);

    ros::SubscribeOptions ops;

    // -- Init body ------------------
    // PID's class
    FreeFloatingBodyPids body_pid;
    ros::Subscriber body_position_sp_subscriber, body_velocity_sp_subscriber, body_state_subscriber;
    ros::Publisher body_command_publisher;
    if(control_body)
    {
        body_pid.Init(control_node, dt, controlled_axes);

        // position setpoint
        body_position_sp_subscriber =
                rosnode.subscribe(body_position_sp_topic, 1, &FreeFloatingBodyPids::PositionSPCallBack, &body_pid);
        // velocity setpoint
        body_velocity_sp_subscriber =
                rosnode.subscribe(body_velocity_sp_topic, 1, &FreeFloatingBodyPids::VelocitySPCallBack, &body_pid);
        // measure
        body_state_subscriber =
                rosnode.subscribe(body_state_topic, 1, &FreeFloatingBodyPids::MeasureCallBack, &body_pid);
        // command
        body_command_publisher =
                rosnode.advertise<geometry_msgs::Wrench>(body_command_topic, 1);
    }

    // -- Init joints ------------------
    FreeFloatingJointPids joint_pid;
    // declare subscriber / publisher
    ros::Subscriber joint_setpoint_subscriber, joint_state_subscriber;
    ros::Publisher joint_command_publisher;

    if(control_joints)
    {
        // pid
        joint_pid.Init(control_node, dt);

        // setpoint
        joint_setpoint_subscriber = rosnode.subscribe(joint_setpoint_topic, 1, &FreeFloatingJointPids::SetpointCallBack, &joint_pid);

        // measure
        joint_state_subscriber = rosnode.subscribe(joint_state_topic, 1, &FreeFloatingJointPids::MeasureCallBack, &joint_pid);

        // command
        joint_command_publisher = rosnode.advertise<sensor_msgs::JointState>(joint_command_topic, 1);
    }

    std::vector<std::string> joint_names;
    if(control_joints)
        control_node.getParam("config/joints/name", joint_names);

    ROS_INFO("Init PID control for %s: %i body axes, %i joints", rosnode.getNamespace().c_str(), (int) controlled_axes.size(), (int) joint_names.size());

    while(ros::ok())
    {
        // update body and publish
        if(control_body)
            if(body_pid.UpdatePID())
                body_command_publisher.publish(body_pid.WrenchCommand());

        // update joints and publish
        if(control_joints)
            if(joint_pid.UpdatePID())
                joint_command_publisher.publish(joint_pid.EffortCommand());

        ros::spinOnce();
        loop.sleep();
    }
}
