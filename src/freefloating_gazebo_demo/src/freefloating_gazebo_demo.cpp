
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_srvs/Empty.h>


using std::cout;
using std::endl;

class Robot
{
protected:
    void ReadArmState(const sensor_msgs::JointState::ConstPtr& _msg)
    {
        joint_states_ = *_msg;
    }
    void ReadBodyState(const nav_msgs::OdometryConstPtr& _msg)
    {
        body_state_ = *_msg;
    }

public:
    Robot(ros::NodeHandle &_ros_node)
    {
        // init publishers
        arm_publisher_ = _ros_node.advertise<sensor_msgs::JointState>("g500arm5e/joint_setpoint", 1);
        body_publisher_ = _ros_node.advertise<geometry_msgs::PoseStamped>("g500arm5e/body_position_setpoint", 1);
        joint_setpoint_.name.resize(3);
        joint_setpoint_.name[0] = "Slew";
        joint_setpoint_.name[1] = "Shoulder";
        joint_setpoint_.name[2] = "Elbow";
        joint_setpoint_.position.resize(3);

        arm_has_setpoint_ = body_has_setpoint_ = false;

        // init subscribers
        arm_subscriber_ = _ros_node.subscribe("g500arm5e/joint_states", 1, &Robot::ReadArmState, this);
        body_subscriber_ = _ros_node.subscribe("g500arm5e/state", 1, &Robot::ReadBodyState, this);

    }

    void MoveArm(const double &_q1, const double &_q2, const double &_q3)
    {
        // write message
        joint_setpoint_.position[0] = _q1;
        joint_setpoint_.position[1] = _q2;
        joint_setpoint_.position[2] = _q3;

        arm_has_setpoint_ = true;
    }

    void MoveBody(const double &_x, const double &_y, const double &_z, const double &_qx, const double &_qy, const double &_qz, const double &_qw)
    {
        // write message
        body_setpoint_.header.stamp = ros::Time::now();
        body_setpoint_.pose.position.x = _x;
        body_setpoint_.pose.position.y = _y;
        body_setpoint_.pose.position.z = _z;
        body_setpoint_.pose.orientation.x = _qx;
        body_setpoint_.pose.orientation.y = _qy;
        body_setpoint_.pose.orientation.z = _qz;
        body_setpoint_.pose.orientation.w = _qw;

        body_has_setpoint_ = true;
    }

    void Publish()
    {
        // publish if needed
        if(arm_has_setpoint_)
            arm_publisher_.publish(joint_setpoint_);
        if(body_has_setpoint_)
            body_publisher_.publish(body_setpoint_);
    }

protected:
    // publishers and messages to be published
    ros::Publisher arm_publisher_, body_publisher_;
    sensor_msgs::JointState joint_setpoint_;
    geometry_msgs::PoseStamped body_setpoint_;
    bool arm_has_setpoint_, body_has_setpoint_;

    // subscribers and received messages
    ros::Subscriber arm_subscriber_, body_subscriber_;
    sensor_msgs::JointState joint_states_;
    nav_msgs::Odometry body_state_;

};

typedef enum
{
    MISSION_START, ARM_CONTROL, BODY_APPROACH, ARM_GRASP, BODY_UP
} mission_state;

int main(int argc, char ** argv)
{
    // this node
    ros::init(argc, argv, "main_control");
    ros::start();
    ros::NodeHandle ros_node;
    const double rate = 30;
    ros::Rate loop(rate);

    // mission state
    mission_state state = MISSION_START;
    double state_wait = 0;

    // water current
    geometry_msgs::Vector3 current;
    current.y = .1;
    ros::Publisher current_publisher = ros_node.advertise<geometry_msgs::Vector3>("/gazebo/current", 0);

    // Robot class
    Robot robot(ros_node);

    // ensure physics are running
    std_srvs::Empty srv;
    ros::ServiceClient client = ros_node.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    cout << "Unpausing physics" << endl;
    client.call(srv);


    // main loop
    while(ros::ok())
    {
        cout << "-----------" << endl;
        switch(state)
        {
        case MISSION_START:
            state = ARM_CONTROL;
            robot.MoveArm(0, .56, 1.1);
            cout << "Setting water current" << endl << "Setting arm in neutral position" << endl;
            state_wait = 3;
            break;
        case ARM_CONTROL:
            state = BODY_APPROACH;
            robot.MoveBody(5.3,6.7,-0.3,1,1,0,0);
            cout << "Approaching body" << endl;
            state_wait = 15;
            break;
        case BODY_APPROACH:
            state = ARM_GRASP;
            cout << "Preparing to grasp" << endl;
//            robot.MoveBody(5.05,6.7,-11.4,1,1,0,0);
            //robot.MoveArm(0, .73, 1.31);
            state_wait = 10;
            break;
        case ARM_GRASP:
            state = BODY_UP;
            cout << "Going up" << endl;
//            robot.MoveBody(5.3, 6.7, -10, 1, 1, 0, 0);
          //  robot.MoveArm(0, 1.3, 1.5);
            state_wait = 1;
        default:
            loop.sleep();
        }

        for(unsigned int i=0; (i< (unsigned int) state_wait * rate) && ros::ok();++i)
        {
            current_publisher.publish(current);
            robot.Publish();
            ros::spinOnce();
            loop.sleep();
        }

        if(state == BODY_UP)
            ros::shutdown();
    }
}
