#include <freefloating_gazebo/butterworth.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

// subscribes to Float32MultiArray with (fc, f1,a1,..,f3,a3)
// publishes raw and filtered data

struct Freq
{
    double a, f;
};

std::vector<Freq> config(3);
Butterworth filter;
double fc = 10;
double dt = 0.001;    // 1000 Hz

void ReadFreq(const std_msgs::Float32MultiArrayConstPtr _msg)
{
    // update signal params
    for(int i = 0; i < 3; ++i)
    {
        config[i].f = _msg->data[1+2*i];
        config[i].a = _msg->data[2+2*i];
    }


    if(_msg->data[0] != fc)
    {
        fc = _msg->data[0];
        filter.init(fc, dt);
        std::cout << "Filtering @ " << fc << " Hz\n";
    }
}


using std::cout;
using std::endl;

int main(int argc, char ** argv)
{
    std::cout << "Filtering @ " << fc << " Hz\n";
    filter.init(fc, dt);

    ros::init(argc, argv, "butterWorth");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("butter", 1);
    sensor_msgs::JointState msg;
    msg.position.resize(2);

    ros::Subscriber sub = nh.subscribe("/freq", 1, ReadFreq);

    ros::Rate loop(1./dt);
    double x,t;

    while(ros::ok())
    {
        t = ros::Time::now().toSec();
        x = 0;
        for(auto c: config)
            x += c.a*cos(c.f*2*M_PI*t);
        msg.header.stamp = ros::Time::now();
        msg.position[0] = x;
        msg.position[1] = filter.filter(x);

        pub.publish(msg);
        ros::spinOnce();
        loop.sleep();
    }
}
