#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void getTransform(const nav_msgs::Odometry::ConstPtr& msg){
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w), tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)),
        ros::Time::now(), "world", "base_link"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "world_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);
    ros::Subscriber sub = n.subscribe("state", 100, getTransform);

    while(n.ok()){
        ros::spinOnce();
        r.sleep();
    }
}
