#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

nav_msgs::Odometry temp;

void getTransform(const nav_msgs::Odometry& msg){
    temp = msg;
}

void transformPoint(const tf::TransformListener& listener){
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "barco_auv/hokuyo_link";

    //we'll just use the most recent transform available for our simple example
    laser_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    laser_point.point.x = temp.pose.pose.position.x;
    laser_point.point.y = temp.pose.pose.position.y;
    laser_point.point.z = temp.pose.pose.position.z;

    try{
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", laser_point, base_point);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_listener");
    ros::NodeHandle n;

    tf::TransformListener listener(ros::Duration(10));
    ros::Subscriber sub = n.subscribe("laserScan", 1000, getTransform);

    //we'll transform a point once every second
    ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

    ros::spin();
}
