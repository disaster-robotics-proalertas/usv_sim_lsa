/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */ 

#include <stdlib.h>
#include <string.h>

//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


int main(int argc, char **argv) {


	if (argc!=8) {
		std::cerr << "USAGE: " << argv[0] << " <topic> <vx> <vy> <vz> <vroll> <vpitch> <vyaw>" << std::endl;
		std::cerr << "units are displacement/simulated_time. Time scale to be implemented." << std::endl;
		return 0;
	}	

	std::string topic(argv[1]);
	double x=atof(argv[2]);
	double y=atof(argv[3]);
	double z=atof(argv[4]);
	double roll=atof(argv[5]);
	double pitch=atof(argv[6]);
	double yaw=atof(argv[7]);

        //std::string nodeName=topic;
        //nodeName.replace(0,1,"a");
	ros::init(argc, argv, "setVehicleVelocity");
	ros::NodeHandle nh;
	ros::Publisher position_pub;
	position_pub=nh.advertise<nav_msgs::Odometry>(topic,1);

	ros::Rate r(25);
	while (ros::ok()) {
		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=0.0;
		odom.pose.pose.position.y=0.0;
		odom.pose.pose.position.z=0.0;
		odom.pose.pose.orientation.x=0.0;
		odom.pose.pose.orientation.y=0.0;
		odom.pose.pose.orientation.z=0.0;
		odom.pose.pose.orientation.w=1;

		odom.twist.twist.linear.x=x;
		odom.twist.twist.linear.y=y;
		odom.twist.twist.linear.z=z;
		odom.twist.twist.angular.x=roll;
		odom.twist.twist.angular.y=pitch;
		odom.twist.twist.angular.z=yaw;
		for (int i=0; i<36; i++) {
			odom.twist.covariance[i]=0;
			odom.pose.covariance[i]=0;
		}
		position_pub.publish(odom);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
