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

#include <osg/Quat>
#include <osg/Vec3d>

int main(int argc, char **argv) {


	if (argc!=8) {
		std::cerr << "USAGE: " << argv[0] << " <topic> <x> <y> <z> <yaw> <long> <step>" << std::endl;
		std::cerr << "units in meters and radians" << std::endl;
		return 0;
	}	

	std::string topic(argv[1]);
	double x=atof(argv[2]);
	double y=atof(argv[3]);
	double z=atof(argv[4]);
	// TODO FIXME yaw is not used below!
	//double yaw=atof(argv[5]);
	double amp=atof(argv[6]);
	double step=atof(argv[7]);

        std::string nodeName=topic;	
	nodeName=nodeName.replace(0,1,"a");
	std::cerr << "NodeName: " << nodeName << std::endl;
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;

	ros::Publisher position_pub;
	position_pub=nh.advertise<nav_msgs::Odometry>(topic,1);

	double angle=0;
	ros::Rate r(20);
	while (ros::ok()) {
		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=x+amp*sin(angle);
		odom.pose.pose.position.y=y+angle*step;
		odom.pose.pose.position.z=z;

		double rz=0;
		if (step>0) {
			rz=M_PI_2-M_PI_2*cos(angle);
		} else {
			rz=-M_PI_2+M_PI_2*cos(angle);
		}
		osg::Quat rot(0, osg::Vec3d(1,0,0), 0, osg::Vec3d(0,1,0), rz, osg::Vec3d(0,0,1));
		odom.pose.pose.orientation.x=rot.x();
		odom.pose.pose.orientation.y=rot.y();
		odom.pose.pose.orientation.z=rot.z();
		odom.pose.pose.orientation.w=rot.w();

		odom.twist.twist.linear.x=0;
		odom.twist.twist.linear.y=0;
		odom.twist.twist.linear.z=0;
		odom.twist.twist.angular.x=0;
		odom.twist.twist.angular.y=0;
		odom.twist.twist.angular.z=0;
		for (int i=0; i<36; i++) {
			odom.twist.covariance[i]=0;
			odom.pose.covariance[i]=0;
		}
		position_pub.publish(odom);

		ros::spinOnce();
		r.sleep();
		angle+=0.01;
	}

	return 0;
}
