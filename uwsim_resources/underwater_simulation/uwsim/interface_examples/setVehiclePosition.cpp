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
#include <osg/Matrix>

int main(int argc, char **argv) {

	ros::init(argc, argv, "setVehiclePosition");
	ros::NodeHandle nh;

	if (argc!=8) {
		std::cerr << "USAGE: " << argv[0] << " <topic> <x> <y> <z> <roll> <pitch> <yaw>" << std::endl;
		std::cerr << "units in meters and radians" << std::endl;
		return 0;
	}	

	std::string topic(argv[1]);
	double x=atof(argv[2]);
	double y=atof(argv[3]);
	double z=atof(argv[4]);
	double roll=atof(argv[5]);
	double pitch=atof(argv[6]);
	double yaw=atof(argv[7]);

	ros::Publisher position_pub;
	position_pub=nh.advertise<nav_msgs::Odometry>(topic,1);

	osg::Matrixd T, Rx, Ry, Rz, transform;
	T.makeTranslate(x,y,z);
	Rx.makeRotate(roll,1,0,0);
	Ry.makeRotate(pitch,0,1,0);
	Rz.makeRotate(yaw,0,0,1);
	transform=Rz*Ry*Rx*T;
	osg::Vec3d trans=transform.getTrans();
	osg::Quat rot=transform.getRotate();

	ros::Rate r(10);
	while (ros::ok()) {
		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=trans.x();
		odom.pose.pose.position.y=trans.y();
		odom.pose.pose.position.z=trans.z();
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
	}

	return 0;
}
