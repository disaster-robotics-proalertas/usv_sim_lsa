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
#include <geometry_msgs/Pose.h>
#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Matrix>


int main(int argc, char **argv) {

	ros::init(argc, argv, "setVehiclePose");
	ros::NodeHandle nh;

	if (argc!=8) {
		std::cerr << "USAGE: " << argv[0] << " <topic> <x> <y> <z> <roll> <pitch> <yaw>" << std::endl;
		std::cerr << "units are in meters and radians." << std::endl;
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
	position_pub=nh.advertise<geometry_msgs::Pose>(topic,1);


	osg::Matrixd T, Rx, Ry, Rz, transform;
	T.makeTranslate(x,y,z);
	Rx.makeRotate(roll,1,0,0);
	Ry.makeRotate(pitch,0,1,0);
	Rz.makeRotate(yaw,0,0,1);
	transform=Rz*Ry*Rx*T;
	osg::Vec3d trans=transform.getTrans();
	osg::Quat rot=transform.getRotate();


	ros::Rate r(25);
	while (ros::ok()) {
		geometry_msgs::Pose pose;

		pose.position.x=trans.x();
		pose.position.y=trans.y();
		pose.position.z=trans.z();
		pose.orientation.x=rot.x();
		pose.orientation.y=rot.y();
		pose.orientation.z=rot.z();
		pose.orientation.w=rot.w();
		
		position_pub.publish(pose);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
