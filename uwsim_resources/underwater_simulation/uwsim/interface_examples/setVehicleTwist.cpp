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
#include <geometry_msgs/TwistStamped.h>


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

        std::string nodeName=topic;
        nodeName.replace(0,1,"a");
	ros::init(argc, argv, "setVehicleVelocity");
	ros::NodeHandle nh;
	ros::Publisher position_pub;
	position_pub=nh.advertise<geometry_msgs::TwistStamped>(topic,1);

	ros::Rate r(25);
	while (ros::ok()) {
		geometry_msgs::TwistStamped twist;

		twist.twist.linear.x=x;
		twist.twist.linear.y=y;
		twist.twist.linear.z=z;
		twist.twist.angular.x=roll;
		twist.twist.angular.y=pitch;
		twist.twist.angular.z=yaw;

		position_pub.publish(twist);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
