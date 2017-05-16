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

/** Sends a control signal to the vehicle actuators, needs the dynamics module running
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>

int main(int argc, char **argv) {
	if (argc < 3) {
                std::cerr << "Usage: " << argv[0] << "<topic> <u1> <u2> ... <un>" << std::endl;
                exit(0);
        }
	std::string topic(argv[1]);

	ros::init(argc, argv, "setVehicleActuators");
	ros::NodeHandle nh;

	ros::Publisher u_pub;
	u_pub=nh.advertise<std_msgs::Float64MultiArray>(topic,1);
	ros::Rate rate(5);

	double u[argc-2];
	for (int i=2; i<argc; i++) u[i-2]=atof(argv[i]);

	while (ros::ok()) {
		std_msgs::Float64MultiArray um;

		for (int i=0; i<argc-2; i++)
			um.data.push_back(u[i]);

        	u_pub.publish(um);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
