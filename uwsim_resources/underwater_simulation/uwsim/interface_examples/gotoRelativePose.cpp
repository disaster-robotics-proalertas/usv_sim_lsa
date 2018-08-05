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


/* Move a vehicle with a velocity proportional to the distance 
 * between the current pose and a desired pose relative to the current one (given in the global coordinate system)
*/

#include <stdlib.h>
#include <string.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Matrix>

bool firstpass=false;
osg::Quat initialQ, goalQ, currentQ;
osg::Vec3d initialT, goalT, currentT;
double totalDistance, currentDistance;
double x, y, z, roll, pitch, yaw;

void vehiclePoseCallback(const nav_msgs::Odometry& odom) {
	if (!firstpass) {
		initialT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
		initialQ.set(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		osg::Matrixd transform;
		transform.setTrans(initialT);
		transform.setRotate(initialQ);

		osg::Matrixd T, Rx, Ry, Rz, transform_rel;
		T.makeTranslate(x,y,z);
		Rx.makeRotate(roll,1,0,0);
		Ry.makeRotate(pitch,0,1,0);
		Rz.makeRotate(yaw,0,0,1);
		transform_rel=Rz*Ry*Rx*T;
		transform=transform_rel*transform;
		goalQ=transform.getRotate();
		goalT=transform.getTrans();

		totalDistance=(goalT-initialT).length();
		firstpass=true;
	} 
	currentT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
	currentDistance=(goalT-currentT).length();
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "gotoRelativePose");
	ros::NodeHandle nh;

	if (argc!=9) {
		std::cerr << "USAGE: " << argv[0] << " <vehiclePoseTopic> <vehicleControlTopic> <rx> <ry> <rz> <rroll> <rpitch> <ryaw>" << std::endl;
		std::cerr << "units are meters and radians." << std::endl;
		return 0;
	}	

	std::string poseTopic(argv[1]);
	std::string controlTopic(argv[2]);
	x=atof(argv[3]);
	y=atof(argv[4]);
	z=atof(argv[5]);
	roll=atof(argv[6]);
	pitch=atof(argv[7]);
	yaw=atof(argv[8]);

	ros::Publisher position_pub=nh.advertise<nav_msgs::Odometry>(controlTopic,1);
	ros::Subscriber position_sub = nh.subscribe(poseTopic, 1, vehiclePoseCallback);

	ros::Rate r(30);
	while (ros::ok()) {
	   if (firstpass) {
		osg::Vec3d vT=(goalT-currentT)*0.15;
		double vScale=(vT.length()>0.05) ? 0.05/vT.length() : 1;
		currentQ.slerp(1-currentDistance/totalDistance,initialQ, goalQ);
		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=currentT.x()+vT.x()*vScale;
		odom.pose.pose.position.y=currentT.y()+vT.y()*vScale;
		odom.pose.pose.position.z=currentT.z()+vT.z()*vScale;
		odom.pose.pose.orientation.x=currentQ.x();
		odom.pose.pose.orientation.y=currentQ.y();
		odom.pose.pose.orientation.z=currentQ.z();
		odom.pose.pose.orientation.w=currentQ.w();

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
	   }
	   ros::spinOnce();
	   r.sleep();
	}

	return 0;
}
