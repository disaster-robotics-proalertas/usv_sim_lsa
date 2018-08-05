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
 * between the current pose and a desired absolute pose 
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

void vehiclePoseCallback(const nav_msgs::Odometry& odom) {
	if (!firstpass) {
		initialT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
		initialQ.set(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		totalDistance=(goalT-initialT).length();
		firstpass=true;
	} 
	currentT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
	currentDistance=(goalT-currentT).length();
}

int main(int argc, char **argv) {


	if (argc!=9) {
		std::cerr << "USAGE: " << argv[0] << " <vehiclePoseTopic> <vehicleControlTopic> <x> <y> <z> <roll> <pitch> <yaw>" << std::endl;
		std::cerr << "units are meters and radians." << std::endl;
		return 0;
	}	

	std::string poseTopic(argv[1]);
	std::string controlTopic(argv[2]);
	double x=atof(argv[3]);
	double y=atof(argv[4]);
	double z=atof(argv[5]);
	double roll=atof(argv[6]);
	double pitch=atof(argv[7]);
	double yaw=atof(argv[8]);
	
	std::string nodeName=controlTopic;
	nodeName.replace(0,1,"a");
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;

	osg::Matrixd T, Rx, Ry, Rz, transform;
	T.makeTranslate(x,y,z);
	Rx.makeRotate(roll,1,0,0);
	Ry.makeRotate(pitch,0,1,0);
	Rz.makeRotate(yaw,0,0,1);
	transform=Rz*Ry*Rx*T;
	goalT=transform.getTrans();
	goalQ=transform.getRotate();

	ros::Publisher position_pub=nh.advertise<nav_msgs::Odometry>(controlTopic,1);
	ros::Subscriber position_sub = nh.subscribe(poseTopic, 1, vehiclePoseCallback);

	ros::Rate r(30);
	while (ros::ok()) {
	   if (firstpass) {
		osg::Vec3d vT=(goalT-currentT)*0.15;
		double vScale=(vT.length()>0.1) ? 0.1/vT.length() : 1;
/*
		std::cerr << "Initial Q: " << initialQ.x() << " " << initialQ.y() << " " << initialQ.z() << " " << initialQ.w() << std::endl;
		std::cerr << "Goal Q: " << goalQ.x() << " " << goalQ.y() << " " << goalQ.z() << " " << goalQ.w() << std::endl;
		std::cerr << "Goal Q: " << currentQ.x() << " " << currentQ.y() << " " << currentQ.z() << " " << currentQ.w() << std::endl;
		std::cerr << "Goal T: " << goalT.x() << " " << goalT.y() << " " << goalT.z() << std::endl;
		std::cerr << "Current T: " << currentT.x() << " " << currentT.y() << " " << currentT.z() << std::endl;
		std::cerr << "Goal-Current" << (goalT-currentT).x() << " " << (goalT-currentT).y() << " " << (goalT-currentT).z() << std::endl;
		std::cerr << "Current dist: " << currentDistance << " Total: " << totalDistance << std::endl;
		std::cerr << "current/total: " << currentDistance/totalDistance << std::endl;
		std::cerr << "Slerp: " << 1 - currentDistance/totalDistance << std::endl;
		std::cerr << std::endl;
*/
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
