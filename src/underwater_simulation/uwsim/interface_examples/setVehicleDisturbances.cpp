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
//OSG
#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Matrix>

bool firstpass=false;
osg::Quat initialQ;
osg::Vec3d initialT;
osg::Matrix wMv_initial;

void vehiclePoseCallback(const nav_msgs::Odometry& odom) {
	if (!firstpass) {
		initialT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
		initialQ.set(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		wMv_initial.setTrans(initialT);
		wMv_initial.setRotate(initialQ);
		firstpass=true;
	} 
}

int main(int argc, char **argv) {

	if (argc!=9) {
		std::cerr << "USAGE: " << argv[0] << " <vehiclePoseTopic> <controlTopic> <ax> <ay> <az> <aroll> <apitch> <ayaw>" << std::endl;
		std::cerr << "<ax> <ay> <az> <ayaw> is the maximum desired variation on the vehicle pose" << std::endl;
 		std::cerr << "units in meters and radians" << std::endl;
 		std::cerr << "Example: " << argv[0] << "/dataNavigator 0.2 0.2 0.2 0.2" << std::endl;
		return 0;
	}	

	std::string poseTopic(argv[1]);
	std::string controlTopic(argv[2]);
	double ax=atof(argv[3]);
	double ay=atof(argv[4]);
	double az=atof(argv[5]);
	double aroll=atof(argv[6]);
	double apitch=atof(argv[7]);
	double ayaw=atof(argv[8]);

	std::string nodeName=controlTopic;
	nodeName.replace(0,1,"a");
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;
	ros::Publisher position_pub;
	position_pub=nh.advertise<nav_msgs::Odometry>(controlTopic,1);
	ros::Subscriber position_sub = nh.subscribe(poseTopic, 1, vehiclePoseCallback);

	double angle=0;
	ros::Rate r(25);
	while (ros::ok()) {
	   if (firstpass) {
		osg::Matrixd T, Rx, Ry, Rz, transform;
		T.makeTranslate(ax*sin(2*angle),ay*sin(angle),az*sin(angle));
		Rx.makeRotate(aroll*sin(angle),1,0,0);
		Ry.makeRotate(apitch*sin(angle),0,1,0);
		Rz.makeRotate(ayaw*sin(angle),0,0,1);
		transform=Rz*Ry*Rx*T*wMv_initial;
		
		osg::Vec3d currentT=transform.getTrans();
		osg::Quat currentQ=transform.getRotate();

		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=currentT.x();
		odom.pose.pose.position.y=currentT.y();
		odom.pose.pose.position.z=currentT.z();
		odom.pose.pose.orientation.x=currentQ.x();
		odom.pose.pose.orientation.y=currentQ.y();
		odom.pose.pose.orientation.z=currentQ.z();
		odom.pose.pose.orientation.w=currentQ.w();
		angle+=0.01;

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
