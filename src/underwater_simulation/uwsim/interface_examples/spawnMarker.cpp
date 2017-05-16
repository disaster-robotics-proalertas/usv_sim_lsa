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

/* Spawns / deletes a Mesh marker in the uwsim scene through a marker service */
//Arguments:
// action (add/modify, del), name, id, mesh, x ,y ,z , r, p, y

#include <ros/ros.h>
#include <underwater_sensor_msgs/SpawnMarker.h>
#include <tf/transform_datatypes.h>



int main( int argc, char** argv )
{
  ros::init(argc, argv, "spawnMarker");
  ros::NodeHandle n;

  if (argc!=11)
  {
    std::cerr << "USAGE: " << argv[0] << " <action> <name> <id> <mesh> <x> <y> <z> <roll> <pitch> <yaw>" << std::endl;
    std::cerr << "units in meters and radians" << std::endl;
    return 0;
  }	

  ros::ServiceClient client = n.serviceClient<underwater_sensor_msgs::SpawnMarker>("SpawnMarker");
  underwater_sensor_msgs::SpawnMarker srv;

  // UWSim does not use frame_id (TODO), it will add the mark in localizedWorld.
  srv.request.marker.header.frame_id = "/localizedWorld";
  srv.request.marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  srv.request.marker.ns = (std::string) argv[2];
  srv.request.marker.id = atoi(argv[3]);

  // Set the marker type.
  srv.request.marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  srv.request.marker.mesh_resource="package://uwsim/" + (std::string) argv[4];

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  if((std::string)argv[1]=="add")
    srv.request.marker.action = visualization_msgs::Marker::ADD;
  else if ((std::string)argv[1]=="del")
    srv.request.marker.action = visualization_msgs::Marker::DELETE;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  srv.request.marker.pose.position.x = atof(argv[5]);
  srv.request.marker.pose.position.y = atof(argv[6]);
  srv.request.marker.pose.position.z = atof(argv[7]);

  tf::Quaternion quat;
  quat.setRPY( atof(argv[8]),atof(argv[9]),atof(argv[10]));
  quat.normalize();

  srv.request.marker.pose.orientation.x = quat.getX();
  srv.request.marker.pose.orientation.y = quat.getY();
  srv.request.marker.pose.orientation.z = quat.getZ();
  srv.request.marker.pose.orientation.w = quat.getW();
	
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  srv.request.marker.scale.x = 1.0;
  srv.request.marker.scale.y = 1.0;
  srv.request.marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  srv.request.marker.color.r = 1.0f;
  srv.request.marker.color.g = 1.0f;
  srv.request.marker.color.b = 1.0f;
  srv.request.marker.color.a = 1.0;

  if (client.call(srv))
  {
    ROS_INFO("Operation exited with %d", srv.response.success);
    std::cout<<srv.response.status_message<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service SpawnMarker (is uwsim running?)");
    return 1;
  }

  return 0;
}
