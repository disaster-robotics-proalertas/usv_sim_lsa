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

/* Creates an interactive marker and moves the vehicle following it */
//Arguments:
// vehicle Topic

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <nav_msgs/Odometry.h>

//Class that holds the vehicle position publisher and the interactive marker feedback
class VehicleFollower
{
  private:
    ros::Publisher pos_pub;
    ros::NodeHandle n;
  public:
    VehicleFollower(std::string topic)
    {
      pos_pub = n.advertise<nav_msgs::Odometry>(topic, 1000);
    }

    //Interactive marker feedback class (calls the publisher)
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
      std::cout<< feedback->marker_name << " is now at "<< feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z << std::endl;

      std::cout<< feedback->marker_name << " orientation is " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y
        << ", " << feedback->pose.orientation.z  << ", " << feedback->pose.orientation.w <<std::endl;

      nav_msgs::Odometry odom;

      odom.pose.pose.position.x = feedback->pose.position.x;
      odom.pose.pose.position.y = feedback->pose.position.y;
      odom.pose.pose.position.z = feedback->pose.position.z;
      odom.pose.pose.orientation.x = feedback->pose.orientation.x;
      odom.pose.pose.orientation.y = feedback->pose.orientation.y;
      odom.pose.pose.orientation.z = feedback->pose.orientation.z;
      odom.pose.pose.orientation.w = feedback->pose.orientation.w;

      odom.twist.twist.linear.x = 0;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.linear.z = 0;
      odom.twist.twist.angular.x = 0;
      odom.twist.twist.angular.y = 0;
      odom.twist.twist.angular.z = 0;
      for (int i = 0; i < 36; i++)
      {
        odom.twist.covariance[i] = 0;
        odom.pose.covariance[i] = 0;
      }

      pos_pub.publish(odom);
    }
};


//Creates the marker and the control classes
int main(int argc, char** argv)
{
  ros::init(argc, argv, "followMarker");

  if (argc!=2)
  {
    std::cerr << "USAGE: " << argv[0] << " <VehicleTopic>" << std::endl;
    return 0;
  }	

  VehicleFollower follower((std::string)argv[1]);

  // create an interactive marker server on the topic namespace uwsim_marker
  interactive_markers::InteractiveMarkerServer server("uwsim_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/world";

  int_marker.name = "followMarker";
  //text orientation depends on world offset, so it's commented
  //int_marker.description = "Vehicle following marker";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.15;
  box_marker.scale.y = 0.15;
  box_marker.scale.z = 0.15;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control to hold the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = false;
  box_control.markers.push_back( box_marker );

  // add the 6DOF control to the interactive marker
  int_marker.controls.push_back( box_control );

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker,boost::bind(&VehicleFollower::processFeedback, &follower, _1) );

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}

