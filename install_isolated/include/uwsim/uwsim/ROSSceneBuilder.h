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


#ifndef ROSSCENEBUILDER_H_
#define ROSSCENEBUILDER_H_

#include <uwsim/SceneBuilder.h>

#include <underwater_sensor_msgs/SpawnMarker.h>

#include <osg_interactive_markers/interactive_marker_display.h>
#include <osg_utils/osg_utils.h>
#include <osg_utils/frame_manager.h>


//This class extends the SceneBuilder with ROS interfaces to modify and interact with the scene geometry.

class ROSSceneBuilder : public SceneBuilder
{

public:

  ROSSceneBuilder(boost::shared_ptr<osg::ArgumentParser> args);
  bool loadScene(ConfigFile config);

  void updateIM();

private:

  ros::NodeHandle n;
  ros::ServiceServer markerService;
  osg::Group * markers;
  boost::shared_ptr<osg_utils::FrameManager> frame_manager;
  boost::shared_ptr<osg_interactive_markers::InteractiveMarkerDisplay> marker_cli;

  ros::WallTime last_wall_time;
  ros::Time last_ros_time;

  typedef std::list< boost::shared_ptr<osg_markers::MarkerBase> > MarkerList;
  MarkerList markerList;

  //This callback will receibe marker requests and create/modify/delete objects in the scene.
  bool markerSRVCallback(underwater_sensor_msgs::SpawnMarker::Request  &req, underwater_sensor_msgs::SpawnMarker::Response &res);


};

#endif
