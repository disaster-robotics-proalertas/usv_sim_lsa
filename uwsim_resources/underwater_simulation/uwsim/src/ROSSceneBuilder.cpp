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


#include <uwsim/ROSSceneBuilder.h>

#include <osg_markers/marker_base.h>

#include <osg_markers/shape_marker.h>
#include <osg_markers/arrow_marker.h>
#include <osg_markers/text_view_facing_marker.h>
#include <osg_markers/triangle_list_marker.h>
#include <osg_markers/mesh_resource_marker.h>

ROSSceneBuilder::ROSSceneBuilder(boost::shared_ptr<osg::ArgumentParser> args)
:SceneBuilder(args)
{

}

bool ROSSceneBuilder::loadScene(ConfigFile config)
{
  SceneBuilder::loadScene(config);

  markerService = n.advertiseService("SpawnMarker", &ROSSceneBuilder::markerSRVCallback, this);
  ROS_INFO("Created spawnMarker service on SpawnMarker.");

  //Create a node that will hold the markers
  markers=new osg::Group();
  scene->localizedWorld->addChild(markers);
  markers->setNodeMask(scene->getOceanScene()->getNormalSceneMask() | scene->getOceanScene()->getReflectedSceneMask() | scene->getOceanScene()->getRefractedSceneMask());

  //Create a marker frame manager, and interactive marker client
  frame_manager = osg_utils::FrameManager::instance();
  frame_manager->setFixedFrame("/world");
  marker_cli= (boost::shared_ptr<osg_interactive_markers::InteractiveMarkerDisplay>)
	 new osg_interactive_markers::InteractiveMarkerDisplay("osg_im","/uwsim_marker/update", markers, *(frame_manager->getTFClient()));

  //Start time elapsed vars for interactive marker client
  last_wall_time = ros::WallTime::now();
  last_ros_time = ros::Time::now();

  ROS_INFO("Created interactive marke service on uwsim_marker.");
}

void ROSSceneBuilder::updateIM()
{
    ros::WallTime current_wall_time=ros::WallTime::now();
    ros::Time current_ros_time=ros::Time::now();
    marker_cli->update((current_wall_time-last_wall_time).toSec(), (current_ros_time-last_ros_time).toSec());
    last_wall_time=current_wall_time;
    last_ros_time=current_ros_time;
}

bool  ROSSceneBuilder::markerSRVCallback(underwater_sensor_msgs::SpawnMarker::Request  &req, underwater_sensor_msgs::SpawnMarker::Response &res)
{
  boost::shared_ptr<osg_markers::MarkerBase> markerSearch;
  //Search for marker in markerList
  for (MarkerList::iterator iter = markerList.begin(); iter != markerList.end(); ++iter)
  {
    markerSearch = *iter;
    if(markerSearch->scene_node_->getName()==boost::lexical_cast<std::string>(req.marker.id) + "+" + req.marker.ns )
    { 
      if(req.marker.action==visualization_msgs::Marker::MODIFY)
      { // If marker exists and action is modify update the message
        markerSearch->setMessage(req.marker);
        res.success=true;
        res.status_message="object " + boost::lexical_cast<std::string>(req.marker.id) + " " + req.marker.ns + " was successfully MODIFIED.";
        return true;
      }
      else if (req.marker.action==visualization_msgs::Marker::DELETE)
      { //If marker exists and action is delete, delete it from object array and marker List
	for(unsigned int i=0; i<objects.size(); i++)
        {
	  if(objects[i]->getName()==boost::lexical_cast<std::string>(req.marker.id) + "+" + req.marker.ns )
	    objects.erase(objects.begin()+i);
        }
	markerList.erase(iter);
	scene->localizedWorld->removeChild(markerSearch->scene_node_->getParent(0));
        markerSearch->scene_node_=NULL;
        res.success=true;
        res.status_message="object " + boost::lexical_cast<std::string>(req.marker.id) + " " + req.marker.ns + " was successfully DELETED.";
	return true;
      }
    }
  }

  if (req.marker.action==visualization_msgs::Marker::DELETE)
  {
    res.success=false;
    res.status_message="object " + boost::lexical_cast<std::string>(req.marker.id) + " " + req.marker.ns + " not found.";
    return true; 
  }

  boost::shared_ptr<osg_markers::MarkerBase> marker;
  osg::MatrixTransform * parent = new osg::MatrixTransform();

  switch (req.marker.type)
  {
    case visualization_msgs::Marker::CUBE:
    case visualization_msgs::Marker::CYLINDER:
    case visualization_msgs::Marker::SPHERE:
    {
      marker.reset(new osg_markers::ShapeMarker(parent));
    }
    break;

    case visualization_msgs::Marker::ARROW:
    {
      marker.reset(new osg_markers::ArrowMarker(parent));
    }
    break;

    case visualization_msgs::Marker::LINE_STRIP:
    {
      ROS_WARN("InteractiveMarkerControl::InteractiveMarkerControl LineStripMarker to be implemented");
    }
    break;

    case visualization_msgs::Marker::LINE_LIST:
    {
      ROS_WARN("InteractiveMarkerControl::InteractiveMarkerControl LineListMarker to be implemented");
    }
    break;

    case visualization_msgs::Marker::SPHERE_LIST:
    case visualization_msgs::Marker::CUBE_LIST:
    case visualization_msgs::Marker::POINTS:
    {
      ROS_WARN("InteractiveMarkerControl::InteractiveMarkerControl PointsMarker to be implemented TODO");
    }
    break;

    case visualization_msgs::Marker::TEXT_VIEW_FACING:
    {
      marker.reset(new osg_markers::TextViewFacingMarker(parent));
    }
    break;
 
    case visualization_msgs::Marker::MESH_RESOURCE:
    {
      marker.reset(new osg_markers::MeshResourceMarker(parent));
    }
    break;

    case visualization_msgs::Marker::TRIANGLE_LIST:
    {
      marker.reset(new osg_markers::TriangleListMarker(parent));
    }
    break;

    default:
      ROS_ERROR( "Unknown marker type: %d", req.marker.type );
  }

  scene->localizedWorld->addChild(parent);
  marker->setMessage(req.marker);

  marker->scene_node_->setNodeMask(scene->getOceanScene()->getNormalSceneMask() | scene->getOceanScene()->getReflectedSceneMask() | scene->getOceanScene()->getRefractedSceneMask());

  markerList.push_back(marker);
  marker->scene_node_->setName(boost::lexical_cast<std::string>(req.marker.id) + "+" + req.marker.ns);

  objects.push_back(marker->scene_node_);

  res.success=true;
  res.status_message="object " + boost::lexical_cast<std::string>(req.marker.id) + " " + req.marker.ns + " was successfully ADDED.";

  return true;
}
