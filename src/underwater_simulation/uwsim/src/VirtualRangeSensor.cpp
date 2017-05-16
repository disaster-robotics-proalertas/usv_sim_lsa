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

#include <uwsim/VirtualRangeSensor.h>
#include <uwsim/UWSimUtils.h>
#include <iostream>

#include <osg/PositionAttitudeTransform>
VirtualRangeSensor::VirtualRangeSensor()
{
}

VirtualRangeSensor::VirtualRangeSensor(std::string name, std::string parentName, osg::Node *root, osg::Node *trackNode, double range,
                                       bool visible,unsigned int mask)
{
  init(name,parentName, root, trackNode, range, visible,mask);
}

void VirtualRangeSensor::init(std::string name, std::string parentName, osg::Node *root, osg::Node *trackNode, double range, bool visible,unsigned int mask)
{
  this->name = name;
  this->parentLinkName=parentName;
  this->root = root;

  this->trackNode = trackNode;
  //Add a switchable frame geometry on the sensor frame
  osg::ref_ptr < osg::Node > axis = UWSimGeometry::createSwitchableFrame();
  //Add label to switchable frame
  axis->asGroup()->addChild(UWSimGeometry::createLabel(name));
  this->trackNode->asGroup()->addChild(axis);

  this->range = range;
  this->visible = visible;

  //make this virtual ray track the node
  node_tracker = new IntersectorUpdateCallback(range, visible, root);
  trackNode->setUpdateCallback(node_tracker);
  trackNode->asGroup()->addChild(node_tracker->geode);

  if(node_tracker->geode)
    node_tracker->geode->setNodeMask(mask);
}

int VirtualRangeSensor::getTFTransform(tf::Pose & pose, std::string & parent){
  parent=parentLinkName;
  pose.setOrigin(tf::Vector3(trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().x(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().y(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().z()));
  pose.setRotation( tf::Quaternion(trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().x(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().y(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().z(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().w()));
  return 1;

}

