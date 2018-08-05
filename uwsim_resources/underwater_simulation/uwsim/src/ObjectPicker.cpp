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

#include <uwsim/ObjectPicker.h>
#include <uwsim/UWSimUtils.h>

ObjectPicker::ObjectPicker() :
    VirtualRangeSensor()
{
}

ObjectPicker::ObjectPicker(std::string name, osg::Node *root, osg::Node *trackNode, double range, bool visible,
                           boost::shared_ptr<URDFRobot> urdf,unsigned int mask)
{
  init(name, root, trackNode, range, visible, urdf, mask);
}

void ObjectPicker::init(std::string name, osg::Node *root, osg::Node *trackNode, double range, bool visible,
                        boost::shared_ptr<URDFRobot> urdf,unsigned int mask)
{
  this->name = name;
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
  node_tracker = new ObjectPickerUpdateCallback(trackNode, range, visible, root, urdf);
  trackNode->setUpdateCallback((ObjectPickerUpdateCallback*)(node_tracker.get()));
  trackNode->asGroup()->addChild(node_tracker->geode);

  if(node_tracker->geode)
    node_tracker->geode->setNodeMask(mask);
}

/*
 bool Hand::closeHand(){

 for(osg::NodePath::iterator i=node_tracker->impact.begin();i!=node_tracker->impact.end();++i){
 osg::ref_ptr<NodeDataType> data = dynamic_cast<NodeDataType*> (i[0]->getUserData());
 if(data!=NULL && data->catchable){  //Search for "catchable" objects in nodepath
 //Get coordinates to change them when changing position in graph
 osg::Matrixd *originalpos=getWorldCoords(i[0]);
 osg::Matrixd *hand = getWorldCoords(trackNode);
 hand->invert(*hand);

 //Turn object to Kinematic state, add callback to move collision shape
 if(data->rb)
 data->rb->setCollisionFlags( data->rb->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
 catched_callback=new BulletPhysics::MirrorTransformCallback(data->rb);
 i[0]->setUpdateCallback(catched_callback);

 //ADD node in hand, remove object from original position.
 trackNode->asTransform()->addChild(i[0]);
 i[0]->getParent(0)->asGroup()->removeChild(i[0]);
 osg::Matrixd matrix=*originalpos * *hand;
 i[0]->asTransform()->asMatrixTransform()->setMatrix(matrix);
 freeHand=0;
 catched=i[0];
 return 1;
 }

 }
 return 0;
 }


 void Hand::openHand(){
 osg::ref_ptr<NodeDataType> data = dynamic_cast<NodeDataType*> (catched->getUserData());

 osg::Matrixd *originalpos=getWorldCoords(catched);
 osg::Matrixd *world = getWorldCoords(root);
 world->invert(*world);

 root->asGroup()->addChild(catched);
 trackNode->asGroup()->removeChild(catched);
 osg::Matrixd matrix=*originalpos * *world;
 catched->asTransform()->asMatrixTransform()->setMatrix(matrix);

 //Get collision shape to current position, destroy callback and turn object to Dynamic state again.
 catched->removeUpdateCallback(catched_callback);
 catched_callback=NULL;
 data->rb->setWorldTransform( osgbCollision::asBtTransform(matrix) );
 if(data->rb)
 data->rb->setCollisionFlags( data->rb->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT );
 freeHand=1;
 catched=NULL;

 }
 */

