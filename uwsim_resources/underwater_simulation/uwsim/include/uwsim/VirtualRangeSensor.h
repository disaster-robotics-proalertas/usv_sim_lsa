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

#ifndef VirtualRangeSensor_H_
#define VirtualRangeSensor_H_

#include "SimulatorConfig.h"
#include "CustomWidget.h"
#include "ConfigXMLParser.h"

#include <osgViewer/Viewer>
#include <osgGA/NodeTrackerManipulator>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osgGA/GUIEventHandler>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/NodeTrackerCallback>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#include <tf/transform_datatypes.h>

//Node tracker that updates the ray coordinates from the tracked node position and computes intersections
class IntersectorUpdateCallback : public osg::NodeTrackerCallback
{
  virtual void operator()(osg::Node *node, osg::NodeVisitor *nv)
  {
    osg::Matrixd mStart, mEnd;
    mStart = osg::computeLocalToWorld(nv->getNodePath());
    traverse(node, nv);

    //osg::Timer_t startTick = osg::Timer::instance()->tick();

    //update ray and compute intersections. Checks intersections along X axis of the local frame
    mEnd = mStart;
    mEnd.preMultTranslate(osg::Vec3d(range, 0, 0));

    //std::cerr << "mStart: " << mStart.getTrans().x() << " " << mStart.getTrans().y() << " " << mStart.getTrans().z() << std::endl;
    //std::cerr << "mEnd: " << mEnd.getTrans().x() << " " << mEnd.getTrans().y() << " " << mEnd.getTrans().z() << std::endl;
    intersector->reset();
    intersector->setStart(mStart.getTrans());
    intersector->setEnd(mEnd.getTrans());

    root->accept(intersectVisitor);

    //osg::Timer_t endTick = osg::Timer::instance()->tick();
    //std::cout<<"Completed in "<<osg::Timer::instance()->delta_s(startTick,endTick)<<std::endl;

    if (intersector->containsIntersections())
    {
      osgUtil::LineSegmentIntersector::Intersection intersection = intersector->getFirstIntersection();
      osg::Vec3d worldIntPoint = intersection.getWorldIntersectPoint();
      //osg::Vec3d localIntPoint=intersection.getLocalIntersectPoint();
      //std::cerr << "Intersection point(world): " << worldIntPoint.x() << " " << worldIntPoint.y() << " " << worldIntPoint.z() << std::endl;
      //std::cerr << "Intersection point(local): " << localIntPoint.x() << " " << localIntPoint.y() << " " << worldIntPoint.z() << std::endl;
      distance_to_obstacle = (worldIntPoint - mStart.getTrans()).length();
      //std::cerr << "Distance to obstacle: " << distance_to_obstacle << std::endl;
    }
    else
      distance_to_obstacle = range;
  }
public:
  double range, distance_to_obstacle;
  osg::ref_ptr<osg::Node> root;
  osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
  osgUtil::IntersectionVisitor intersectVisitor;

  osg::ref_ptr<osg::Geode> geode; //Geometry node that draws the beam
  osg::ref_ptr<osg::Geometry> beam;

  IntersectorUpdateCallback(double range, bool visible, osg::Node *root)
  {
    this->range = range;
    this->distance_to_obstacle = range;
    this->root = root;
    intersector = new osgUtil::LineSegmentIntersector(osg::Vec3d(0, 0, 0), osg::Vec3d(0, 0, 0));
    intersectVisitor.setIntersector(intersector.get());

    if (visible)
    {
      beam = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
      osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
      osg::Vec3d start(0, 0, 0);
      osg::Vec3d end(range, 0, 0);
      points->push_back(start);
      points->push_back(end);
      osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
      color->push_back(osg::Vec4(0.0, 1.0, 0.0, 0.6));
      beam->setVertexArray(points.get());
      beam->setColorArray(color.get());
      beam->setColorBinding(osg::Geometry::BIND_OVERALL);
      beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
      geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
      geode->addDrawable(beam.get());
    }
  }
};

/**  Virtual range sensor that computes the distance to an obstacle along a given direction */
class VirtualRangeSensor
{
public:
  std::string name, parentLinkName;
  osg::ref_ptr<osg::Node> trackNode;
  osg::ref_ptr<osg::Node> root;
  double range; ///< Max distance of the beam
  bool visible; ///< Whether to make the beam visible or not
  osg::ref_ptr<IntersectorUpdateCallback> node_tracker;

  VirtualRangeSensor(std::string name, std::string parentName, osg::Node *root, osg::Node *trackNode, double range, bool visible,unsigned int mask);
  VirtualRangeSensor();
  int getTFTransform(tf::Pose & pose, std::string & parent);

  virtual void init(std::string name, std::string parentName, osg::Node *root, osg::Node *trackNode, double range, bool visible, unsigned int mask);

};

#endif /* VirtualRangeSensor_H_ */
