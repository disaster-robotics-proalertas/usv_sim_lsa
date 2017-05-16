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

#ifndef GPSSENSOR_H_
#define GPSSENSOR_H_

#include <osg/Node>
#include <osg/Matrix>
#include <osg/Group>
#include <osg/Vec3d>

#include <boost/random.hpp>
#include "osgOceanScene.h"
#include <tf/transform_datatypes.h>

class GPSSensor
{

public:
  std::string name,parentLinkName;

  /** Constructor
   * @param name the name of the GPS sensor
   * @param parentName the name of the link that holds the GPS sensor
   * @param parent the node of the scene graph that holds the sensor
   * @param rMl the sensor measures are given with respect to the root (r). Use rMl to transform them to another frame ('l' is the new frame, typically the localized world)
   * @param std the standard deviation on the sensor measures
   */
  GPSSensor(osgOceanScene *oscene, std::string sensor_name, std::string parentName, osg::Node *parent, osg::Matrixd rMl, double std = 0) :
      name(sensor_name),parentLinkName(parentName), oscene_(oscene), parent_(parent), rMl_(rMl), std_(std)
  {
    node_ = new osg::Node();
    parent->asGroup()->addChild(node_);
  }

  osg::Vec3d getMeasurement();
  int getTFTransform(tf::Pose & pose, std::string & parent);

  double getStandardDeviation()
  {
    return std_;
  }

  /** Returns the depth of the sensor (in meters) under the water*/
  double depthBelowWater();

  virtual ~GPSSensor()
  {
  }

private:
  osg::ref_ptr<osgOceanScene> oscene_;
  osg::ref_ptr<osg::Node> parent_;
  osg::Matrixd rMl_;
  double std_;
  osg::ref_ptr<osg::Node> node_;

  boost::mt19937 rng_; ///< Boost random number generator
};

#endif /* GPSSENSOR_H_ */
