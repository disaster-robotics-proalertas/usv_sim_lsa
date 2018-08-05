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

#ifndef INERTIALMEASUREMENTUNIT_H_
#define INERTIALMEASUREMENTUNIT_H_

#include <osg/Vec3d>
#include <osg/Node>
#include <osg/Matrix>
#include <osg/Group>

#include <boost/random.hpp>
#include <tf/transform_datatypes.h>

class InertialMeasurementUnit
{

public:
  std::string name,parentLinkName;

  /** Constructor
   * @param imu_name the name of the IMU
   * @param parentName the name of the link that holds the IMU
   * @param imu_parent the node of the scene graph that holds the imu
   * @param rMl the imu measures are given with respect to the root (r). Use rMl to transform them to another frame ('l' is the new frame, typically the localized world)
   * @param imu_std the standard deviation on the imu measures
   */
  InertialMeasurementUnit(std::string imu_name, std::string parentName, osg::Node *imu_parent, osg::Matrixd rMl, double imu_std = 0) :
      name(imu_name),parentLinkName(parentName), parent_(imu_parent), rMl_(rMl), std_(imu_std)
  {
    imu_node_ = new osg::Node();
    imu_parent->asGroup()->addChild(imu_node_);
  }

  int getTFTransform(tf::Pose & pose, std::string & parent);
  osg::Quat getMeasurement();

  double getStandardDeviation()
  {
    return std_;
  }

  virtual ~InertialMeasurementUnit()
  {
  }

private:
  osg::ref_ptr<osg::Node> parent_;
  osg::Matrixd rMl_;
  double std_;
  osg::ref_ptr<osg::Node> imu_node_;

  boost::mt19937 rng_; ///< Boost random number generator
};

#endif /* INERTIALMEASUREMENTUNIT_H_ */
