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

#ifndef KINEMATICCHAIN_H_
#define KINEMATICCHAIN_H_

#include "SimulatorConfig.h"
#include <osg/MatrixTransform>
#include <ros/ros.h>

#include <iostream>
#include <string.h>
#include "BulletPhysics.h"

struct MimicArm
{
  int joint;
  double offset, multiplier;
  int sliderCrank;
};

/** Abstract Kinematic Chain class for holding articulated 3D Models
 */
class KinematicChain
{
public:
  std::vector<osg::ref_ptr<osg::Node> > link; ///< pointers to link models
  std::vector<double> q; ///< Joint values
  std::vector<std::pair<double, double> > limits; ///<Limits for joints by default -PI , PI
  std::vector<std::string> names; //Names of the joints

  std::vector<MimicArm> mimic; //Mimic joints info
  std::vector<int> jointType; //type of joints 0 fixed, 1 rotation, 2 prismatic
  std::vector<osg::ref_ptr<osg::MatrixTransform> > joints; ///< pointers to transforms between links
  std::vector<osg::ref_ptr<osg::MatrixTransform> > zerojoints; ///<pointers to original (zero) transforms between links
  osg::ref_ptr<osg::MatrixTransform> baseTransform; ///<pointer to the first node in the graph (base tranform)

  //osg::MatrixTransform *tool_transform;	///< Transform between the end-effector and the tool base frame
  //osg::ref_ptr<osg::Node> tool;		///< Pointer to the tool osg node

  std::vector<double> qLastSafe; ///< Last joint values not colliding
  BulletPhysics * physics; /// Bullet physics pointer to query

  ros::WallTime last;
  int started;

  KinematicChain(int nlinks, int njoints);

  void setJointPosition(double *q, int n);
  void setJointVelocity(double *qdot, int n);
  void setJointPosition(std::vector<double> &q, std::vector<std::string> names = std::vector<std::string>());
  void setJointVelocity(std::vector<double> &qdot, std::vector<std::string> names = std::vector<std::string>());
  std::vector<double> getJointPosition();
  std::vector<std::string> getJointName();
  std::map<std::string, double> getFullJointMap();  //Builds a map: name -> jointValue with all non-fixed joints (even mimic joints)

  /** Get the number of links */
  int getNumberOfLinks()
  {
    return link.size();
  }
  /** Get the number of joints*/
  int getNumberOfJoints()
  {
    return q.size();
  }

  ~KinematicChain();

protected:
  virtual void updateJoints(std::vector<double> &q)=0; ///< Implemented by childs for joint position update
};

#endif /* KINEMATICCHAIN_H_ */
