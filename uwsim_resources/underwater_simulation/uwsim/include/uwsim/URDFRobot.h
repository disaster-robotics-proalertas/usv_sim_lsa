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

#ifndef URDFROBOT_H_
#define URDFROBOT_H_

#include "SimulatorConfig.h"
#include "KinematicChain.h"
#include "ConfigXMLParser.h"

#include <osgOcean/OceanScene>
#include <osg/Switch>

#include <iostream>
#include <string.h>

class URDFRobot : public KinematicChain
{

public:

  std::vector<osg::Vec3d> joint_axis;
  std::string URDFFile;

  URDFRobot(osgOcean::OceanScene *oscene, Vehicle vehicle);
  void addToKinematicChain(osg::Node * link, btRigidBody* body);

  ~URDFRobot();

protected:

  void updateJoints(std::vector<double> &q);
  void updateJoints(std::vector<double> &q, int startJoint, int numJoints);

private:
  void moveJoints(std::vector<double> &q);

};

#endif /* URDFROBOT_H_ */
