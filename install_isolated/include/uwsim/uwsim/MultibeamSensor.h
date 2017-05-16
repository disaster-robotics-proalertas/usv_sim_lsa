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

#ifndef MULTIBEAMSENSOR_H_
#define MULTIBEAMSENSOR_H_

#include "VirtualCamera.h"

class MultibeamSensor
{
  struct Remap
  {
    int pixel1, pixel2;
    double weight1, weight2;
    double distort;
  };

public:
  std::vector<VirtualCamera> vcams; //Virtual Cameras
  std::string name, parentLinkName;
  int numpixels, camPixels, nCams;
  double range, initAngle, finalAngle, angleIncr, camsFOV;
  osg::ref_ptr<osg::Geode> geode; //Geometry node that draws the beam
  std::vector<Remap> remapVector;
  osg::Node *trackNode;

  MultibeamSensor(osg::Group *uwsim_root, std::string name, std::string parentName, osg::Node *trackNode, double initAngle, double finalAngle,
                  double alpha, double range, unsigned int mask, int visible,unsigned int ARMask);
  void preCalcTable();
  int getTFTransform(tf::Pose & pose, std::string & parent);
};

#endif
