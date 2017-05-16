/*
 * VirtualStructuredLightProjector.h
 *
 *  Created on: 06/02/2013
 *      Author: Miquel Massot
 *	Modified by: Javier Perez
 *
 */

#ifndef VirtualSLSProjector_H
#define VirtualSLSProjector_H

#include <osgDB/ReadFile>

#include "VirtualCamera.h"

/**  Virtual Structured Light sensor that Projects a light (laser or not) in the scene */
class VirtualSLSProjector
{
public:
  std::string name;
  std::string image_name;
  osg::ref_ptr<osg::Node> node;
  osg::ref_ptr<osg::Node> root;
  double range; ///< Max projection range
  double fov; ///< Field of view
  unsigned int textureUnit;
  osg::Texture2D* dbgDepthTexture;
  VirtualCamera camera;

  VirtualSLSProjector(std::string name,std::string parentName, osg::Node *root, osg::Node *node, std::string image_name, double fov,
                      bool laser);
  VirtualSLSProjector();

  virtual void init(std::string name,std::string parentName, osg::Node *root, osg::Node *node, std::string image_name, double range,
                    double fov, bool laser);
};

#endif
