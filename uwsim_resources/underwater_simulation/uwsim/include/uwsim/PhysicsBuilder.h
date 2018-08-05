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

#ifndef PHYSICSBUILDER_H
#define PHYSICSBUILDER_H

#include "SceneBuilder.h"
#include "BulletPhysics.h"
#include "ConfigXMLParser.h"

class PhysicsBuilder
{
public:
  osg::ref_ptr<BulletPhysics> physics;
public:

  PhysicsBuilder(SceneBuilder * scene_builder, ConfigFile config);
  PhysicsBuilder()
  {
  }
  void loadPhysics(SceneBuilder * scene_builder, ConfigFile config);

};

#endif
