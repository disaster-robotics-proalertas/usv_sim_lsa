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

#include <uwsim/PhysicsBuilder.h>

PhysicsBuilder::PhysicsBuilder(SceneBuilder * scene_builder, ConfigFile config)
{
  loadPhysics(scene_builder, config);
}

void PhysicsBuilder::loadPhysics(SceneBuilder * scene_builder, ConfigFile config)
{

  physics = new BulletPhysics(config.physicsConfig, scene_builder->scene->getOceanSurface());
  OSG_INFO << "Loading Physics" << std::endl;

  //Add physics to vehicles
  for (unsigned int i = 0; i < scene_builder->iauvFile.size(); i++)
  {
    for (unsigned int j = 0; j < scene_builder->iauvFile[i]->urdf->link.size(); j++)
    {
      osg::Node * link = scene_builder->iauvFile[i]->urdf->link[j];
      osg::Node * cs = NULL;
      btRigidBody* rigidBody;
      boost::shared_ptr<PhysicProperties> pp;
      pp.reset(new PhysicProperties);
      pp->init();
      pp->isKinematic = 1;
      pp->csType = "compound box";
      pp->mass=0;
      //Look for config vehicle, to search for physic properties.
      for (std::list<Vehicle>::iterator cfgVehicle = config.vehicles.begin(); cfgVehicle != config.vehicles.end();
          cfgVehicle++)
        if (cfgVehicle->name == scene_builder->iauvFile[i]->name)
          for (unsigned int part = 0; part < cfgVehicle->links.size(); part++)
            if (cfgVehicle->links[part].name == link->getName()) //Config vehicle found!
            {
              if(cfgVehicle->links[part].cs) //Add collisionShape if available
              {
                cs = UWSimGeometry::loadGeometry(cfgVehicle->links[part].cs);
                if (!cs)
                  std::cerr << "Collision shape couldn't load, using visual to create physics" << std::endl;
              }
	      //When creating the collision shape from geometry it uses child nodes, so we need to load the  
	      // geometry on a clean node to avoid multiple collision shapes
	      //TODO: avoid to reload the same geometry reusing visual geometry
	      if(!cs)
		cs = UWSimGeometry::loadGeometry(cfgVehicle->links[part].geom);

              if(cfgVehicle->links[part].mass > 0)  //If mass is set in config add it.
                pp->mass=cfgVehicle->links[part].mass;
            }

      CollisionDataType * colData = new CollisionDataType(link->getName(), scene_builder->iauvFile[i]->name, 1);
      rigidBody = physics->addObject(NULL, link, colData, pp, cs);

      //Update visual node data with physics properties
      osg::ref_ptr < NodeDataType > data = dynamic_cast<NodeDataType*>(link->getUserData());
      data->rigidBody = rigidBody;
      link->setUserData(data);
    }
    scene_builder->iauvFile[i]->urdf->physics = physics;

    assert(scene_builder->iauvFile[i]->devices.get() != NULL);
    //apply physics to devices
    for (unsigned int j = 0; j < scene_builder->iauvFile[i]->devices->all.size(); j++)
      scene_builder->iauvFile[i]->devices->all[j]->applyPhysics(physics);
  }

  //Add physics to objects  
  for (unsigned int i = 0; i < scene_builder->objects.size(); i++)
  {
    //create Matrix Transform to use it on physics
    osg::Matrix mat;
    mat.makeIdentity();
    osg::MatrixTransform * mt = new osg::MatrixTransform();
    osg::Group * parent = scene_builder->objects[i]->getParent(0);
    mt->addChild(scene_builder->objects[i]);
    parent->removeChild(scene_builder->objects[i]);
    parent->addChild(mt);

    //NodeDataType * data;
    CollisionDataType * colData = new CollisionDataType(scene_builder->objects[i]->getName(), " ", 0);

    //Search for object in config, and look for physic properties
    boost::shared_ptr < PhysicProperties > pp;
    for (std::list<Object>::iterator j = config.objects.begin(); j != config.objects.end(); j++)
    {
      if (j->name == scene_builder->objects[i]->getName())
      {
        pp = j->physicProperties;
      }

    }

    //Check if object has a collisionShape defined
    osg::Node * cs = NULL;
    if (pp && pp->cs != ""){
      boost::shared_ptr<Geometry> geom =(boost::shared_ptr<Geometry>) new Geometry;
      geom->file=pp->cs;
      geom->type=0;
      cs = UWSimGeometry::loadGeometry(geom);
    }

    btRigidBody* rigidBody;

    rigidBody = physics->addObject(mt, scene_builder->objects[i], colData, pp, cs);
    
    //wMb->setUserData(data); 

    //store physics in object's data
    osg::ref_ptr < NodeDataType > data = dynamic_cast<NodeDataType*>(scene_builder->objects[i]->getUserData());
    data->rigidBody = rigidBody;
    scene_builder->objects[i]->setUserData(data);
  }

  //Add ROSPhysInterfaces
  while (config.ROSPhysInterfaces.size() > 0)
  {
    ROSInterfaceInfo rosInterface = config.ROSPhysInterfaces.front();

    boost::shared_ptr < ROSInterface > iface;

    if (rosInterface.type == ROSInterfaceInfo::contactSensorToROS)
      iface = boost::shared_ptr < contactSensorToROS
          > (new contactSensorToROS(scene_builder->root, physics, rosInterface.targetName, rosInterface.topic,
                                    rosInterface.rate));

    scene_builder->ROSInterfaces.push_back(iface);
    config.ROSPhysInterfaces.pop_front();

  }

  OSG_INFO << "Physics Loaded!" << std::endl;

}

