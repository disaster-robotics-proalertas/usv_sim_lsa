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

#ifndef BULLETPHYSICS_H_
#define BULLETPHYSICS_H_

#include "SimulatorConfig.h"
#include "UWSimUtils.h"

#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>

#include <btBulletDynamicsCommon.h>
#include <iostream>

#include <osgOcean/OceanScene>

//#include <osgbCollision/GLDebugDrawer.h>

#define UWSIM_DEFAULT_GRAVITY	btVector3(0,0,-1)

// Define filter groups
enum CollisionTypes
{
  COL_NOTHING = 0x00000000, COL_OBJECTS = 0x00000001, COL_VEHICLE = 0x00000010, COL_EVERYTHING = 0x11111111,
};

/*class NodeDataType : public osg::Referenced{
 public:
 NodeDataType(btRigidBody * rigidBody,int catcha){ catchable=catcha; rb=rigidBody;};
 int catchable;
 btRigidBody * rb;

 };*/

class CollisionDataType : public osg::Referenced
{
public:
  CollisionDataType(std::string nam, std::string vehName, int isVehi)
  {
    vehicleName = vehName;
    name = nam;
    isVehicle = isVehi;
  }
  ;
  std::string getObjectName()
  {
    if (isVehicle)
      return vehicleName;
    else
      return name;
  }
  ;
  std::string name, vehicleName;
  int isVehicle;

};

//Adds tick callback manager which will do all stuff needed in pretick callback
void preTickCallback(btDynamicsWorld *world, btScalar timeStep);

//Adds tick callback manager which will do all stuff needed in posttick callback
void postTickCallback(btDynamicsWorld *world, btScalar timeStep);

class BulletPhysics : public osg::Referenced
{

public:
  typedef enum
  {
    SHAPE_BOX, SHAPE_SPHERE, SHAPE_TRIMESH, SHAPE_COMPOUND_TRIMESH, SHAPE_COMPOUND_BOX, SHAPE_COMPOUND_CYLINDER
  } collisionShapeType_t;

  btDiscreteDynamicsWorld * dynamicsWorld;
  //osgbCollision::GLDebugDrawer debugDrawer;

  BulletPhysics(PhysicsConfig physicsConfig, osgOcean::OceanTechnique* oceanSurf);

  void setGravity(btVector3 g)
  {
    dynamicsWorld->setGravity(g);
  }
  btRigidBody* addObject(osg::MatrixTransform *root, osg::Node *node, CollisionDataType * data,
                         boost::shared_ptr<PhysicProperties> pp, osg::Node * colShape = NULL);

  void stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep);
  void printManifolds();

  int getNumCollisions();

  btRigidBody* copyObject(btRigidBody * copied);
  int physicsStep;

  btPersistentManifold * getCollision(int i);

  ~BulletPhysics()
  {
  }
  ;

  //This class stores information related to different sources that need to be called in internal tick callbacks.
  //To know force sensors.
  class TickCallbackManager
  {
    private:
      //Force sensor structures;
      struct ForceSensorcbInfo
      {
	btRigidBody * copy, * target;
        btVector3 linInitial, angInitial,linFinal, angFinal;
      };
      std::vector<ForceSensorcbInfo> forceSensors;
    
      void preTickForceSensors();
      void postTickForceSensors();
    public:

      int substep; //Tracks number of substep;

      TickCallbackManager(){};
      int addForceSensor(btRigidBody * copy, btRigidBody * target);
      void getForceSensorSpeed(int forceSensor, double linSpeed[3],double angSpeed[3]);
      void physicsInternalPreProcessCallback(btScalar timeStep);
      void physicsInternalPostProcessCallback(btScalar timeStep);
  };


  TickCallbackManager * callbackManager;

private:
  btDefaultCollisionConfiguration * collisionConfiguration;
  btCollisionDispatcher * dispatcher;
  btConstraintSolver * solver;
  btBroadphaseInterface * inter;

  osgOcean::OceanTechnique* oceanSurface;

  void cleanManifolds();
  btCollisionShape* GetCSFromOSG(osg::Node * node, collisionShapeType_t ctype);

};


#endif	

