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

#include <uwsim/BulletPhysics.h>

#if BT_BULLET_VERSION > 279
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#endif

// Define filter masks
unsigned int vehicleCollidesWith(COL_OBJECTS);
unsigned int objectsCollidesWith(COL_EVERYTHING);

//This motion state just keeps track of another btObject (used for force sensors)
class MirrorMotionState : public btMotionState
{
public:
  MirrorMotionState(btRigidBody * copied)
  {
    copy=copied;
  }

  virtual void getWorldTransform(btTransform &worldTrans) const
  {
     copy->getMotionState()->getWorldTransform(worldTrans);
  }

  virtual void setWorldTransform(const btTransform &worldTrans)
  {
    //There is no need to do nothing as there is no real object
  }

protected:
  btRigidBody * copy;

};

//Copies a bullet body with it's physical properties. (used for force sensors)
btRigidBody* BulletPhysics::copyObject(btRigidBody * copied)
{

  btVector3 inertia;
  copied->getCollisionShape()->calculateLocalInertia(1.0/copied->getInvMass(), inertia);

  MirrorMotionState* motion = new MirrorMotionState(copied);
  btRigidBody::btRigidBodyConstructionInfo rb(1.0/copied->getInvMass(), motion,copied->getCollisionShape(),inertia);  //inertia should be copied too
  btRigidBody* body = new btRigidBody(rb);

  CollisionDataType * colData = new CollisionDataType("copied", "copied", 1);
  body->setUserPointer(colData);

  //Restrictions are not being copied

  body->setDamping(copied->getLinearDamping(), copied->getAngularDamping());

  //addRigidBody adds its own collision masks, changing after object creation do not update masks so objects are removed and readded in order to update masks to improve collisions performance.
  dynamicsWorld->addRigidBody(body);

  //We suppose it's a vehicle
  dynamicsWorld->btCollisionWorld::removeCollisionObject(body);
  dynamicsWorld->addCollisionObject(body, short(COL_VEHICLE), short(vehicleCollidesWith));

  body->setActivationState(DISABLE_DEACTIVATION);


  return (body);
}


class MyMotionState : public btMotionState
{
public:

  MyMotionState(osg::Node * obj, osg::MatrixTransform *root)
  {
    transf = root;
    object = obj;
  }

  void setNode(osg::Node *node)
  {
    object = node;
  }

  virtual void getWorldTransform(btTransform &worldTrans) const
  {
    boost::shared_ptr<osg::Matrix> mat = getWorldCoords(object);
    //BTTransforms do not use scale, so we turn it back to 1.
    mat->preMultScale(osg::Vec3d(1.0/mat->getScale().x(),1.0/mat->getScale().y(),1.0/mat->getScale().z())); 
    worldTrans = osgbCollision::asBtTransform(*mat);
  }

  virtual void setWorldTransform(const btTransform &worldTrans)
  {
    //Object initial position
    boost::shared_ptr<osg::Matrix> mat = getWorldCoords(transf->getParent(0));

    //Get object position in matrixd
    osg::Matrixd worldMatrix;
    btQuaternion rot = worldTrans.getRotation();
    btVector3 pos = worldTrans.getOrigin();
    worldMatrix.makeRotate(osg::Quat(rot.x(), rot.y(), rot.z(), rot.w()));
    worldMatrix.setTrans(osg::Vec3d(pos.x(), pos.y(), pos.z()));
    //BTtransforms do not use scale, as collisionShape is scaled to Node scale we apply osg scale
    worldMatrix.preMultScale(osg::Vec3d(mat->getScale().x(),mat->getScale().y(),mat->getScale().z()));

    //matrix transform from object initial position to final position
    osg::Matrixd rootmat = worldMatrix * (mat->inverse(*mat));

    //Apply transform to object matrix
    transf->setMatrix(rootmat);
  }

protected:
  osg::Node * object;
  osg::MatrixTransform *transf;

};

void BulletPhysics::stepSimulation(btScalar timeStep, int maxSubSteps = 1,
                                   btScalar fixedTimeStep = btScalar(1.) / btScalar(60.))
{
  //dynamicsWorld->debugDrawWorld();
  //printManifolds();
  //cleanManifolds();
  physicsStep=1; //Keeps track of ongoing physics processing.
  callbackManager->substep=0;
  ((btDynamicsWorld*)dynamicsWorld)->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
  physicsStep=0;
}

void BulletPhysics::printManifolds()
{
  //std::cout<<dispatcher->getNumManifolds()<<std::endl;
  for (int i = 0; i < dispatcher->getNumManifolds(); i++)
  {
    btCollisionObject* colObj0 = (btCollisionObject*)dispatcher->getManifoldByIndexInternal(i)->getBody0();
    btCollisionObject* colObj1 = (btCollisionObject*)dispatcher->getManifoldByIndexInternal(i)->getBody1();
    CollisionDataType * nombre = (CollisionDataType *)colObj0->getUserPointer();
    CollisionDataType * nombre2 = (CollisionDataType *)colObj1->getUserPointer();
    double min = 999999;
    for (int j = 0; j < dispatcher->getManifoldByIndexInternal(i)->getNumContacts(); j++)
      if (dispatcher->getManifoldByIndexInternal(i)->getContactPoint(j).getDistance() < min)
        min = dispatcher->getManifoldByIndexInternal(i)->getContactPoint(j).getDistance();
    if (min < 999999)
    {
      std::cout << i << " ";
      if (nombre)
        std::cout << nombre->name << " " << " ";
      if (nombre2)
        std::cout << nombre2->name;
      std::cout << " " << min << std::endl;
    }
  }
}

//Adds tick callback manager which will do all stuff needed in pretick callback
void preTickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    BulletPhysics::TickCallbackManager *w = static_cast<BulletPhysics::TickCallbackManager *>(world->getWorldUserInfo());
    w->physicsInternalPreProcessCallback(timeStep);
}

//Adds tick callback manager which will do all stuff needed in posttick callback
void postTickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    BulletPhysics::TickCallbackManager *w = static_cast<BulletPhysics::TickCallbackManager *>(world->getWorldUserInfo());
    w->physicsInternalPostProcessCallback(timeStep);
}

//oceanSurf is not used right now, but should be to add water physics
BulletPhysics::BulletPhysics(PhysicsConfig physicsConfig, osgOcean::OceanTechnique* oceanSurf) 
{
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  
  #if BT_BULLET_VERSION <= 279
    solver = new btSequentialImpulseConstraintSolver();
  #else
    if(physicsConfig.solver==PhysicsConfig::Dantzig){
      btDantzigSolver* mlcp = new btDantzigSolver();
      solver = new btMLCPSolver(mlcp);
    }

    else if(physicsConfig.solver==PhysicsConfig::SolveProjectedGauss){
      btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
      solver = new btMLCPSolver(mlcp);
    }

    else if(physicsConfig.solver==PhysicsConfig::SequentialImpulse){
      solver = new btSequentialImpulseConstraintSolver();
    }
  #endif


  btVector3 worldAabbMin(-10000, -10000, -10000);
  btVector3 worldAabbMax(10000, 10000, 10000);
  inter = new btAxisSweep3(worldAabbMin, worldAabbMax, 1000);

  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, inter, solver, collisionConfiguration);
  dynamicsWorld->getDispatchInfo().m_enableSPU = true;

  btVector3 gravity(physicsConfig.gravity[0], physicsConfig.gravity[1], physicsConfig.gravity[2]);
  if (physicsConfig.gravity[0] == 0 && physicsConfig.gravity[1] == 0 && physicsConfig.gravity[2] == 0)
  {
    gravity = UWSIM_DEFAULT_GRAVITY;
  }

  dynamicsWorld->setGravity(gravity);
  oceanSurface = oceanSurf;

  /*debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawContactPoints|| btIDebugDraw::DBG_DrawWireframe || btIDebugDraw::DBG_DrawText);
   dynamicsWorld->setDebugDrawer(&debugDrawer);
   debugDrawer.BeginDraw();
   debugDrawer.setEnabled(true);*/

   //Create pre-tick and post-tick callbacks
   callbackManager= new TickCallbackManager();
   dynamicsWorld->setInternalTickCallback(preTickCallback, static_cast<void *>(callbackManager),true);
   dynamicsWorld->setInternalTickCallback(postTickCallback, static_cast<void *>(callbackManager));
}

//Adds a force sensor to tickCallbackManager
int BulletPhysics::TickCallbackManager::addForceSensor(btRigidBody * copy, btRigidBody * target)
{
  ForceSensorcbInfo fs;
  fs.target=target;
  fs.copy=copy;
  forceSensors.push_back(fs);

  return forceSensors.size()-1;
}

//Gets Speed difference from tickcallbackmanager using reference returned on addition
void BulletPhysics::TickCallbackManager::getForceSensorSpeed(int forceSensor, double linSpeed[3],double angSpeed[3])
{
  linSpeed[0]=forceSensors[forceSensor].linFinal.x()-forceSensors[forceSensor].linInitial.x();
  linSpeed[1]=forceSensors[forceSensor].linFinal.y()-forceSensors[forceSensor].linInitial.y();
  linSpeed[2]=forceSensors[forceSensor].linFinal.z()-forceSensors[forceSensor].linInitial.z();

  angSpeed[0]=forceSensors[forceSensor].angFinal.x()-forceSensors[forceSensor].angInitial.x();
  angSpeed[1]=forceSensors[forceSensor].angFinal.y()-forceSensors[forceSensor].angInitial.y();
  angSpeed[2]=forceSensors[forceSensor].angFinal.z()-forceSensors[forceSensor].angInitial.z();
}

//forceSensor pre-tick callback
void BulletPhysics::TickCallbackManager::preTickForceSensors()
{
  for(int i=0;i<forceSensors.size();i++)
  {
    forceSensors[i].copy->setCenterOfMassTransform(forceSensors[i].target->getCenterOfMassTransform());
    forceSensors[i].copy->clearForces();
    forceSensors[i].copy->setLinearVelocity(forceSensors[i].target->getLinearVelocity());
    forceSensors[i].copy->setAngularVelocity(forceSensors[i].target->getAngularVelocity());
    forceSensors[i].linInitial=forceSensors[i].copy->getLinearVelocity();
    forceSensors[i].angInitial=forceSensors[i].copy->getAngularVelocity();
  }
}

//forceSensor post-tick callback
void BulletPhysics::TickCallbackManager::postTickForceSensors()
{
  for(int i=0;i<forceSensors.size();i++)
  {
    forceSensors[i].linFinal=forceSensors[i].copy->getLinearVelocity();
    forceSensors[i].angFinal=forceSensors[i].copy->getAngularVelocity();
  }
}

//This function will be called just before physics step (or substep) start,
//It may be used to add buoyancy and drag forces to dynamic objects
void BulletPhysics::TickCallbackManager::physicsInternalPreProcessCallback(btScalar timeStep)
{
  if (substep==0){
    preTickForceSensors();
  }
}

//This function will be called just after physics step (or substep) finishes
void BulletPhysics::TickCallbackManager::physicsInternalPostProcessCallback(btScalar timeStep)
{
  postTickForceSensors();
  substep++;
}

btCollisionShape* BulletPhysics::GetCSFromOSG(osg::Node * node, collisionShapeType_t ctype)
{
  btCollisionShape* cs = NULL;

  if (ctype == SHAPE_BOX)
    cs = osgbCollision::btBoxCollisionShapeFromOSG(node);
  else if (ctype == SHAPE_SPHERE)
    cs = osgbCollision::btSphereCollisionShapeFromOSG(node);
  else if (ctype == SHAPE_COMPOUND_TRIMESH)
    cs = osgbCollision::btCompoundShapeFromOSGGeodes(node, CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE);
  else if (ctype == SHAPE_COMPOUND_BOX)
    cs = osgbCollision::btCompoundShapeFromOSGGeodes(node, BOX_SHAPE_PROXYTYPE);
  else if (ctype == SHAPE_COMPOUND_CYLINDER)
    cs = osgbCollision::btCompoundShapeFromOSGGeodes(node, CYLINDER_SHAPE_PROXYTYPE);
  else if (ctype == SHAPE_TRIMESH)
    cs = osgbCollision::btTriMeshCollisionShapeFromOSG(node);

  return cs;
}

btRigidBody* BulletPhysics::addObject(osg::MatrixTransform *root, osg::Node *node, CollisionDataType * data,
                                      boost::shared_ptr<PhysicProperties> pp, osg::Node * colShape)
{
  //if not physic properties set default.
  if (!pp)
  {
    pp.reset(new PhysicProperties);
    pp->init();
    if (data->isVehicle)
    {
      pp->isKinematic = 1;
      pp->csType = "compound box";
      pp->mass = 0; //there is no need to set mass (and inertia) to kinematic objects
    }
  }

  collisionShapeType_t ctype = SHAPE_BOX;

  if (pp->csType == "box")
    ctype = BulletPhysics::SHAPE_BOX;
  else if (pp->csType == "sphere")
    ctype = BulletPhysics::SHAPE_SPHERE;
  else if (pp->csType == "compound box")
    ctype = BulletPhysics::SHAPE_COMPOUND_BOX;
  else if (pp->csType == "compound cylinder")
    ctype = BulletPhysics::SHAPE_COMPOUND_CYLINDER;
  else if (pp->csType == "trimesh")
    ctype = BulletPhysics::SHAPE_TRIMESH;
  else if (pp->csType == "compound trimesh")
    ctype = BulletPhysics::SHAPE_COMPOUND_TRIMESH;
  else
    OSG_WARN << data->name << " has an unknown collision shape type: " << pp->csType
        << ". Using default box shape(dynamic) trimesh(kinematic). Check xml file, allowed collision shapes are 'box' 'compound box' 'trimesh' 'compound trimesh' 'compound cylinder."
        << std::endl;

  btCollisionShape* cs;
  if (colShape == NULL)
    cs = GetCSFromOSG(node, ctype);
  else
    cs = GetCSFromOSG(colShape, ctype);

  //As btTransforms do not work with scale, we scale the collision shape
  boost::shared_ptr<osg::Matrix> mat = getWorldCoords(node);
  cs->setLocalScaling(btVector3(mat->getScale().x(),mat->getScale().y(),mat->getScale().z()));

  btVector3 inertia = btVector3(pp->inertia[0], pp->inertia[1], pp->inertia[2]);

  MyMotionState* motion = new MyMotionState(node, root);
  if (inertia.length() == 0) //asking bullet to calculate inertia only if it is unset
    cs->calculateLocalInertia(pp->mass, inertia);
  btRigidBody::btRigidBodyConstructionInfo rb(pp->mass, motion, cs, inertia);
  btRigidBody* body = new btRigidBody(rb);
  body->setUserPointer(data);

  if (pp->maxAngularLimit[0] >= pp->minAngularLimit[0] || pp->maxAngularLimit[1] >= pp->minAngularLimit[1]
      || pp->maxAngularLimit[1] >= pp->minAngularLimit[1] || pp->maxLinearLimit[0] >= pp->minLinearLimit[0]
      || pp->maxLinearLimit[1] >= pp->minLinearLimit[1] || pp->maxLinearLimit[1] >= pp->minLinearLimit[1])
  {
    btRigidBody* pBodyB = new btRigidBody(0, motion, 0);
    pBodyB->setActivationState(DISABLE_DEACTIVATION);

    btTransform frameInA, frameInB;
    frameInA = btTransform::getIdentity();
    frameInB = btTransform::getIdentity();
    if (pp->minAngularLimit[1] < pp->maxAngularLimit[1])
    { //There is a constraint on Y axis (quaternions uncertainties can make it unstable)
      if (not (pp->minAngularLimit[0] < pp->maxAngularLimit[0] or pp->minAngularLimit[2] < pp->maxAngularLimit[2]))
      { //There is no constraint on X and Z so we rotate to move it
        frameInA.getBasis().setEulerZYX(0, 0, 1.57);
        frameInB.getBasis().setEulerZYX(0, 0, 1.57);
        double aux = pp->minAngularLimit[0];
        pp->minAngularLimit[0] = pp->minAngularLimit[1];
        pp->minAngularLimit[1] = aux;
        aux = pp->maxAngularLimit[0];
        pp->maxAngularLimit[0] = pp->maxAngularLimit[1];
        pp->maxAngularLimit[1] = aux;
      } //TODO check other possible rotations to avoid aborting.
      else if (pp->minAngularLimit[1] <= -1.50 || pp->minAngularLimit[1] >= 1.50)
      { //safety threshold
        std::cerr << "Constraints in Y axis must be between -PI/2 PI/2" << std::endl;
        exit(0);
      }
    }

    btGeneric6DofConstraint* pGen6DOF = new btGeneric6DofConstraint(*body, *pBodyB, frameInA, frameInB, true);
    pGen6DOF->setLinearLowerLimit(btVector3(pp->minLinearLimit[0], pp->minLinearLimit[1], pp->minLinearLimit[2]));
    pGen6DOF->setLinearUpperLimit(btVector3(pp->maxLinearLimit[0], pp->maxLinearLimit[1], pp->maxLinearLimit[2]));

    pGen6DOF->setAngularLowerLimit(btVector3(pp->minAngularLimit[0], pp->minAngularLimit[1], pp->minAngularLimit[2]));
    pGen6DOF->setAngularUpperLimit(btVector3(pp->maxAngularLimit[0], pp->maxAngularLimit[1], pp->maxAngularLimit[2]));

    dynamicsWorld->addConstraint(pGen6DOF, true);
  }

  //body->setLinearFactor(btVector3(pp->linearFactor[0],pp->linearFactor[1],pp->linearFactor[2]));
  //body->setAngularFactor(btVector3(pp->angularFactor[0],pp->angularFactor[1],pp->angularFactor[2]));

  body->setDamping(pp->linearDamping, pp->angularDamping);

  //addRigidBody adds its own collision masks, changing after object creation do not update masks so objects are removed and readded in order to update masks to improve collisions performance.
  dynamicsWorld->addRigidBody(body);
  if (data->isVehicle)
  {
    dynamicsWorld->btCollisionWorld::removeCollisionObject(body);
    dynamicsWorld->addCollisionObject(body, short(COL_VEHICLE), short(vehicleCollidesWith));
  }
  else
  {
    dynamicsWorld->btCollisionWorld::removeCollisionObject(body);
    dynamicsWorld->addCollisionObject(body, short(COL_OBJECTS), short(objectsCollidesWith));
  }

  body->setActivationState(DISABLE_DEACTIVATION);
  if (pp->isKinematic)
  {
    body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
  }

  return (body);
}

int BulletPhysics::getNumCollisions()
{
  return dispatcher->getNumManifolds();
}

btPersistentManifold * BulletPhysics::getCollision(int i)
{
  return dispatcher->getManifoldByIndexInternal(i);
}

void BulletPhysics::cleanManifolds()
{ //it removes contact points with long lifetime
  //std::cout<<dispatcher->getNumManifolds()<<"aa"<<std::endl;
  for (int i = 0; i < dispatcher->getNumManifolds(); i++)
  {
    btPersistentManifold * col = dispatcher->getManifoldByIndexInternal(i);
    //std::cout<<col->getNumContacts()<<std::endl;
    for (int j = 0; j < col->getNumContacts(); j++)
      if (col->getContactPoint(j).getLifeTime() > 300)
        col->removeContactPoint(j);

  }
}
