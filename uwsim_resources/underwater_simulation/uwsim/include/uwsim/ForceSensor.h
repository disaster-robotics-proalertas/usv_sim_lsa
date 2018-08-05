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

#ifndef FORCESENSOR_H_
#define FORCESENSOR_H_
#include "SimulatedDevice.h"
using namespace uwsim;

class ForceSensor_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  std::string target;
  double offsetp[3];
  double offsetr[3];
  //constructor
  ForceSensor_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
  }
};

//Driver/ROSInterface factory class
class ForceSensor_Factory : public SimulatedDeviceFactory
{
public:
  //this is the only place the device/interface type is set
  ForceSensor_Factory(std::string type_ = "ForceSensor") :
      SimulatedDeviceFactory(type_)
  {
  }
  ;

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
                                                             std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

//can be a sparate header file for actual implementation classes...

#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "BulletPhysics.h"

//Driver class

class ForceSensor : public SimulatedDevice
{
  void applyPhysics(BulletPhysics * bulletPhysics);
  double lastTimeStep;
  int CBreference;
public:
  BulletPhysics * physics;
  btRigidBody * copy, * btTarget;  //Rigid object copy with physical reaction
  osg::ref_ptr<osg::Node> target;
  double offsetp[3];
  osg::Matrixd offset; //We only need rotation as traslation goes to bullet directly
  int physicsApplied;

  ForceSensor(ForceSensor_Config * cfg, osg::ref_ptr<osg::Node> target);
  void getForceTorque(double force[3], double torque[3]);
};



//ROS publishers and subscribers work exactly as before, no subclassing is needed
class ForceSensor_ROSPublisher : public ROSPublisherInterface
{
  //this is just an example, use a pointer to SimulatedIAUV, if only ROSInterface is implemented
  //pointer to a device
  ForceSensor * dev;
public:
  ForceSensor_ROSPublisher(ForceSensor *dev, std::string topic, int rate) :
      ROSPublisherInterface(topic, rate), dev(dev)
  {
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~ForceSensor_ROSPublisher()
  {
  }
};

#endif
