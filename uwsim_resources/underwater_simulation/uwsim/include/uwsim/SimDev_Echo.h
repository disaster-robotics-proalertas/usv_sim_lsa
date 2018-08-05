///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//"Echo" example, SimulatedDevice_Echo.h
#ifndef SIMULATEDDEVICE_ECHO_H_
#define SIMULATEDDEVICE_ECHO_H_
#include "SimulatedDevice.h"
using namespace uwsim;

/*
 * Example header of driver/rosinterface configuration/factory
 *
 * Included in SimulatedDevices.cpp
 */

//Driver/ROSInterface configuration
class SimDev_Echo_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  std::string info;
  //constructor
  SimDev_Echo_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
  }
};

//Driver/ROSInterface factory class
class SimDev_Echo_Factory : public SimulatedDeviceFactory
{
public:
  //this is the only place the device/interface type is set
  SimDev_Echo_Factory(std::string type_ = "echo") :
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
#include <std_msgs/String.h>

//Driver class
class SimDev_Echo : public SimulatedDevice
{
  void applyPhysics(BulletPhysics * bulletPhysics)
  {
  }
public:
  std::string info; //Device's property

  SimDev_Echo(SimDev_Echo_Config * cfg);
};

//ROS publishers and subscribers work exactly as before, no subclassing is needed
class SimDev_Echo_ROSPublisher : public ROSPublisherInterface
{
  //this is just an example, use a pointer to SimulatedIAUV, if only ROSInterface is implemented
  //pointer to a device
  SimDev_Echo * dev;
public:
  SimDev_Echo_ROSPublisher(SimDev_Echo *dev, std::string topic, int rate) :
      ROSPublisherInterface(topic, rate), dev(dev)
  {
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~SimDev_Echo_ROSPublisher()
  {
  }
};

#endif
