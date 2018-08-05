/*
 * Copyright (c) 2013 Tallinn University of Technology.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Contributors:
 *     Yuri Gavshin
 */

#include <uwsim/SimulatedDevices.h>
#include <uwsim/ConfigXMLParser.h>
#include <ros/ros.h>
#include <osg/Notify>

#include <pluginlib/class_loader.h>
using namespace uwsim;

class SimulatedDevicesLoader
{
  boost::shared_ptr<pluginlib::ClassLoader<SimulatedDeviceFactory> > simdev_loader;
  vector<string> available_plugins;
public:
  //a list of "factories" to initialize and apply a device and/or rosinterface
  std::vector<SimulatedDeviceFactory::Ptr> factories;

  SimulatedDevicesLoader()
  {
    simdev_loader.reset(new pluginlib::ClassLoader<SimulatedDeviceFactory>("uwsim", "uwsim::SimulatedDeviceFactory"));

    available_plugins = simdev_loader->getDeclaredClasses();
    for (size_t i = 0; i < available_plugins.size(); ++i)
    {
      OSG_ALWAYS << "Loading SimulatedDevices plugin: '" << available_plugins.at(i) << "'" << std::endl;
      factories.push_back(simdev_loader->createInstance(available_plugins.at(i)));
    }

    for (size_t i = 0; i < factories.size(); ++i)
      for (size_t j = 0; j < i; ++j)
        if (factories[i]->getType() == factories[j]->getType())
          OSG_FATAL << "SimulatedDevices factories types must be unique, but the same type '" << factories[i]->getType()
              << "' is specified at indexes " << j << " and " << i << " in initFacotries() in SimulatedDevices.cpp"
              << std::endl;
  }

  ~SimulatedDevicesLoader()
  {
    factories.clear();
    for (size_t i = 0; i < available_plugins.size(); ++i)
    {
      simdev_loader->unloadLibraryForClass(available_plugins.at(i));
    }
  }
};

boost::shared_ptr<SimulatedDevicesLoader> loader(new SimulatedDevicesLoader());

SimulatedDevices::SimulatedDevices()
{
}

std::vector<boost::shared_ptr<ROSInterface> > SimulatedDevices::getInterfaces(
    ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  std::vector < boost::shared_ptr<ROSInterface> > ifaces;
  bool isFactoryFound = false;
  if (rosInterface.type == ROSInterfaceInfo::SimulatedDevice)
  {
    for (size_t i = 0; i < loader->factories.size(); ++i)
      if (loader->factories[i]->getType() == rosInterface.subtype)
      {
        isFactoryFound = true;
        std::vector < boost::shared_ptr<ROSInterface> > ifaces_ = loader->factories[i]->getInterface(rosInterface,
                                                                                                     iauvFile);
        for (size_t j = 0; j < ifaces_.size(); ++j)
          ifaces.push_back(ifaces_[j]);
      }

    if (!isFactoryFound)
      OSG_FATAL << "Unknown ROSIterface '" << rosInterface.subtype << "ROS', skipping..." << std::endl;
  }
  return ifaces;
}

void SimulatedDevices::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *oscene)
{
  for (size_t iteration = 0; iteration < 10; ++iteration) //executed max 10 times
  {
    bool configured = true;
    for (size_t i = 0; i < loader->factories.size(); ++i)
    {
      if (!loader->factories[i]->applyConfig(auv, vehicleChars, oscene, iteration))
        configured = false;
    }
    if (configured)
      break;
  }
}

static void processConfigNode(const xmlpp::Node* node, ConfigFile * config, SimulatedDeviceConfig::Ptr cfg)
{
  if (!cfg)
    return;

  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "name" && cfg->name.length() == 0)
      config->extractStringChar(child, cfg->name);
  }
}

std::vector<SimulatedDeviceConfig::Ptr> SimulatedDevices::processConfig(const xmlpp::Node* node, ConfigFile * config,
                                                                        bool isDevice)
{
  std::vector < SimulatedDeviceConfig::Ptr > devs;
  if (node->get_name() == "text" || node->get_name() == "comment")
    return devs;
  if (isDevice)
  {
    bool isFactoryFound = false;
    for (size_t i = 0; i < loader->factories.size(); ++i)
      if (loader->factories[i]->getType() == node->get_name())
      {
        isFactoryFound = true;
        SimulatedDeviceConfig::Ptr dev = loader->factories[i]->processConfig(node, config);
        if (dev)
          processConfigNode(node, config, dev);
        devs.push_back(dev);
      }
    if (!isFactoryFound)
      OSG_FATAL << "Unknown SimulatedDevice '" << node->get_name() << "', skipping..." << std::endl;
  }
  else
  {
    xmlpp::Node::NodeList list = node->get_children();
    for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
    {
      const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
      std::vector < SimulatedDeviceConfig::Ptr > devs_ = processConfig(child, config, true);
      for (size_t i = 0; i < devs_.size(); ++i)
        devs.push_back(devs_.at(i));
    }
  }
  return devs;
}

SimulatedDeviceConfig::SimulatedDeviceConfig(std::string type)
{
  this->type = type;
}

SimulatedDeviceFactory::SimulatedDeviceFactory(std::string type)
{
  this->type = type;
}

SimulatedDevice::SimulatedDevice(SimulatedDeviceConfig * cfg)
{
  this->type = cfg->getType();
  this->name = cfg->name;
}
