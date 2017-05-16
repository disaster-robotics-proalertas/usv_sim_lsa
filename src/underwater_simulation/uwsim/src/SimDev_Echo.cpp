//"Echo" example, SimulatedDevice_Echo.cpp

#include <pluginlib/class_list_macros.h>
#include <uwsim/SimDev_Echo.h>

SimDev_Echo::SimDev_Echo(SimDev_Echo_Config * cfg) :
    SimulatedDevice(cfg)
{
  this->info = cfg->info;
}

SimulatedDeviceConfig::Ptr SimDev_Echo_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{
  SimDev_Echo_Config * cfg = new SimDev_Echo_Config(getType());
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "info")
      config->extractStringChar(child, cfg->info);
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool SimDev_Echo_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder,
                                      size_t iteration)
{
  if (iteration > 0)
    return true;
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType())
    {
      SimDev_Echo_Config * cfg = dynamic_cast<SimDev_Echo_Config *>(vehicleChars.simulated_devices[i].get());
      if (cfg && cfg->info.length() > 0)
      {
        auv->devices->all.push_back(SimDev_Echo::Ptr(new SimDev_Echo(cfg)));
      }
      else
        OSG_FATAL << "SimDev_Echo device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
            << vehicleChars.name << "' has empty info, discarding..." << std::endl;
    }
  return true;
}

std::vector<boost::shared_ptr<ROSInterface> > SimDev_Echo_Factory::getInterface(
    ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  std::vector < boost::shared_ptr<ROSInterface> > ifaces;
  for (size_t i = 0; i < iauvFile.size(); ++i)
    for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d)
      if (iauvFile[i]->devices->all[d]->getType() == this->getType()
          && iauvFile[i]->devices->all[d]->name == rosInterface.targetName)
      {
        ifaces.push_back(
            boost::shared_ptr < ROSInterface
                > (new SimDev_Echo_ROSPublisher(dynamic_cast<SimDev_Echo*>(iauvFile[i]->devices->all[d].get()),
                                                rosInterface.topic, rosInterface.rate)));
        //rosInterface.values are for new and non-standard xml configurations, but it looks like currently existing rosInterface fields are enough...
        //below is just an example how to get values, alternatively you can use rosInterface.values["name"]
        //for(std::map<std::string,std::string>::iterator it = rosInterface.values.begin() ;it != rosInterface.values.end();++it)
        //	ROS_INFO("rosInterface.values[%s]='%s'",  it->first.c_str(), it->second.c_str());
      }
  if (ifaces.size() == 0)
    ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());
  return ifaces;
}

void SimDev_Echo_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("SimDev_Echo_ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise < std_msgs::String > (topic, 1);
}

void SimDev_Echo_ROSPublisher::publish()
{
  std_msgs::String msg;
  if (dev != NULL)
    msg.data = dev->info;
  else
    msg.data = "dev==NULL";
  pub_.publish(msg);
}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(SimDev_Echo_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(SimDev_Echo_Factory, SimDev_Echo_Factory, uwsim::SimulatedDeviceFactory)
#endif

