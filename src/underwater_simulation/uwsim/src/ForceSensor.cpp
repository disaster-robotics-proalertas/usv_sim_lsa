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

#include <pluginlib/class_list_macros.h>
#include <uwsim/ForceSensor.h>

ForceSensor::ForceSensor(ForceSensor_Config * cfg, osg::ref_ptr<osg::Node> target) :
    SimulatedDevice(cfg)
{
  this->target = target;
  this->offsetp[0]=cfg->offsetp[0];  this->offsetp[1]=cfg->offsetp[1];  this->offsetp[2]=cfg->offsetp[2];
  offset.makeRotate(osg::Quat(cfg->offsetr[0], osg::Vec3d(1, 0, 0), cfg->offsetr[1], osg::Vec3d(0, 1, 0), cfg->offsetr[2],osg::Vec3d(0, 0, 1)));
  physics=NULL;
  physicsApplied=0;
}

void ForceSensor::applyPhysics(BulletPhysics * bulletPhysics)
{
  physics=bulletPhysics;
  osg::ref_ptr<NodeDataType> data = dynamic_cast<NodeDataType*>(target->getUserData());
  copy=physics->copyObject(data->rigidBody);
  copy->setGravity(btVector3(0,0,0));
  copy->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(offsetp[0],offsetp[1],offsetp[2])));
  btTarget = data->rigidBody;
  btTarget->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(offsetp[0],offsetp[1],offsetp[2])));

  if(btTarget->getInvMass()==0)
  {
    ROS_FATAL("ForceSensor %s used in a null mass object.", name.c_str());
    exit(0);
  } 

  CBreference=physics->callbackManager->addForceSensor(copy,btTarget);
  physicsApplied=1;
}

void ForceSensor::getForceTorque(double force[3], double torque[3])
{

  if(!physics)
  {
    ROS_FATAL("ForceSensor %s can't retrieve physics information. Missing enable physics?", name.c_str());
    exit(0);
  }
  if(physics->physicsStep ==0) //Check if physics is looping (data is not reliable)
  {

    osg::Matrix ObjectMat= offset * *getWorldCoords(target); //aply rotation offset to transform from world coords.

    double linSpeed[3],angSpeed[3];
    physics->callbackManager->getForceSensorSpeed(CBreference,linSpeed,angSpeed);

    osg::Vec3 res=ObjectMat.getRotate().inverse()*osg::Vec3(linSpeed[0],linSpeed[1],linSpeed[2]);

    force[0]=res.x();
    force[1]=res.y();
    force[2]=res.z();

    res=ObjectMat.getRotate().inverse()*osg::Vec3(angSpeed[0],angSpeed[1],angSpeed[2]);

    //torque forces are extremely high so they ar reduced.(some physics parameter must be wrong).
    torque[0]=res.x()/10;
    torque[1]=res.y()/10;
    torque[2]=res.z()/10;
  }
  else  //Maybe last reliable data should be sent
  {
    force[0]=0;
    force[1]=0;
    force[2]=0;

    torque[0]=0;
    torque[1]=0;
    torque[2]=0;

  }
}

SimulatedDeviceConfig::Ptr ForceSensor_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{
  ForceSensor_Config * cfg = new ForceSensor_Config(getType());
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if(child->get_name() == "target")
      config->extractStringChar(child, cfg->target);
    else if(child->get_name() == "offsetp")
      config->extractPositionOrColor(child, cfg->offsetp);
    else if(child->get_name() == "offsetr")
      config->extractPositionOrColor(child, cfg->offsetr);
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool ForceSensor_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder,
                                      size_t iteration)
{
  if (iteration > 0)
    return true;
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType())
    {
      ForceSensor_Config * cfg = dynamic_cast<ForceSensor_Config *>(vehicleChars.simulated_devices[i].get());
      osg::ref_ptr<osg::Node> target;
      for(int j=0;j<auv->urdf->link.size();j++)
      {
        if(auv->urdf->link[j]->getName()==cfg->target)
        {
          target=auv->urdf->link[j];
        }
      }
      auv->devices->all.push_back(ForceSensor::Ptr(new ForceSensor(cfg,target)));
    }
  return true;
}

std::vector<boost::shared_ptr<ROSInterface> > ForceSensor_Factory::getInterface(
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
                > (new ForceSensor_ROSPublisher(dynamic_cast<ForceSensor*>(iauvFile[i]->devices->all[d].get()),
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

void ForceSensor_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("ForceSensor_ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise < geometry_msgs::WrenchStamped > (topic, 1);
  while (!dev->physicsApplied)
  {
    ROS_INFO("ForceSensor_ROSPublisher Waiting for physics to be initialized...");
    sleep(1.0);
  }
}

void ForceSensor_ROSPublisher::publish()
{

  double force[3], torque[3], elapsed;

  dev->getForceTorque(force,torque);

  elapsed = 1.0/publish_rate;

  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = getROSTime();
  msg.header.frame_id =dev->target->getName();

  msg.wrench.force.x=force[0]/elapsed*(1/dev->btTarget->getInvMass());
  msg.wrench.force.y=force[1]/elapsed*(1/dev->btTarget->getInvMass());
  msg.wrench.force.z=force[2]/elapsed*(1/dev->btTarget->getInvMass());

  msg.wrench.torque.x=torque[0]/elapsed*(1/dev->btTarget->getInvMass());
  msg.wrench.torque.y=torque[1]/elapsed*(1/dev->btTarget->getInvMass());
  msg.wrench.torque.z=torque[2]/elapsed*(1/dev->btTarget->getInvMass());

  pub_.publish(msg);
}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(ForceSensor_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(ForceSensor_Factory, ForceSensor_Factory, uwsim::SimulatedDeviceFactory)
#endif

