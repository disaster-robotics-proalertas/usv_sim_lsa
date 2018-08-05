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

#ifndef SIMULATEDDEVICES_H_
#define SIMULATEDDEVICES_H_
#include "SimulatedDevice.h"

//Class added to SimulatedIAUV as devices->, put your data into into this class members or use all vector to store your device
class SimulatedDevices
{
public:
  //Object members
  std::vector<uwsim::SimulatedDevice::Ptr> all;

  SimulatedDevices();

  //Applies parsed XML configurations by calling appropriate methods
  //on all registered factories (set in initFactories() in SimulatedDevices.cpp)
  void applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *oscene);

  //Factory methods

  //returns configured ROSInterface based on given XML configuration
  static std::vector<boost::shared_ptr<ROSInterface> > getInterfaces(
      ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);

  //Parses driver's XML configuration
  static std::vector<uwsim::SimulatedDeviceConfig::Ptr> processConfig(const xmlpp::Node* node, ConfigFile * config,
                                                                      bool isDevice = false);
};
#endif /* SIMULATEDDEVICES_H_ */
