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

#ifndef SIMULATEDIAUV_H_
#define SIMULATEDIAUV_H_

#include "osgOceanScene.h"
#include "URDFRobot.h"
#include "VirtualCamera.h"
#include "ConfigXMLParser.h"
#include "VirtualRangeSensor.h"
#include "VirtualSLSProjector.h"
#include "ObjectPicker.h"
#include "InertialMeasurementUnit.h"
#include "PressureSensor.h"
#include "GPSSensor.h"
#include "DVLSensor.h"
#include "MultibeamSensor.h"

class SceneBuilder;

/* An I-AUV */
class SimulatedIAUV
{
public:
  std::vector<VirtualCamera> camview;
  std::vector<VirtualRangeSensor> range_sensors;
  std::vector<VirtualSLSProjector> sls_projectors;
  std::vector<ObjectPicker> object_pickers;
  std::vector<InertialMeasurementUnit> imus;
  std::vector<PressureSensor> pressure_sensors;
  std::vector<GPSSensor> gps_sensors;
  std::vector<DVLSensor> dvl_sensors;
  std::vector<MultibeamSensor> multibeam_sensors;
  boost::shared_ptr<SimulatedDevices> devices;

  typedef enum
  {
    ARM5, PA10
  } arm_t;

  std::string name; ///< Vehicle name
  boost::shared_ptr<URDFRobot> urdf; ///< URDF I-AUV
  //osg::LightSource* lightSource;	///< vehicle lamp
  osg::ref_ptr<osg::MatrixTransform> baseTransform;
  osg::Vec3d scale;  //Vehicle scale factor

  SimulatedIAUV(SceneBuilder *oscene, Vehicle vehicle);

  //void setVirtualCamera(std::string name, osg::Transform* transform, osg::Vec3d eye, osg::Vec3d orientation, osg::Vec3d upVector, int width, int height);

  //setPosition
  void setVehiclePosition(double x, double y, double z, double yaw)
  {
    setVehiclePosition(x, y, z, 0, 0, yaw);
  }
  void setVehiclePosition(double x, double y, double z, double roll, double pitch, double yaw);
  void setVehiclePosition(double p[6])
  {
    setVehiclePosition(p[0], p[1], p[2], p[3], p[4], p[5]);
  }
  void setVehiclePosition(osg::Matrixd m);

  unsigned int getNumCams()
  {
    return camview.size();
  }
  unsigned int getNumRangeSensors()
  {
    return range_sensors.size();
  }
  unsigned int getNumObjectPickers()
  {
    return object_pickers.size();
  }

  ~SimulatedIAUV()
  {
    OSG_DEBUG << "Simulated IAUV destructor" << std::endl;
  }
};

#endif /* SIMULATEDIAUV_H_ */
