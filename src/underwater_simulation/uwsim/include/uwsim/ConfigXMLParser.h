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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifndef CONFIGXMLPARSER_H_
#define CONFIGXMLPARSER_H_

#include "SimulatedDevices.h"
#include <libxml++/libxml++.h>
#include <urdf/model.h>

#include <iostream>
using namespace std;
#include <cstdlib>
#include <list>

struct ROSInterfaceInfo
{
  typedef enum
  {
    Unknown, ROSOdomToPAT, PATToROSOdom, ROSJointStateToArm, ArmToROSJointState, VirtualCameraToROSImage,
    RangeSensorToROSRange, ROSImageToHUD, ROSTwistToPAT, ROSPoseToPAT, ImuToROSImu, PressureSensorToROS, GPSSensorToROS,
    DVLSensorToROS, RangeImageSensorToROSImage, multibeamSensorToLaserScan, SimulatedDevice, contactSensorToROS, WorldToROSTF,
    ROSPointCloudLoader, RangeCameraToPCL
  } type_t;
  string subtype; //type of a SimulatedDevice
  std::map<std::string, std::string> values; //all configuration values for a SimulatedDevice
  string topic, infoTopic, targetName, rootName;
  type_t type; //Type of ROSInterface
  int rate; //if it's necessary
  unsigned int w, h; //width and height if necessary
  unsigned int posx, posy, depth, blackWhite, enableObjects; ///< default (x,y) position of the widget if necessary, blackWhite camera
  double scale; ///< default scale of the widget if necessary
  bool del; //Used in ROSPointCloudLoader
};

struct Parameters
{
  double fx, fy, x0, y0, f, n, k;
};

struct Vcam
{
  string name;
  string linkName, roscam, roscaminfo;
  std::string frameId; ///Frame Id for stereo camera images
  int resw, resh, link, range, bw;
  double showpath;
  double position[3], orientation[3];
  double baseLine; ///baseline for stereo cameras
  double fov;
  double std; //Additive gaussian noise deviation
  boost::shared_ptr<Parameters> parameters;
  bool underwaterParticles;
  void init()
  {
    name = "";
    linkName = "";
    roscam = "";
    roscaminfo = "";
    resw = 160;
    resh = 120;
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
    baseLine = 0.0;
    frameId = "";
    showpath = 0;
    parameters.reset();
    range = 0;
    bw = 0;
    fov=50;
    std=0.005;
    underwaterParticles=false;
  }
};

struct slProjector
{
  string name;
  string linkName;
  string image_name;
  double position[3], orientation[3];
  double fov;
  int laser;
  int link;
  void init()
  {
    name = "";
    linkName = "";
    image_name = "";
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
    fov = 0;
    laser = 1;
  }
};

struct rangeSensor
{
  string name;
  string linkName;
  double position[3], orientation[3];
  double range;
  int visible;
  int link;
  void init()
  {
    name = "";
    linkName = "";
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
    range = 0;
    visible = 0;
  }
};

struct Imu
{
  string name;
  string linkName;
  double std; //standard deviation
  double position[3], orientation[3];
  int link;
  void init()
  {
    name = "";
    linkName = "";
    std = 0.0;
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
  }
};

struct XMLPressureSensor
{
  string name;
  string linkName;
  double std; //standard deviation
  double position[3], orientation[3];
  int link;
  void init()
  {
    name = "";
    linkName = "";
    std = 0.0;
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
  }
};

struct XMLGPSSensor
{
  string name;
  string linkName;
  double std; //standard deviation
  double position[3], orientation[3];
  int link;
  void init()
  {
    name = "";
    linkName = "";
    std = 0.0;
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
  }
};

struct XMLDVLSensor
{
  string name;
  string linkName;
  double std; //standard deviation
  double position[3], orientation[3];
  int link;
  void init()
  {
    name = "";
    linkName = "";
    std = 0.0;
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
  }
};

struct XMLMultibeamSensor
{
  string name;
  string linkName;
  double position[3], orientation[3];
  int link;
  int visible;
  double initAngle, finalAngle, angleIncr, range;
  bool underwaterParticles;
  void init()
  {
    name = "";
    linkName = "";
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
    underwaterParticles=false;
    visible = 0;
  }
};

struct Mimic
{
  string jointName;
  double offset, multiplier;
};

struct Geometry
{
  int type; //Related to geometry, 0: mesh from file, 1:box, 2:cylinder, 3:sphere, 4:NoVisual
  double boxSize[3]; //only used in box type
  double length, radius; //only used in cylinder and sphere types
  string file; // only used in mesh type
  double scale[3]; //used in mesh files
};

struct Link
{
  string name;
  double position[3];
  double rpy[3];
  double quat[4];
  std::string material;
  boost::shared_ptr<Geometry> cs, geom;
  double mass;
};

struct Joint
{
  string name;
  int parent, child; //references to Link
  int mimicp, type; //0 fixed, 1 rotation, 2 prismatic.
  float lowLimit, upLimit;
  boost::shared_ptr<Mimic> mimic;
  double position[3];
  double rpy[3];
  double axis[3];
  double quat[4];
};

struct Material
{
  string name;
  double r, g, b, a;
};

struct Vehicle
{
  string name;
  std::vector<Link> links;
  std::vector<Joint> joints;
  int nlinks;
  int njoints;
  int ninitJoints;
  double position[3];
  double orientation[3];
  double scale[3];
  std::vector<double> jointValues;
  std::map<std::string, Material> materials;
  std::list<Vcam> Vcams;
  std::list<Vcam> VRangecams;
  std::list<slProjector> sls_projectors;
  std::list<rangeSensor> range_sensors, object_pickers;
  std::list<Imu> imus;
  std::list<XMLPressureSensor> pressure_sensors;
  std::list<XMLGPSSensor> gps_sensors;
  std::list<XMLDVLSensor> dvl_sensors;
  std::list<XMLMultibeamSensor> multibeam_sensors;
  std::vector<uwsim::SimulatedDeviceConfig::Ptr> simulated_devices;
  std::string URDFFile;
};

struct PhysicProperties
{
  double mass;
  double inertia[3];
  double linearDamping;
  double angularDamping;
  double minLinearLimit[3];
  double maxLinearLimit[3];
  double minAngularLimit[3];
  double maxAngularLimit[3];
  int isKinematic;
  std::string csType, cs;
  void init()
  {
    mass = 1;
    inertia[0] = 0;
    inertia[1] = 0;
    inertia[2] = 0;
    csType = "box";
    cs = "";
    linearDamping = 0;
    angularDamping = 0;
    minLinearLimit[0] = 1;
    minLinearLimit[1] = 1;
    minLinearLimit[2] = 1;
    maxLinearLimit[0] = 0;
    maxLinearLimit[1] = 0;
    maxLinearLimit[2] = 0;
    isKinematic = 0;
    minAngularLimit[0] = 1;
    minAngularLimit[1] = 1;
    minAngularLimit[2] = 1;
    maxAngularLimit[0] = 0;
    maxAngularLimit[1] = 0;
    maxAngularLimit[2] = 0;
  }
  ;
};

struct Object
{
  string name, file;
  double position[3];
  double orientation[3];
  double scale[3];
  double offsetp[3];
  double offsetr[3];
  double buried;// % Object buried in the seafloor
  boost::shared_ptr<PhysicProperties> physicProperties;
};

struct ShowTrajectory
{
  std::string target;
  double color[3];
  int lineStyle;
  double timeWindow;
  void init()
  {
    target="";
    color[0]= 1; color[1]= 0; color[2] = 0;
    lineStyle=1;
    timeWindow=-1;
  }
};

struct PhysicsConfig
{
 typedef enum
 {
    Dantzig,SolveProjectedGauss,SequentialImpulse
 } solver_type;
 double gravity[3];
 double frequency;
 int subSteps;
 solver_type solver;

 void init()
 {
   memset(gravity, 0, 3 * sizeof(double));
   frequency = 60;
   subSteps = 0;
   solver=Dantzig;
 }
 
};

class ConfigFile
{
public:
  //made process and extract methods public to be used in Simulated Devices implementations

  void esPi(string in, double &param);

  void extractFloatChar(const xmlpp::Node* node, double &param);
  void extractIntChar(const xmlpp::Node* node, int &param);
  void extractUIntChar(const xmlpp::Node* node, unsigned int &param);
  void extractStringChar(const xmlpp::Node* node, string &param);
  void extractPositionOrColor(const xmlpp::Node* node, double param[3]);
  void extractOrientation(const xmlpp::Node* node, double param[3]);

  void processFog(const xmlpp::Node* node);
  void processOceanState(const xmlpp::Node* node);
  void processSimParams(const xmlpp::Node* node);
  void processShowTrajectory(const xmlpp::Node* node, ShowTrajectory & trajectory);
  void processParameters(const xmlpp::Node*, Parameters *params);
  void processVcam(const xmlpp::Node* node, Vcam &vcam);
  void processSLProjector(const xmlpp::Node* node, slProjector &slp);
  void processRangeSensor(const xmlpp::Node* node, rangeSensor &rs);
  void processImu(const xmlpp::Node* node, Imu &rs);
  void processPressureSensor(const xmlpp::Node* node, XMLPressureSensor &ps);
  void processDVLSensor(const xmlpp::Node* node, XMLDVLSensor &s);
  void processGPSSensor(const xmlpp::Node* node, XMLGPSSensor &s);
  void processMultibeamSensor(const xmlpp::Node* node, XMLMultibeamSensor &s);
  void processCamera(const xmlpp::Node* node);
  void processJointValues(const xmlpp::Node* node, std::vector<double> &jointValues, int &ninitJoints);
  void processVehicle(const xmlpp::Node* node, Vehicle &vehicle);
  void processPhysicProperties(const xmlpp::Node* node, PhysicProperties &pp);
  void processObject(const xmlpp::Node* node, Object &object);
  void processROSInterface(const xmlpp::Node* node, ROSInterfaceInfo &rosInterface);
  void processROSInterfaces(const xmlpp::Node* node);
  void processXML(const xmlpp::Node* node);

  void processGeometry(urdf::Geometry * geometry, Geometry * geom);
  void processPose(urdf::Pose pose, double position[3], double rpy[3], double quat[4]);
  void processVisual(boost::shared_ptr<const urdf::Visual> visual, Link &link,
                     std::map<std::string, Material> &materials);
  void processJoint(boost::shared_ptr<const urdf::Joint> joint, Joint &jointVehicle, int parentLink, int childLink);
  int processLink(boost::shared_ptr<const urdf::Link> link, Vehicle &vehicle, int nlink, int njoint,
                  std::map<std::string, Material> &materials); //returns current link number
  int processURDFFile(string file, Vehicle &vehicle);

  void postprocessVehicle(Vehicle &vehicle);

public:
  double windx, windy, windSpeed, depth, reflectionDamping, waveScale, choppyFactor, crestFoamHeight,
         oceanSurfaceHeight, fogDensity, lightRate;
  int isNotChoppy, disableShaders, eye_in_hand, freeMotion, resw, resh, enablePhysics;
  string arm, vehicleToTrack;
  double camPosition[3], camLookAt[3], fogColor[3], color[3], attenuation[3], offsetr[3], offsetp[3];
  double camFov, camAspectRatio, camNear, camFar;
  list<Vehicle> vehicles;
  list<Object> objects;
  list<ROSInterfaceInfo> ROSInterfaces;
  list<ROSInterfaceInfo> ROSPhysInterfaces; //Physics interfaces are loaded after physics
  list<ShowTrajectory> trajectories;
  PhysicsConfig physicsConfig;


  ConfigFile(const std::string &fName);
};

#endif
