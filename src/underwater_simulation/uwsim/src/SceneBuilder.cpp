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

#include <uwsim/SceneBuilder.h>

#include <osg/Notify>

#include <string>
#include <vector>

#include <uwsim/osgOceanScene.h>
#include <uwsim/HUDCamera.h>
#include <uwsim/VirtualRangeSensor.h>
#include <uwsim/ROSInterface.h>
#include <uwsim/UWSimUtils.h>
#include <uwsim/TrajectoryVisualization.h>

using namespace std;

SceneBuilder::SceneBuilder()
{
  int argc = 0;
  char **argv = NULL;
  arguments.reset(new osg::ArgumentParser(&argc, argv));
}

SceneBuilder::SceneBuilder(int *argc, char **argv)
{
  arguments.reset(new osg::ArgumentParser(argc, argv));
}

SceneBuilder::SceneBuilder(boost::shared_ptr<osg::ArgumentParser> args)
{
  arguments = args;
}

bool SceneBuilder::loadScene(std::string xml_file)
{
  ConfigFile config(xml_file);
  //TODO: if config.valid() {
  return loadScene(config);
}

/** Creates a scene graph from an XML Scene file. Overrides XML values with arguments passed in the constructor
 */
bool SceneBuilder::loadScene(ConfigFile config)
{
  float windx = config.windx, windy = config.windy;
  while (arguments->read("--windx", windx))
    ;
  while (arguments->read("--windy", windy))
    ;
  osg::Vec2f windDirection(windx, windy);

  float windSpeed = config.windSpeed;
  while (arguments->read("--windSpeed", windSpeed))
    ;

  float depth = config.depth;
  //while (arguments->read("--depth", depth));

  float reflectionDamping = config.reflectionDamping;
  while (arguments->read("--reflectionDamping", reflectionDamping))
    ;

  float reswidth = config.resw, resheight = config.resh;
  while (arguments->read("--resw", reswidth))
    ;
  while (arguments->read("--resh", resheight))
    ;

  float scale = config.waveScale;
  while (arguments->read("--waveScale", scale))
    ;

  bool isChoppy = not config.isNotChoppy;
  while (arguments->read("--isNotChoppy"))
    isChoppy = false;

  float choppyFactor = config.choppyFactor;
  while (arguments->read("--choppyFactor", choppyFactor))
    ;
  choppyFactor = -choppyFactor;

  float crestFoamHeight = config.crestFoamHeight;
  while (arguments->read("--crestFoamHeight", crestFoamHeight))
    ;

  double oceanSurfaceHeight = config.oceanSurfaceHeight;
  while (arguments->read("--oceanSurfaceHeight", oceanSurfaceHeight))
    ;

  bool disableShaders = config.disableShaders;
  if (arguments->read("--disableShaders"))
    disableShaders = true;

  bool disableTextures = false;
  if (arguments->read("--disableTextures"))
    disableTextures = true;

  bool freeMotion = config.freeMotion;
  if (arguments->read("--freeMotion"))
  {
    freeMotion = true;
  }

  osgOcean::ShaderManager::instance().enableShaders(!disableShaders);

  root = new osg::Group;

  //Initialize ocean scene.
  scene = new osgOceanScene(config.offsetp, config.offsetr, windDirection, windSpeed, depth, reflectionDamping, scale,
                            isChoppy, choppyFactor, crestFoamHeight, false, "terrain");

  if (disableShaders)
  {
    // If shaders disabled, disable all special effects that depend on shaders.
    scene->getOceanScene()->enableDistortion(false);
    scene->getOceanScene()->enableGlare(false);
    scene->getOceanScene()->enableUnderwaterDOF(false);

    // These are only implemented in the shader, with no fixed-pipeline equivalent
    scene->getOceanScene()->enableUnderwaterScattering(false);
    // For these two, we might be able to use projective texturing so it would
    // work on the fixed pipeline?
    scene->getOceanScene()->enableReflections(false);
    scene->getOceanScene()->enableRefractions(false);
    scene->getOceanScene()->enableGodRays(false); // Could be done in fixed pipeline?
    scene->getOceanScene()->enableSilt(false); // Could be done in fixed pipeline?
  }
  else //Use UWSim default scene shader
  {
   static const char model_vertex[] = "default_scene.vert";
   static const char model_fragment[] = "default_scene.frag";
   osg::Program* program = osgOcean::ShaderManager::instance().createProgram("object_shader", model_vertex,model_fragment, "", "");
   scene->getOceanScene()->setDefaultSceneShader(program);

   root->getOrCreateStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
   root->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
   root->getStateSet()->addUniform(new osg::Uniform("SLStex", 3));
   root->getStateSet()->addUniform(new osg::Uniform("SLStex2", 4));
   root->getStateSet()->addUniform( new osg::Uniform("stddev", 0.0f ) );
   root->getStateSet()->addUniform( new osg::Uniform("mean", 0.0f ) );
   root->getStateSet()->addUniform( new osg::Uniform("light", (float)config.lightRate ) );
  }

  scene->getOceanScene()->setOceanSurfaceHeight(oceanSurfaceHeight);
  scene->getOceanScene()->setUnderwaterFog(config.fogDensity,
                                           osg::Vec4f(config.fogColor[0], config.fogColor[1], config.fogColor[2], 1));
  scene->getOceanScene()->setUnderwaterDiffuse(osg::Vec4f(config.color[0], config.color[1], config.color[2], 1));
  scene->getOceanScene()->setUnderwaterAttenuation(
      osg::Vec3f(config.attenuation[0], config.attenuation[1], config.attenuation[2]));

  //Add config file iauv
  int nvehicle = config.vehicles.size();
  int slsProjectors=0;
  for (int i = 0; i < nvehicle; i++)
  {
    Vehicle vehicle = config.vehicles.front();
    boost::shared_ptr < SimulatedIAUV > siauv(new SimulatedIAUV(this, vehicle));
    iauvFile.push_back(siauv);
    config.vehicles.pop_front();

    scene->addObject(iauvFile[i]->baseTransform);

    siauv->setVehiclePosition(vehicle.position[0], vehicle.position[1], vehicle.position[2], vehicle.orientation[0],
                              vehicle.orientation[1], vehicle.orientation[2]);

    for (int j = 0; j < vehicle.nlinks; j++)
    {
      NodeDataType * data = new NodeDataType(0);
      siauv->urdf->link[j]->setUserData(data);
    }

    if (vehicle.jointValues.size() && siauv->urdf != NULL)
    {
      siauv->urdf->setJointPosition(vehicle.jointValues);
    }

    slsProjectors+=vehicle.sls_projectors.size();
  }

  //Enable or disable sls shader computation
  if(slsProjectors==0)
  {
    root->getOrCreateStateSet()->addUniform(new osg::Uniform("sls_projector", false));
  }
  else
  {
    root->getOrCreateStateSet()->addUniform(new osg::Uniform("sls_projector", true));
  }

  //Add objects added in config file.
  while (config.objects.size() > 0)
  {
    Object auxObject = config.objects.front();

    osg::Matrixd wMb_m;
    wMb_m.makeRotate(
        osg::Quat(auxObject.orientation[0], osg::Vec3d(1, 0, 0), auxObject.orientation[1], osg::Vec3d(0, 1, 0),
                  auxObject.orientation[2], osg::Vec3d(0, 0, 1)));
    wMb_m.setTrans(auxObject.position[0], auxObject.position[1], auxObject.position[2]);
    wMb_m.preMultScale(osg::Vec3d(auxObject.scale[0],auxObject.scale[1],auxObject.scale[2]));

    osg::ref_ptr < osg::MatrixTransform > wMb = new osg::MatrixTransform(wMb_m);
    osg::Node *object = scene->addObject(wMb, auxObject.file, &auxObject);
    object->setName(auxObject.name);

    //FIXME: Do not trust on object name
    if (auxObject.name != "terrain")
    {
      NodeDataType * data = new NodeDataType(1, auxObject.position, auxObject.orientation);
      object->setUserData(data);
    }
    else
    {
      NodeDataType * data = new NodeDataType(0);
      object->setUserData(data);
    }

    if (auxObject.buried>0) //If object is buried create a reactive heightfield 
    {
      osg::Node* mud=createHeightField(object,"sand2.jpg",auxObject.buried, iauvFile);
      object->asGroup()->addChild(mud);
    }  

    objects.push_back(object);
    config.objects.pop_front();
  }

  //Set-up the scene graph and main loop
  root->addChild(scene->getScene());

  //   iauv->lightSource->addChild(scene->getScene());	//Add vehicles light sources to the scene. Check if can be added to the .osg file.
  //   root->addChild( iauv->lightSource );

  OSG_INFO << "Setting vehicle virtual cameras" << std::endl;
  for (int j = 0; j < nvehicle; j++)
  {
    for (unsigned int i = 0; i < iauvFile[j]->getNumCams(); i++)
    {
      iauvFile[j]->camview[i].textureCamera->addChild(scene->getScene());
      root->addChild(iauvFile[j]->camview[i].textureCamera);

      //Add same fog as water fog. TODO: Can it be moved inside VirtualCamera?
      osg::Fog *fog = new osg::Fog();
      fog->setMode(osg::Fog::EXP2);
      fog->setFogCoordinateSource(osg::Fog::FRAGMENT_DEPTH);
      fog->setDensity(config.fogDensity);
      fog->setColor(osg::Vec4d(config.fogColor[0], config.fogColor[1], config.fogColor[2], 1));
      iauvFile[j]->camview[i].textureCamera->getOrCreateStateSet()->setAttributeAndModes(fog, osg::StateAttribute::ON);
    }
  }

  OSG_INFO << "Setting interfaces with external software..." << std::endl;
  while (config.ROSInterfaces.size() > 0)
  {
    ROSInterfaceInfo rosInterface = config.ROSInterfaces.front();

    boost::shared_ptr < ROSInterface > iface;
    if (rosInterface.type == ROSInterfaceInfo::ROSOdomToPAT)
      iface = boost::shared_ptr < ROSOdomToPAT
          > (new ROSOdomToPAT(root, rosInterface.topic, rosInterface.targetName));

    if (rosInterface.type == ROSInterfaceInfo::ROSTwistToPAT)
      iface = boost::shared_ptr < ROSTwistToPAT
          > (new ROSTwistToPAT(root, rosInterface.topic, rosInterface.targetName));

    if (rosInterface.type == ROSInterfaceInfo::PATToROSOdom)
      iface = boost::shared_ptr < PATToROSOdom
          > (new PATToROSOdom(root, rosInterface.targetName, rosInterface.topic, rosInterface.rate));

    if (rosInterface.type == ROSInterfaceInfo::WorldToROSTF)
    {
      iface = boost::shared_ptr < WorldToROSTF
          > (new WorldToROSTF( this, rosInterface.rootName, rosInterface.enableObjects, rosInterface.rate));

    }
    if (rosInterface.type == ROSInterfaceInfo::ROSPointCloudLoader)
    {
      iface = boost::shared_ptr < ROSPointCloudLoader
          > (new ROSPointCloudLoader(rosInterface.topic,root,scene->getOceanScene()->getARMask(),rosInterface.del));

    }
    if (rosInterface.type == ROSInterfaceInfo::ROSJointStateToArm
        || rosInterface.type == ROSInterfaceInfo::ArmToROSJointState)
    {
      int correspondences=0;
      //Find corresponding SimulatedIAUV Object
      for (int j = 0; j < nvehicle; j++)
      {
        if (iauvFile[j]->name == rosInterface.targetName)
        {
          if (rosInterface.type == ROSInterfaceInfo::ROSJointStateToArm)
            iface = boost::shared_ptr < ROSJointStateToArm > (new ROSJointStateToArm(rosInterface.topic, iauvFile[j]));
          else
            iface = boost::shared_ptr < ArmToROSJointState
                > (new ArmToROSJointState(iauvFile[j].get(), rosInterface.topic, rosInterface.rate));
	  correspondences++;
        }
      }
      if(correspondences==0)
        ROS_WARN ("Arm ROSInterface is not able to find %s vehicle.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("Arm ROSInterface more than one %s vehicle.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::VirtualCameraToROSImage)
    {
      int correspondences=0;
      //Find corresponding VirtualCamera Object on all the vehicles
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int c = 0; c < iauvFile[j]->getNumCams(); c++)
          if (iauvFile[j]->camview[c].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < VirtualCameraToROSImage
                > (new VirtualCameraToROSImage(&(iauvFile[j]->camview[c]), rosInterface.topic, rosInterface.infoTopic,
                                               rosInterface.rate, rosInterface.depth));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("VirtualCameraToROSInterface is not able to find %s camera.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("VirtualCameraToROSInterface more than one %s cameras.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::RangeCameraToPCL)
    {
      int correspondences=0;
      //Find corresponding VirtualCamera Object on all the vehicles
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int c = 0; c < iauvFile[j]->getNumCams(); c++)
          if (iauvFile[j]->camview[c].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < RangeCameraToPCL
                > (new RangeCameraToPCL(&(iauvFile[j]->camview[c]), rosInterface.topic,rosInterface.rate));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("VirtualCameraToROSInterface is not able to find %s camera.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("VirtualCameraToROSInterface more than one %s cameras.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::RangeImageSensorToROSImage)
    {
      int correspondences=0;
      //Find corresponding VirtualCamera Object on all the vehicles
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int c = 0; c < iauvFile[j]->getNumCams(); c++)
          if (iauvFile[j]->camview[c].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < VirtualCameraToROSImage
                > (new VirtualCameraToROSImage(&(iauvFile[j]->camview[c]), rosInterface.topic, rosInterface.infoTopic,
                                               rosInterface.rate, 1));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("RangeImageSensorToROSImage is not able to find %s sensor.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("RangeImageSensorToROSImage more than one %s sensors.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::ROSImageToHUD)
    {
      boost::shared_ptr < HUDCamera
          > realcam(
              new HUDCamera(rosInterface.w, rosInterface.h, rosInterface.posx, rosInterface.posy, rosInterface.scale,
                            rosInterface.blackWhite));
      iface = boost::shared_ptr < ROSImageToHUDCamera
          > (new ROSImageToHUDCamera(rosInterface.topic, rosInterface.infoTopic, realcam));
      realcams.push_back(realcam);
    }

    if (rosInterface.type == ROSInterfaceInfo::RangeSensorToROSRange)
    {
      int correspondences=0;
      //Find corresponding VirtualRangeSensor Object on all the vehicles (look for rangeSensors and objectPickers)
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int c = 0; c < iauvFile[j]->getNumRangeSensors(); c++)
          if (iauvFile[j]->range_sensors[c].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < RangeSensorToROSRange
                > (new RangeSensorToROSRange(&(iauvFile[j]->range_sensors[c]), rosInterface.topic, rosInterface.rate));
	    correspondences++;
	  }

        for (unsigned int c = 0; c < iauvFile[j]->getNumObjectPickers(); c++)
          if (iauvFile[j]->object_pickers[c].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < RangeSensorToROSRange
                > (new RangeSensorToROSRange(&(iauvFile[j]->object_pickers[c]), rosInterface.topic, rosInterface.rate));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("RangeSensorToROSRange is not able to find %s sensor.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("RangeSensorToROSRange more than one %s sensors.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::ImuToROSImu)
    {
      int correspondences=0;
      //Find corresponding IMU Object on all the vehicles
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int i = 0; i < iauvFile[j]->imus.size(); i++)
          if (iauvFile[j]->imus[i].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < ImuToROSImu
                > (new ImuToROSImu(&(iauvFile[j]->imus[i]), rosInterface.topic, rosInterface.rate));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("ImuToROSImu is not able to find %s sensor.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("ImuToROSImu more than one %s sensors.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::PressureSensorToROS)
    {
      int correspondences=0;
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int i = 0; i < iauvFile[j]->pressure_sensors.size(); i++)
          if (iauvFile[j]->pressure_sensors[i].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < PressureSensorToROS
                > (new PressureSensorToROS(&(iauvFile[j]->pressure_sensors[i]), rosInterface.topic, rosInterface.rate));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("PressureSensorToROS is not able to find %s sensor.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("PressureSensorToROS more than one %s sensors.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::GPSSensorToROS)
    {
      int correspondences=0;
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int i = 0; i < iauvFile[j]->gps_sensors.size(); i++)
          if (iauvFile[j]->gps_sensors[i].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < GPSSensorToROS
                > (new GPSSensorToROS(&(iauvFile[j]->gps_sensors[i]), rosInterface.topic, rosInterface.rate));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("GPSSensorToROS is not able to find %s sensor.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("GPSSensorToROS more than one %s sensors.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::DVLSensorToROS)
    {
      int correspondences=0;
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int i = 0; i < iauvFile[j]->dvl_sensors.size(); i++)
          if (iauvFile[j]->dvl_sensors[i].name == rosInterface.targetName)
	  {
            iface = boost::shared_ptr < DVLSensorToROS
                > (new DVLSensorToROS(&(iauvFile[j]->dvl_sensors[i]), rosInterface.topic, rosInterface.rate));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("DVLSensorToROS is not able to find %s sensor.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("DVLSensorToROS more than one %s sensors.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::multibeamSensorToLaserScan)
    {
      int correspondences=0;
      for (int j = 0; j < nvehicle; j++)
      {
        for (unsigned int i = 0; i < iauvFile[j]->multibeam_sensors.size(); i++)
          if (iauvFile[j]->multibeam_sensors[i].name == rosInterface.targetName)
	  {
            iface =
                boost::shared_ptr < MultibeamSensorToROS
                    > (new MultibeamSensorToROS(&(iauvFile[j]->multibeam_sensors[i]), rosInterface.topic,
                                                rosInterface.rate));
	    correspondences++;
	  }
      }
      if(correspondences==0)
        ROS_WARN ("multibeamSensorToLaserScan is not able to find %s sensor.",rosInterface.targetName.c_str());
      else if (correspondences > 1)
        ROS_WARN ("multibeamSensorToLaserScan more than one %s sensors.",rosInterface.targetName.c_str());
    }

    if (rosInterface.type == ROSInterfaceInfo::ROSPoseToPAT)
      iface = boost::shared_ptr < ROSPoseToPAT > (new ROSPoseToPAT(root, rosInterface.topic, rosInterface.targetName));

    if (rosInterface.type == ROSInterfaceInfo::SimulatedDevice)
    {
      std::vector < boost::shared_ptr<ROSInterface> > ifaces = SimulatedDevices::getInterfaces(rosInterface, iauvFile);
      for (size_t i = 0; i < ifaces.size(); ++i)
        ROSInterfaces.push_back(ifaces[i]);
    }

    if (iface)
      ROSInterfaces.push_back(iface);
    config.ROSInterfaces.pop_front();
  }
  //root->addChild(physics.debugDrawer.getSceneGraph());

  while (config.trajectories.size() > 0)
  {
    ShowTrajectory trajectory = config.trajectories.front();

    osg::ref_ptr<TrajectoryUpdateCallback> node_tracker = new TrajectoryUpdateCallback(trajectory.color, 0.02,trajectory.lineStyle,
        trajectory.timeWindow, root, scene->getOceanScene()->getARMask());

    osg::Node * trackNode=findRN(trajectory.target,root);
    if(trackNode)
    {
      trackNode->setUpdateCallback(node_tracker);
      trajectories.push_back(trackNode);
    }
    else
      OSG_FATAL << "Scene Builder: "<<trajectory.target<<" trajectory target not found, please check your XML."<<std::endl;

    config.trajectories.pop_front();
  }

  return true;
}

SceneBuilder::~SceneBuilder()
{
  //for(unsigned int i=0; i<ROSInterfaces.size();i++){
  //ROSInterfaces[i]->cancel();
  //ROSInterfaces[i]->join();
  //}

}

