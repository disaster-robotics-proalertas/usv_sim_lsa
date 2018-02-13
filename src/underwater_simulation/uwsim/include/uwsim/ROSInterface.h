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

#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

//UWSIM
#include "SimulatorConfig.h"

#include "URDFRobot.h"
#include "SimulatedIAUV.h"
#include "VirtualCamera.h"
#include "VirtualRangeSensor.h"
#include "PressureSensor.h"
#include "GPSSensor.h"
#include "DVLSensor.h"
#include "HUDCamera.h"
#include "MultibeamSensor.h"
#include "UWSimUtils.h"
#include "BulletPhysics.h"
#include "SceneBuilder.h"

//OSG
#include <OpenThreads/Thread>
#include <osg/PrimitiveSet>
#include <osg/Geode>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Drawable>
#include <osg/Geometry>

//STL
#include <vector>

#include <boost/shared_ptr.hpp>

//ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
//#include <cola2_common/NavigationData.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <pcl_ros/point_cloud.h>

//Max time (in seconds) between two consecutive control references
#define MAX_ELAPSED	1

class ROSInterface : public OpenThreads::Thread
{
protected:
  std::string topic;
  ros::NodeHandle nh_;
  static ros::Time current_time_;

public:
  ROSInterface(std::string topic)
  {
    this->topic = topic;
  }

  virtual ~ROSInterface()
  {
  }

  /**
   * Sets the static ros time in the ROSInterface,
   * to be called once per simulation step
   */
  static void setROSTime(const ros::Time& time)
  {
    current_time_ = time;
  }

  /**
   * Retrieve the current ROS time.
   * Use this instead of calling ros::Time::now()
   * when setting the timestamp of your messages.
   */
  static ros::Time getROSTime()
  {
    return current_time_;
  }
};

class ROSSubscriberInterface : public ROSInterface
{
protected:
  ros::Subscriber sub_;
public:
  ROSSubscriberInterface(std::string topic);

  virtual void createSubscriber(ros::NodeHandle &nh)=0;

  /* Thread code */
  void run();

  ~ROSSubscriberInterface();
};

class ROSOdomToPAT : public ROSSubscriberInterface
{
  osg::ref_ptr<osg::MatrixTransform> transform;
  ros::WallTime last;
  int started;

public:
  ROSOdomToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName);

  virtual void createSubscriber(ros::NodeHandle &nh);

  virtual void processData(const nav_msgs::Odometry::ConstPtr& odom);
  ~ROSOdomToPAT();
};

class ROSTwistToPAT : public ROSSubscriberInterface
{
  osg::ref_ptr<osg::MatrixTransform> transform;
  ros::WallTime last;
  int started;
public:
  ROSTwistToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName);

  virtual void createSubscriber(ros::NodeHandle &nh);

  virtual void processData(const geometry_msgs::TwistStamped::ConstPtr& odom);
  ~ROSTwistToPAT();
};

class ROSPoseToPAT : public ROSSubscriberInterface
{
  osg::ref_ptr<osg::MatrixTransform> transform;
public:
  ROSPoseToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName);

  virtual void createSubscriber(ros::NodeHandle &nh);

  virtual void processData(const geometry_msgs::Pose::ConstPtr& odom);
  ~ROSPoseToPAT();
};

class ROSPointCloudLoader : public ROSSubscriberInterface
{
  osg::ref_ptr<osg::Group> scene_root;
  unsigned int nodeMask;
  osg::ref_ptr < osg::MatrixTransform > lastPCD;
  bool deleteLastPCD;
public:
  ROSPointCloudLoader(std::string topic, osg::ref_ptr<osg::Group> root, unsigned int mask,bool del);
  virtual void createSubscriber(ros::NodeHandle &nh);
  virtual void processData(const sensor_msgs::PointCloud2ConstPtr& msg);
  void colourCloudDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  double interpolate(double val, double y0, double x0, double y1, double x1);
  double base(double val);
  ~ROSPointCloudLoader();
};

/*
 class ROSNavigationDataToPAT: public ROSSubscriberInterface {
 osg::PositionAttitudeTransform *transform;
 public:
 ROSNavigationDataToPAT(std::string topic, osg::PositionAttitudeTransform *t): ROSSubscriberInterface(topic) {
 transform=t;
 }

 virtual void createSubscriber(ros::NodeHandle &nh) {
 sub_ = nh.subscribe<cola2_common::NavigationData>(topic, 10, &ROSNavigationDataToPAT::processData, this);
 }

 virtual void processData(const cola2_common::NavigationData::ConstPtr& odom) {
 //Simulated vehicle frame wrt real vehicle frame
 vpHomogeneousMatrix vMsv(0.8,0,0.8,0,M_PI,0);

 vpHomogeneousMatrix sMsv;


 //Set a position reference
 //Pose of the real vehicle wrt to the localization origin
 vpRotationMatrix pRv(vpRxyzVector(odom->pose[3],odom->pose[4],odom->pose[5]));
 vpTranslationVector pTv(odom->pose[0],odom->pose[1],odom->pose[2]);
 vpHomogeneousMatrix pMv;
 pMv.buildFrom(pTv, pRv);

 //Localization origin wrt simulator origin
 vpRxyzVector sRVp(M_PI,0,-M_PI_2);
 vpRotationMatrix sRp(sRVp);
 vpTranslationVector sTp(-514921,-4677958,3.4);

 vpHomogeneousMatrix sMp;
 sMp.buildFrom(sTp,sRp);

 sMsv=sMp*pMv*vMsv;

 if (transform!=NULL) {
 //OSG_DEBUG << "SimulatedVehicle::processData baseTransform not null" << std::endl;
 transform->setPosition(osg::Vec3d(sMsv[0][3],sMsv[1][3],sMsv[2][3]));
 vpRotationMatrix mr;
 sMsv.extract(mr);
 vpRxyzVector vr(mr);
 osg::Quat sQsv(vr[0],osg::Vec3d(1,0,0), vr[1],osg::Vec3d(0,1,0), vr[2],osg::Vec3d(0,0,1));
 transform->setAttitude(sQsv);
 //baseTransform->setAttitude(osg::Quat(js->pose.pose.orientation.w,osg::Vec3d(0,0,1)));
 }
 }

 ~ROSNavigationDataToPAT(){}
 };
 */

class ROSJointStateToArm : public ROSSubscriberInterface
{
  boost::shared_ptr<SimulatedIAUV> arm;
public:
  ROSJointStateToArm(std::string topic, boost::shared_ptr<SimulatedIAUV> arm);
  virtual void createSubscriber(ros::NodeHandle &nh);

  virtual void processData(const sensor_msgs::JointState::ConstPtr& js);
  ~ROSJointStateToArm();
};

class ROSImageToHUDCamera : public ROSSubscriberInterface
{
  boost::shared_ptr<HUDCamera> cam;
  boost::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Subscriber image_sub;
  std::string image_topic;
public:
  ROSImageToHUDCamera(std::string topic, std::string info_topic, boost::shared_ptr<HUDCamera> camera);

  virtual void createSubscriber(ros::NodeHandle &nh);

  virtual void processData(const sensor_msgs::ImageConstPtr& msg);
  ~ROSImageToHUDCamera();
};

class ROSPublisherInterface : public ROSInterface
{
protected:
  int publish_rate;
  ros::Publisher pub_;
public:
  ROSPublisherInterface(std::string topic, int publish_rate);

  virtual void createPublisher(ros::NodeHandle &nh)=0;
  virtual void publish()=0;

  /* Thread code */
  void run();

  ~ROSPublisherInterface();
};

class PATToROSOdom : public ROSPublisherInterface
{
  osg::ref_ptr<osg::MatrixTransform> transform;
public:
  PATToROSOdom(osg::Group *rootNode, std::string vehicleName, std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~PATToROSOdom();
};

class OceanSurfaceToROSOceanVehicle : public ROSPublisherInterface
{
protected:
  osg::Node *vehicleNode;
  osg::Node *linkNode_;
  osg::ref_ptr<osgOcean::OceanScene> _oceanScene;
  std::string _vehicleName;
  std::string _linkName;
public:
  OceanSurfaceToROSOceanVehicle(osg::Group *rootNode, std::string vehicleName, std::string linkName, std::string topic, int rate, osgOcean::OceanScene* ptrOcean);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~OceanSurfaceToROSOceanVehicle();
};

class WorldToROSTF : public ROSPublisherInterface
{
  std::vector< osg::ref_ptr<osg::MatrixTransform> > transforms_;
  std::vector< boost::shared_ptr<robot_state_publisher::RobotStatePublisher> > robot_pubs_;
  boost::shared_ptr<tf::TransformBroadcaster> tfpub_;
  std::string worldRootName_; 
  unsigned int enableObjects_;
  SceneBuilder * scene;
public:
  WorldToROSTF(SceneBuilder * scene, std::string worldRootName, unsigned int enableObjects, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~WorldToROSTF();
};

class ImuToROSImu : public ROSPublisherInterface
{
  InertialMeasurementUnit *imu_;

public:
  ImuToROSImu(InertialMeasurementUnit *imu, std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~ImuToROSImu();
};

class PressureSensorToROS : public ROSPublisherInterface
{
  PressureSensor *sensor_;

public:
  PressureSensorToROS(PressureSensor *sensor, std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~PressureSensorToROS();
};

class GPSSensorToROS : public ROSPublisherInterface
{
  GPSSensor *sensor_;

public:
  GPSSensorToROS(GPSSensor *sensor, std::string topic, int rate) :
      ROSPublisherInterface(topic, rate), sensor_(sensor)
  {
  }

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~GPSSensorToROS()
  {
  }
};

class DVLSensorToROS : public ROSPublisherInterface
{
  DVLSensor *sensor_;

public:
  DVLSensorToROS(DVLSensor *sensor, std::string topic, int rate) :
      ROSPublisherInterface(topic, rate), sensor_(sensor)
  {
  }

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~DVLSensorToROS()
  {
  }
};

class ArmToROSJointState : public ROSPublisherInterface
{
  boost::shared_ptr<URDFRobot> arm;
public:
  ArmToROSJointState(SimulatedIAUV *arm, std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~ArmToROSJointState();
};

class VirtualCameraToROSImage : public ROSPublisherInterface
{

  //Updates camera buffer when publisher is not publishing
  class CameraBufferCallback : public osg::Camera::DrawCallback
  {
    public:
      virtual void operator () (const osg::Camera& camera) const;
      CameraBufferCallback(VirtualCameraToROSImage * publisher,VirtualCamera *camera,int depth);
    private:

      VirtualCameraToROSImage * pub;
      VirtualCamera *cam;
      int depth;
  };

  boost::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Publisher img_pub_;
  std::string image_topic;
  VirtualCamera *cam;
  int depth;
  int bw;
public:

  VirtualCameraToROSImage(VirtualCamera *camera, std::string topic, std::string info_topic, int rate, int depth);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~VirtualCameraToROSImage();

  osg::ref_ptr < osg::Image > osgimage;
  OpenThreads::Mutex mutex; //Mutex to avoid image overwriting

};

class RangeCameraToPCL : public ROSPublisherInterface
{
  VirtualCamera *cam;

public:

  RangeCameraToPCL(VirtualCamera *camera, std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~RangeCameraToPCL();
};

class RangeSensorToROSRange : public ROSPublisherInterface
{
  VirtualRangeSensor *rs;
public:
  RangeSensorToROSRange(VirtualRangeSensor *rangesensor, std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~RangeSensorToROSRange();
};

class MultibeamSensorToROS : public ROSPublisherInterface
{
  MultibeamSensor *MB;
public:
  MultibeamSensorToROS(MultibeamSensor *multibeamSensor, std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~MultibeamSensorToROS();
};

class contactSensorToROS : public ROSPublisherInterface
{
  BulletPhysics * physics;
  std::string target;
  osg::Group *rootNode;
public:
  contactSensorToROS(osg::Group *rootNode, BulletPhysics * physics, std::string target, std::string topic, int rate);

  void createPublisher(ros::NodeHandle &nh);

  void publish();

  ~contactSensorToROS();
};
#endif

