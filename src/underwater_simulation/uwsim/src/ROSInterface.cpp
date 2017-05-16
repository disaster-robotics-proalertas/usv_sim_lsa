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

#include <uwsim/ROSInterface.h>
#include <uwsim/UWSimUtils.h>
#include <osg/LineWidth>
#include <osg/Material>
#include <osgOcean/ShaderManager>
#include <uwsim/osgPCDLoader.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <underwater_sensor_msgs/Pressure.h>
#include <underwater_sensor_msgs/DVL.h>
#include <std_msgs/Bool.h>
#include <osg/LineStipple>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

// static member
ros::Time ROSInterface::current_time_;

ROSSubscriberInterface::ROSSubscriberInterface(std::string topic) :
    ROSInterface(topic)
{
  startThread();
}

/* Thread code */
void ROSSubscriberInterface::run()
{
  ros::Duration(2).sleep();
  createSubscriber (nh_);
}

ROSSubscriberInterface::~ROSSubscriberInterface()
{
  join();
}

ROSOdomToPAT::ROSOdomToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName) :
    ROSSubscriberInterface(topic)
{
  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first = findNode.getFirst();
  if (first == NULL)
  {
    transform = NULL;
  }
  else
  {
    transform = dynamic_cast<osg::MatrixTransform*>(first);
  }
  started = 0; //Used in time
}

void ROSOdomToPAT::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSOdomToPAT subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe < nav_msgs::Odometry > (topic, 10, &ROSOdomToPAT::processData, this);
  if (sub_ == ros::Subscriber())
  {
    ROS_ERROR("ROSOdomToPAT::createSubscriber cannot subscribe to topic %s", topic.c_str());
  }
}

void ROSOdomToPAT::processData(const nav_msgs::Odometry::ConstPtr& odom)
{
  if (transform != NULL)
  {
    //vpHomogeneousMatrix sMsv;
    osg::Matrixd sMsv_osg;
    //If velocity is zero, use the position reference. If not, use velocity
    //If position is not zero, use the position reference. If not, use velocity
    if (odom->pose.pose.orientation.x != 0 || odom->pose.pose.orientation.y != 0 || odom->pose.pose.orientation.z != 0
        || odom->pose.pose.position.x != 0 || odom->pose.pose.position.y != 0 || odom->pose.pose.position.z != 0)
    {
      sMsv_osg.setTrans(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
      sMsv_osg.setRotate(
          osg::Quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                    odom->pose.pose.orientation.w));
      sMsv_osg.preMultScale(transform->getMatrix().getScale());
    }
    else
    {
      //Get the current vehicle position and attitude in an homogeneous matrix
      sMsv_osg = transform->getMatrix();

      //Time issues
      double elapsed = 0;
      if (started != 0)
      {
        ros::WallDuration t_diff = ros::WallTime::now() - last;
        elapsed = t_diff.toSec();
        //If elapsed>MAX_ELAPSED, consider this is sent by a different publisher, so that the counter has to restart
        if (elapsed > 1)
          elapsed = 0;
      }
      started = 1;
      last = ros::WallTime::now();
      //std::cout<<elapsed<<std::endl;

      osg::Matrixd vMnv;
      osg::Matrixd T, Rx, Ry, Rz;
      T.makeTranslate(odom->twist.twist.linear.x * elapsed, odom->twist.twist.linear.y * elapsed,
                      odom->twist.twist.linear.z * elapsed);
      Rx.makeRotate(odom->twist.twist.angular.x * elapsed, 1, 0, 0);
      Ry.makeRotate(odom->twist.twist.angular.y * elapsed, 0, 1, 0);
      Rz.makeRotate(odom->twist.twist.angular.z * elapsed, 0, 0, 1);
      vMnv = Rz * Ry * Rx * T;

      sMsv_osg = vMnv * sMsv_osg;
    }
    transform->setMatrix(sMsv_osg);
  }
}

ROSOdomToPAT::~ROSOdomToPAT()
{
}

ROSTwistToPAT::ROSTwistToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName) :
    ROSSubscriberInterface(topic)
{
  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first = findNode.getFirst();
  if (first == NULL)
  {
    transform = NULL;
  }
  else
  {
    transform = dynamic_cast<osg::MatrixTransform*>(first);
  }
  started = 0; //used in time
}

void ROSTwistToPAT::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSTwistToPAT subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe < geometry_msgs::TwistStamped > (topic, 10, &ROSTwistToPAT::processData, this);
}

void ROSTwistToPAT::processData(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
  if (transform != NULL)
  {
    //vpHomogeneousMatrix sMsv;
    osg::Matrixd sMsv_osg;
    //Get the current vehicle position and attitude in an homogeneous matrix
    sMsv_osg = transform->getMatrix();

    //Time issues
    double elapsed = 0;
    if (started != 0)
    {
      ros::WallDuration t_diff = ros::WallTime::now() - last;
      elapsed = t_diff.toSec();
      //If elapsed>MAX_ELAPSED, consider this is sent by a different publisher, so that the counter has to restart
      if (elapsed > 1)
        elapsed = 0;
    }
    started = 1;
    last = ros::WallTime::now();

    osg::Matrixd vMnv;
    osg::Matrixd T, Rx, Ry, Rz;
    T.makeTranslate(twist->twist.linear.x * elapsed, twist->twist.linear.y * elapsed, twist->twist.linear.z * elapsed);
    Rx.makeRotate(twist->twist.angular.x * elapsed, 1, 0, 0);
    Ry.makeRotate(twist->twist.angular.y * elapsed, 0, 1, 0);
    Rz.makeRotate(twist->twist.angular.z * elapsed, 0, 0, 1);
    vMnv = Rz * Ry * Rx * T;

    sMsv_osg = vMnv * sMsv_osg;

    transform->setMatrix(sMsv_osg);
  }
}

ROSTwistToPAT::~ROSTwistToPAT()
{
}

ROSPoseToPAT::ROSPoseToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName) :
    ROSSubscriberInterface(topic)
{
  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first = findNode.getFirst();
  if (first == NULL)
  {
    transform = NULL;
  }
  else
  {
    transform = dynamic_cast<osg::MatrixTransform*>(first);
  }
}

void ROSPoseToPAT::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSPoseToPAT subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe < geometry_msgs::Pose > (topic, 10, &ROSPoseToPAT::processData, this);
}

void ROSPoseToPAT::processData(const geometry_msgs::Pose::ConstPtr& pose)
{
  if (transform != NULL)
  {
    //vpHomogeneousMatrix sMsv;
    osg::Matrixd sMsv_osg;

    sMsv_osg.setTrans(pose->position.x, pose->position.y, pose->position.z);
    sMsv_osg.setRotate(osg::Quat(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w));
    sMsv_osg.preMultScale(transform->getMatrix().getScale());

    transform->setMatrix(sMsv_osg);

  }
}

ROSPoseToPAT::~ROSPoseToPAT()
{
}

ROSPointCloudLoader::ROSPointCloudLoader(std::string topic, osg::ref_ptr<osg::Group> root,unsigned int mask,bool del)
: ROSSubscriberInterface(topic), scene_root(root), nodeMask(mask), deleteLastPCD(del)
{
  
}

void ROSPointCloudLoader::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSPointCloudLoader subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe(topic, 10, &ROSPointCloudLoader::processData, this);
}

void ROSPointCloudLoader::processData(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colour(new pcl::PointCloud<pcl::PointXYZRGB>);
  // I assume that 4 fields is always X Y Z RGB
  if( msg->fields.size() != 4 ){
    pcl::PointCloud<pcl::PointXYZ>::Ptr original(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *original);
    colourCloudDepth( original, colour );
    }else{
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*msg, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, *colour);
    }

   osgPCDLoader<pcl::PointXYZRGB> pcdLoader(*colour.get());

   osg::ref_ptr < osg::Node > frame_id=findRN(msg->header.frame_id,scene_root);

   if(frame_id)
   {
     osg::ref_ptr < osg::Node > LWNode=findRN("localizedWorld",scene_root);
     boost::shared_ptr<osg::Matrix> LWMat=getWorldCoords(LWNode);
     LWMat->invert(*LWMat);

     boost::shared_ptr<osg::Matrix> WorldToBase=getWorldCoords(frame_id);

     osg::Matrixd  res=*WorldToBase * *LWMat;
     osg::ref_ptr < osg::MatrixTransform > WorldToBaseTransform= new osg::MatrixTransform(res);
     WorldToBaseTransform->addChild(pcdLoader.getGeode());

     pcdLoader.getGeode()->setNodeMask(nodeMask);
     LWNode->asGroup()->addChild(WorldToBaseTransform);
     if(deleteLastPCD)
     {
       LWNode->asGroup()->removeChild(lastPCD);
       lastPCD=WorldToBaseTransform;
     }
  }
  else
  {
    ROS_WARN ("%s is not a valid frame id for PointCloudLoader.",msg->header.frame_id.c_str());
  }
}

/*
  ColorCloudDepth returns an RGB cloud coloured with depth with matlab's jet color scale
  */
void ROSPointCloudLoader::colourCloudDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

  //COnverts in unorganized...
  cloud->header = cloudIn->header;
  // Get Max and Min
  double mx(-10000000),mn(10000000000),mean;
  bool look_for_min_z(true);
  bool look_for_max_z(true);

  for (int k=0;k<cloudIn->size();k++){
    if (mx<cloudIn->points[k].z && look_for_max_z) mx = cloudIn->points[k].z;
    if (mn>cloudIn->points[k].z && look_for_min_z) mn = cloudIn->points[k].z;
  }
  mean = (mx+mn)/2;

  // Compute Color
  uint8_t r, g, b;
  for (int k=0;k<cloudIn->size();k++){
    pcl::PointXYZRGB point;
    point.x = cloudIn->points[k].x;
    point.y = cloudIn->points[k].y;
    point.z = cloudIn->points[k].z;
    double z = (point.z - mn)/(mx-mn) * 2 - 1;
    r = (int) (base(z - 0.5) * 255);
    g = (int) (base(z) * 255);
    b = (int) (base(z + 0.5) * 255);

    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

    point.rgb = *reinterpret_cast<float*>(&rgb);
    cloud->push_back(point);
  }
}

/*
  Interpolate interpolates a value, used to colorize cloud
  */
double ROSPointCloudLoader::interpolate(double val, double y0, double x0, double y1, double x1){
  return (val - x0)*(y1-y0)/(x1-x0) + y0;
}

double ROSPointCloudLoader::base(double val){
  if (val <= -0.75) return 0;
  else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
  else if (val <= 0.25) return 1;
  else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
  else return 0;
}

ROSPointCloudLoader::~ROSPointCloudLoader(){}

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

ROSJointStateToArm::ROSJointStateToArm(std::string topic, boost::shared_ptr<SimulatedIAUV> arm) :
    ROSSubscriberInterface(topic)
{
  this->arm = arm;
}

void ROSJointStateToArm::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSJointStateToArm subscriber on topic %s", topic.c_str());
  sub_ = nh.subscribe < sensor_msgs::JointState > (topic, 10, &ROSJointStateToArm::processData, this);
}

void ROSJointStateToArm::processData(const sensor_msgs::JointState::ConstPtr& js)
{
  //Receive request from client
  if (js->position.size() != 0)
  {
    //position command
    std::vector < std::string > names = js->name;
    std::vector<double> position = js->position;
    arm->urdf->setJointPosition(position, names);
  }
  else if (js->velocity.size() != 0)
  {
    //velocity command
    std::vector < std::string > names = js->name;
    std::vector<double> velocity = js->velocity;
    arm->urdf->setJointVelocity(velocity, names);
  }
}

ROSJointStateToArm::~ROSJointStateToArm()
{
}

ROSImageToHUDCamera::ROSImageToHUDCamera(std::string topic, std::string info_topic, boost::shared_ptr<HUDCamera> camera) :
    ROSSubscriberInterface(info_topic), cam(camera), image_topic(topic)
{
}

void ROSImageToHUDCamera::createSubscriber(ros::NodeHandle &nh)
{
  ROS_INFO("ROSImageToHUDCamera subscriber on topic %s", topic.c_str());
  it.reset(new image_transport::ImageTransport(nh));
  OSG_DEBUG << "ROSImageToHUDCamera::createSubscriber Subscribing to image topic " << image_topic << std::endl;
  image_sub = it->subscribe(image_topic, 1, &ROSImageToHUDCamera::processData, this);
  //OSG_INFO << "ROSCamera::ROSCamera Subscribing to camera info topic " << info_topic << std::endl;
  //sub_=nh_.subscribe<sensor_msgs::CameraInfo>(info_topic, 1, &ROSImageToHUDCamera::imageInfoCallback, this);
}

void ROSImageToHUDCamera::processData(const sensor_msgs::ImageConstPtr& msg)
{
  OSG_DEBUG << "ROSImageToHUDCamera::imageCallback start: " << msg->width << "x" << msg->height << " step:" << msg->step
      << std::endl;

  //unsigned char *msgdata=(unsigned char*)&(msg->data[0]);
  //osg_image->setImage(width,height,1,0,GL_RGB,GL_UNSIGNED_BYTE,msgdata,osg::Image::NO_DELETE);
  char *osgimage_data = (char*)cam->osg_image->data();
  //Memory cannot be directly copied, since the image frame used in OpenSceneGraph (OpenGL glReadPixels) is on
  //the bottom-left looking towards up-right, whereas ROS sensor_msgs::Image::data expects origin on top-left
  //looking towards bottom-right. Therefore it must be manually arranged, although this could be much improved:
  for (unsigned int i = 0; i < msg->height; i++)
  {
    for (unsigned int j = 0; j < msg->step; j++)
    {
      osgimage_data[i * msg->step + j] = msg->data[(msg->height - i - 1) * msg->step + j];
    }
  }
  cam->ready_ = true;
  OSG_DEBUG << "ROSImageToHUDCamera::imageCallback exit" << std::endl;
}

ROSImageToHUDCamera::~ROSImageToHUDCamera()
{
}

ROSPublisherInterface::ROSPublisherInterface(std::string topic, int publish_rate) :
    ROSInterface(topic)
{
  this->publish_rate = publish_rate;
  OSG_DEBUG << "ROSPublisherInterface Thread starting..." << topic << std::endl;
  startThread();
  OSG_DEBUG << "ROSPublisherInterface Thread created" << std::endl;
}

/* Thread code */
void ROSPublisherInterface::run()
{
  ros::Duration(2).sleep();
  createPublisher (nh_);

  ros::Rate rate(publish_rate);
  while (ros::ok())
  {
    publish();

    rate.sleep();
  }
}

ROSPublisherInterface::~ROSPublisherInterface()
{
  join();
}

PATToROSOdom::PATToROSOdom(osg::Group *rootNode, std::string vehicleName, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate)
{

  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first = findNode.getFirst();
  if (first == NULL)
  {
    transform = NULL;
  }
  else
  {
    transform = dynamic_cast<osg::MatrixTransform*>(first);
  }
}

void PATToROSOdom::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("PATToROSOdom publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < nav_msgs::Odometry > (topic, 1);
}

void PATToROSOdom::publish()
{
  if (transform != NULL)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = getROSTime();

    osg::Matrixd mat = transform->getMatrix();
    osg::Vec3d pos = mat.getTrans();
    osg::Quat rot = mat.getRotate();

    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    odom.pose.pose.orientation.x = rot.x();
    odom.pose.pose.orientation.y = rot.y();
    odom.pose.pose.orientation.z = rot.z();
    odom.pose.pose.orientation.w = rot.w();

    //twist and covariances not implemented at the moment
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;
    for (int i = 0; i < 36; i++)
    {
      odom.twist.covariance[i] = 0;
      odom.pose.covariance[i] = 0;
    }

    pub_.publish(odom);
  }
}

PATToROSOdom::~PATToROSOdom()
{
}

ImuToROSImu::ImuToROSImu(InertialMeasurementUnit *imu, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), imu_(imu)
{
}

void ImuToROSImu::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("Imu publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::Imu > (topic, 1);
}

void ImuToROSImu::publish()
{
  if (imu_ != NULL)
  {
    osg::Quat rot = imu_->getMeasurement();

    sensor_msgs::Imu imu;
    imu.header.stamp = getROSTime();
    imu.header.frame_id = "world";
    imu.orientation.x = rot.x();
    imu.orientation.y = rot.y();
    imu.orientation.z = rot.z();
    imu.orientation.w = rot.w();

    imu.orientation_covariance[0] = imu.orientation_covariance[4] = imu.orientation_covariance[8] = std::pow(
        imu_->getStandardDeviation(), 2);

    pub_.publish(imu);
  }
}

ImuToROSImu::~ImuToROSImu()
{
}

PressureSensorToROS::PressureSensorToROS(PressureSensor *sensor, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), sensor_(sensor)
{
}

void PressureSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("PressureSensor publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < underwater_sensor_msgs::Pressure > (topic, 1);
}

void PressureSensorToROS::publish()
{
  if (sensor_ != NULL)
  {
    double pressure = sensor_->getMeasurement();

    underwater_sensor_msgs::Pressure v;
    v.pressure = pressure;
    v.header.stamp = getROSTime();
    v.header.frame_id = "world";

    pub_.publish(v);
  }
}

PressureSensorToROS::~PressureSensorToROS()
{
}

void GPSSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("GPSSensor publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::NavSatFix > (topic, 1);
}

void GPSSensorToROS::publish()
{
  if (sensor_ != NULL)
  {
    osg::Vec3d wTgps = sensor_->getMeasurement();

    //publish only if near to the ocean surface
    if (sensor_->depthBelowWater() < 0.5)
    {
      sensor_msgs::NavSatFix m;
      m.latitude = wTgps[0];
      m.longitude = wTgps[1];
      m.position_covariance[0] = m.position_covariance[4] = m.position_covariance[8] = std::pow(
          sensor_->getStandardDeviation(), 2);
      m.position_covariance_type = m.COVARIANCE_TYPE_DIAGONAL_KNOWN;

      pub_.publish(m);
    }
  }
}

void DVLSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("DVLSensor publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < underwater_sensor_msgs::DVL > (topic, 1);
}

void DVLSensorToROS::publish()
{
  if (sensor_ != NULL)
  {
    osg::Vec3d vdvl = sensor_->getMeasurement();

    underwater_sensor_msgs::DVL m;
    m.bi_x_axis = vdvl.x();
    m.bi_y_axis = vdvl.y();
    m.bi_z_axis = vdvl.z();

    pub_.publish(m);
  }
}

ArmToROSJointState::ArmToROSJointState(SimulatedIAUV *arm, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate)
{
  this->arm = arm->urdf;
}

void ArmToROSJointState::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("ArmToROSJointState publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::JointState > (topic, 1);
}

void ArmToROSJointState::publish()
{
  if (arm != NULL)
  {
    sensor_msgs::JointState js;
    js.header.stamp = getROSTime();
    std::vector<double> q = arm->getJointPosition();
    std::vector<std::string> names=arm->getJointName();
    for (size_t i = 0; i < q.size(); i++)
    {
      js.name.push_back(names[i]);
      js.position.push_back(q[i]);
      js.effort.push_back(0);
    }

    pub_.publish(js);
  }
}

ArmToROSJointState::~ArmToROSJointState()
{
}

void VirtualCameraToROSImage::CameraBufferCallback::operator () (const osg::Camera& camera) const
{
  if(pub)
  {
    pub->mutex.lock();
    if (depth)
    {
      pub->osgimage = new osg::Image(*cam->depthTexture.get());
    }
    else
    {
     pub->osgimage = new osg::Image(*cam->renderTexture.get());
    }
    pub->mutex.unlock();
  }
}

VirtualCameraToROSImage::CameraBufferCallback::CameraBufferCallback(VirtualCameraToROSImage * publisher,VirtualCamera *camera,int depth)
{
  pub = publisher;
  cam=camera; //We could read data from cameracb, but it is already on virtualCamera.
  this->depth=depth;
}

VirtualCameraToROSImage::VirtualCameraToROSImage(VirtualCamera *camera, std::string topic, std::string info_topic,
                                                 int rate, int depth) :
    ROSPublisherInterface(info_topic, rate), cam(camera), image_topic(topic)
{
  it.reset(new image_transport::ImageTransport(nh_));
  this->depth = depth;
  this->bw = camera->bw;
  CameraBufferCallback * buffercb = new  CameraBufferCallback(this,cam,depth); 
  cam->textureCamera->setPostDrawCallback(buffercb); 
}

void VirtualCameraToROSImage::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("VirtualCameraToROSImage publisher on topic %s", topic.c_str());
  while (!it)
  {
    ROS_INFO("VirtualCameraToROSImage Waiting for transport to be initialized...");
  }
  img_pub_ = it->advertise(image_topic, 1);
  pub_ = nh.advertise < sensor_msgs::CameraInfo > (topic, 1);
}

void VirtualCameraToROSImage::publish()
{
  //OSG_DEBUG << "OSGImageToROSImage::publish entering" << std::endl;
  if (osgimage != NULL && osgimage->getTotalSizeInBytes() != 0)
  {
    //OSG_DEBUG << "\t image size: " << cam->renderTexture->s() << " " << cam->renderTexture->t() << " " << cam->renderTexture->getTotalSizeInBytes() << std::endl;
    int w, h, d;
    w = osgimage->s();
    h = osgimage->t();
    d = osgimage->getTotalSizeInBytes();

    if (d != 0)
    {
      sensor_msgs::Image img;
      sensor_msgs::CameraInfo img_info;
      img_info.header.stamp = img.header.stamp = getROSTime();
      img_info.header.frame_id = img.header.frame_id = cam->frameId;
      if (depth)
        img.encoding = std::string("32FC1");
      else if (bw)
        img.encoding = std::string("mono8");
      else
        img.encoding = std::string("rgb8");

      img.is_bigendian = 0;
      img.height = h;
      img.width = w;
      if (bw)
        d /=3;
      img.step = d / h;
      img.data.resize(d);
      img_info.width = w;
      img_info.height = h;

      img_info.D.resize(4, 0.0);

      img_info.R[0] = 1.0;
      img_info.R[4] = 1.0;
      img_info.R[8] = 1.0;

      img_info.K[0] = cam->fx;
      img_info.K[2] = cam->cx;
      img_info.K[4] = cam->fy;
      img_info.K[5] = cam->cy;
      img_info.K[8] = 1;

      img_info.P[0] = cam->fx;
      img_info.P[2] = cam->cx;
      img_info.P[3] = cam->Tx;
      img_info.P[5] = cam->fy;
      img_info.P[6] = cam->cy;
      img_info.P[7] = cam->Ty;
      img_info.P[10] = 1;

      img_info.roi.x_offset = 0;
      img_info.roi.y_offset = 0;
      img_info.roi.height = img_info.height;
      img_info.roi.width = img_info.width;
      img_info.roi.do_rectify = false;

      img_info.binning_x = 0;
      img_info.binning_y = 0;

      double fov, aspect, near, far;
      cam->textureCamera->getProjectionMatrixAsPerspective(fov, aspect, near, far);
      double a = far / (far - near);
      double b = (far * near) / (near - far);

      //img_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

      mutex.lock();
      unsigned char *virtualdata = (unsigned char*)osgimage->data();
      //memcpy(&(img.data.front()),virtualdata,d*sizeof(char));
      //Memory cannot be directly copied, since the image frame used in OpenSceneGraph (OpenGL glReadPixels) is on
      //the bottom-left looking towards up-right, whereas ROS sensor_msgs::Image::data expects origin on top-left
      //looking towards bottom-right. Therefore it must be manually arranged.
      if (virtualdata != NULL)
        for (int i = 0; i < h; i++)
        {
          unsigned char *srcRow = virtualdata + i * img.step*(bw?3:1);
          unsigned char *dstRow = img.data.data() + (h - i - 1) * img.step;
          for (unsigned int j = 0;
              (!depth && j < img.step) || (depth && j < w); j++)
          {
            if (bw)
            {
              dstRow[j] = srcRow[j * 3] * 0.2989;
              dstRow[j] += srcRow[j * 3 + 1] * 0.5870;
              dstRow[j] += srcRow[j * 3 + 2] * 0.1140;
            }
            else if (depth)
            {
              float D = b / (((float*)srcRow)[j] - a);
              if (D == far)
                D = INFINITY;
              else if (D == near)
                D = -INFINITY;
              else if (!isfinite(D) || D < near || D > far)
                D = NAN;
              ((float*)dstRow)[j] = D;
            }
            else
              dstRow[j] = srcRow[j];
          }
        }
      else
        memset(&(img.data.front()), 0, d);
      mutex.unlock();

      img_pub_.publish(img);
      pub_.publish(img_info);
    }
  }
  //OSG_DEBUG << "OSGImageToROSImage::publish exit" << std::endl;
}

VirtualCameraToROSImage::~VirtualCameraToROSImage()
{
}

RangeCameraToPCL::RangeCameraToPCL(VirtualCamera *camera, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), cam(camera)
{
}

void RangeCameraToPCL::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("RangeCameraToPCL publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < pcl::PointCloud<pcl::PointXYZ> > (topic, 1);
}

void RangeCameraToPCL::publish()
{      
  double fov, aspect, near, far;
  int w, h, d;
  float * data = (float *)cam->depthTexture->data();

  if (data != NULL)
  {

    w = cam->width;
    h = cam->height;

    cam->textureCamera->getProjectionMatrixAsPerspective(fov, aspect, near, far);
    double a = far / (far - near);
    double b = (far * near) / (near - far);
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
    msg->header.frame_id = cam->name;

    //Rotations to transform from OSG to TF conventions are already taken into account
    for (int i = 0; i < h; i++)
    {
      for (int j = 0; j < w; j++)
      {
	double depth=  (b / (data[i*w+j] - a)) ;
	msg->points.push_back (pcl::PointXYZ( (j-cam->cx)/cam->fx * depth , -(i-cam->cy)/cam->fy * depth, depth));

      }
    }

    //msg->header.stamp = getROSTime();
    pub_.publish (msg);

  }
}

RangeCameraToPCL::~RangeCameraToPCL()
{
}

RangeSensorToROSRange::RangeSensorToROSRange(VirtualRangeSensor *rangesensor, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), rs(rangesensor)
{
}

void RangeSensorToROSRange::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("RangeSensorToROSRange publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::Range > (topic, 1);
}

void RangeSensorToROSRange::publish()
{
  if (rs != NULL)
  {
    sensor_msgs::Range r;
    r.header.stamp = getROSTime();
    r.radiation_type = sensor_msgs::Range::ULTRASOUND;
    r.field_of_view = 0; //X axis of the sensor
    r.min_range = 0;
    r.max_range = rs->range;
    r.range = (rs->node_tracker != NULL) ? rs->node_tracker->distance_to_obstacle : r.max_range;

    pub_.publish(r);
  }
}

RangeSensorToROSRange::~RangeSensorToROSRange()
{
}

MultibeamSensorToROS::MultibeamSensorToROS(MultibeamSensor *multibeamSensor, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), MB(multibeamSensor)
{
}

void MultibeamSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO(" MultibeamSensorToROS publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < sensor_msgs::LaserScan > (topic, 1);
}

void MultibeamSensorToROS::publish()
{
  if (MB != NULL)
  {
    sensor_msgs::LaserScan ls;
    ls.header.stamp = getROSTime();
    ls.header.frame_id = MB->name;

    double fov, aspect, near, far;

    ls.range_min = near;
    ls.range_max = MB->range; //far plane should be higher (z-buffer resolution)
    ls.angle_min = MB->initAngle * M_PI / 180;
    ls.angle_max = MB->finalAngle * M_PI / 180;
    ls.angle_increment = MB->angleIncr * M_PI / 180;

    std::vector<double> tmp;
    tmp.resize(MB->camPixels*MB->nCams);
    for(unsigned int j=0; j<MB->nCams ;j++)
    {
      MB->vcams[j].textureCamera->getProjectionMatrixAsPerspective(fov, aspect, near, far);

      float * data = (float *)MB->vcams[j].depthTexture->data();
      double a = far / (far - near);
      double b = (far * near) / (near - far);

      for (int i = 0; i < MB->camPixels; i++)
      {
        double Z = (data[i]); ///4294967296.0;
        tmp[i+MB->camPixels*j] = b / (Z - a);
      }
    }

    ls.ranges.resize(MB->numpixels);
    for (int i = 0; i < MB->numpixels; i++)
    {
      ls.ranges[i] = (tmp[MB->remapVector[i].pixel1] * MB->remapVector[i].weight1
          + tmp[MB->remapVector[i].pixel2] * MB->remapVector[i].weight2) * MB->remapVector[i].distort;
      if (ls.ranges[i] > MB->range)
        ls.ranges[i] = MB->range;
    }

    pub_.publish(ls);
  }
}

MultibeamSensorToROS::~MultibeamSensorToROS()
{
}

contactSensorToROS::contactSensorToROS(osg::Group *rootNode, BulletPhysics * physics, std::string target,
                                       std::string topic, int rate) :
    ROSPublisherInterface(topic, rate)
{
  this->rootNode = rootNode;
  this->physics = physics;
  this->target = target;
}

void contactSensorToROS::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("contactSensorToROS publisher on topic %s", topic.c_str());
  pub_ = nh.advertise < std_msgs::Bool > (topic, 1);
}

void contactSensorToROS::publish()
{
  int colliding = 0;

  for (int i = 0; i < physics->getNumCollisions(); i++)
  {
    btPersistentManifold * col = physics->getCollision(i);

    #if BT_BULLET_VERSION <= 279
    //Get objects colliding
    btRigidBody* obA = static_cast<btRigidBody*>(col->getBody0());
    btRigidBody* obB = static_cast<btRigidBody*>(col->getBody1());

    //Check if target is involved in collision
    CollisionDataType * data = (CollisionDataType *)obA->getUserPointer();
    CollisionDataType * data2 = (CollisionDataType *)obB->getUserPointer();

    #else
    CollisionDataType * data = (CollisionDataType *)col->getBody0()->getUserPointer();
    CollisionDataType * data2 = (CollisionDataType *)col->getBody0()->getUserPointer();
    #endif

    int numContacts = col->getNumContacts();

    if (data2->getObjectName() == target || data->getObjectName() == target)
    {
      for (int j = 0; j < numContacts; j++)
      {
        btManifoldPoint pt = col->getContactPoint(j);
        if (pt.getDistance() < 0.f)
          colliding = 1;
      }
    }

  }
  std_msgs::Bool msg;
  msg.data = colliding;
  pub_.publish(msg);
}

contactSensorToROS::~contactSensorToROS()
{
}


WorldToROSTF::WorldToROSTF(  SceneBuilder * scene, std::string worldRootName, unsigned int enableObjects, int rate ) :
    ROSPublisherInterface(worldRootName, rate)
{
   this->scene=scene;
   for(int i = 0; i < scene->iauvFile.size(); i++)
   {
      KDL::Tree tree;
      if (!kdl_parser::treeFromFile(scene->iauvFile[i].get()->urdf->URDFFile, tree))
      {
         ROS_ERROR("Failed to construct kdl tree");
      }
      else
      {
         ROS_INFO("Loaded tree, %d segments, %d joints", tree.getNrOfSegments(), tree.getNrOfJoints());
      }
      
      osg::ref_ptr<osg::MatrixTransform> transform;
      robot_pubs_.push_back(boost::shared_ptr<robot_state_publisher::RobotStatePublisher>(
       new robot_state_publisher::RobotStatePublisher(tree)));
  
      findNodeVisitor findNode(scene->iauvFile[i].get()->name);
      scene->root->accept(findNode);
      osg::Node *first = findNode.getFirst();
      if (first == NULL)
      {
         transform = NULL;
      }
      else
      {
         transform = dynamic_cast<osg::MatrixTransform*>(first);
      }
      transforms_.push_back(transform);
   }
   worldRootName_ = worldRootName;
   enableObjects_ = enableObjects;
}

void WorldToROSTF::createPublisher(ros::NodeHandle &nh)
{   
   tfpub_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
}

void WorldToROSTF::publish()
{

   //Publish vehicle frames
   for( int i = 0; i < scene->iauvFile.size(); i++ )
   {
      // Publish moving joints
      robot_pubs_[i]->RobotStatePublisher::publishTransforms(scene->iauvFile[i].get()->urdf->getFullJointMap(), getROSTime(), scene->iauvFile[i].get()->name);
      // Publish fixed joints
      robot_pubs_[i]->RobotStatePublisher::publishFixedTransforms(scene->iauvFile[i].get()->name,true);
      //Publish odometry
      if (transforms_[i] != NULL)
      {
         osg::Matrixd mat = transforms_[i]->getMatrix();
         osg::Vec3d pos = mat.getTrans();
         osg::Quat rot = mat.getRotate();

         tf::Vector3 p(pos.x(), pos.y(), pos.z());
         tf::Quaternion q(rot.x(), rot.y(), rot.z(), rot.w());
         tf::Pose pose(q,p);
         tf::StampedTransform t(pose, getROSTime(), "/" + worldRootName_, "/"+scene->iauvFile[i].get()->name);
      
         tf::Vector3 p2(0, 0, 0);
         tf::Quaternion q2(0, 0, 0, 1);
         tf::Pose pose2(q2,p2);
         tf::StampedTransform t2(pose2, getROSTime(), "/"+scene->iauvFile[i].get()->name, "/"+scene->iauvFile[i].get()->name + "/base_link");

         tfpub_->sendTransform(t2);
         tfpub_->sendTransform(t);  
      }

      //publish Cameras
      for(int j=0; j< scene->iauvFile[i].get()->camview.size();j++)
      {
        tf::Pose pose;
        std::string parent;
        tf::Pose OSGToTFconvention;
        if(scene->iauvFile[i].get()->camview[j].getTFTransform(pose,parent))
        {
          OSGToTFconvention.setOrigin(tf::Vector3(0,0,0));
          OSGToTFconvention.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI));//OSG convention is different to tf:
          //Remember that in opengl/osg, the camera frame is a right-handed system with Z going backwards (opposite to the viewing direction) and Y up.
          //While in tf convention, the camera frame is a right-handed system with Z going forward (in the viewing direction) and Y down.

          int multibeam=false;
          for(int k=0;k<scene->iauvFile[i].get()->multibeam_sensors.size();k++) //check if camera comes from multibeam
	    if(scene->iauvFile[i].get()->multibeam_sensors[k].name==scene->iauvFile[i].get()->camview[j].name)
              multibeam=true;
              //OSGToTFconvention.setRotation(tf::Quaternion(tf::Vector3(0,1,0),M_PI/2));  //As we are using camera to simulate it, we need to rotate it

          if(!multibeam){
            pose=pose*OSGToTFconvention;
            tf::StampedTransform t(pose, getROSTime(),   "/"+scene->iauvFile[i].get()->name + "/" +parent, scene->iauvFile[i].get()->camview[j].name);
            tfpub_->sendTransform(t);
          }
        }  
      }

      //publish multibeams
      for(int j=0; j< scene->iauvFile[i].get()->multibeam_sensors.size();j++)
      {
        tf::Pose pose;
        std::string parent;
        if(scene->iauvFile[i].get()->multibeam_sensors[j].getTFTransform(pose,parent))
        {
          tf::StampedTransform t(pose, getROSTime(),   "/"+scene->iauvFile[i].get()->name + "/" +parent, scene->iauvFile[i].get()->multibeam_sensors[j].name);
          tfpub_->sendTransform(t);
        }  
      }


      //publish imus
      for(int j=0; j< scene->iauvFile[i].get()->imus.size();j++)
      {
        tf::Pose pose;
        std::string parent;
        if(scene->iauvFile[i].get()->imus[j].getTFTransform(pose,parent))
        {
          tf::StampedTransform t(pose, getROSTime(),   "/"+scene->iauvFile[i].get()->name + "/" +parent, scene->iauvFile[i].get()->imus[j].name);
          tfpub_->sendTransform(t);
        }  
      }

      //publish RangeSensor
      for(int j=0; j< scene->iauvFile[i].get()->range_sensors.size();j++)
      {
        tf::Pose pose;
        std::string parent;
        if(scene->iauvFile[i].get()->range_sensors[j].getTFTransform(pose,parent))
        {
          tf::StampedTransform t(pose, getROSTime(),   "/"+scene->iauvFile[i].get()->name + "/" +parent, scene->iauvFile[i].get()->range_sensors[j].name);
          tfpub_->sendTransform(t);
        }  
      }

      //publish PressureSensor
      for(int j=0; j< scene->iauvFile[i].get()->pressure_sensors.size();j++)
      {
        tf::Pose pose;
        std::string parent;
        if(scene->iauvFile[i].get()->pressure_sensors[j].getTFTransform(pose,parent))
        {
          tf::StampedTransform t(pose, getROSTime(),   "/"+scene->iauvFile[i].get()->name + "/" +parent, scene->iauvFile[i].get()->pressure_sensors[j].name);
          tfpub_->sendTransform(t);
        }  
      }

      //publish GPSSensor
      for(int j=0; j< scene->iauvFile[i].get()->gps_sensors.size();j++)
      {
        tf::Pose pose;
        std::string parent;
        if(scene->iauvFile[i].get()->gps_sensors[j].getTFTransform(pose,parent))
        {
          tf::StampedTransform t(pose, getROSTime(),   "/"+scene->iauvFile[i].get()->name + "/" +parent, scene->iauvFile[i].get()->gps_sensors[j].name);
          tfpub_->sendTransform(t);
        }  
      }

      //publish DVLSensor
      for(int j=0; j< scene->iauvFile[i].get()->dvl_sensors.size();j++)
      {
        tf::Pose pose;
        std::string parent;
        if(scene->iauvFile[i].get()->dvl_sensors[j].getTFTransform(pose,parent))
        {
          tf::StampedTransform t(pose, getROSTime(),   "/"+scene->iauvFile[i].get()->name + "/" +parent, scene->iauvFile[i].get()->dvl_sensors[j].name);
          tfpub_->sendTransform(t);
        }  
      }

   }

   //Publish object frames
   if(enableObjects_)
   {

     boost::shared_ptr<osg::Matrix> LWMat=getWorldCoords(scene->scene->localizedWorld);
     LWMat->invert(*LWMat);

     for(unsigned int i=0;i<scene->objects.size();i++)
     {
       boost::shared_ptr<osg::Matrix> objectMat= getWorldCoords(scene->objects[i]);

       osg::Matrixd  res=*objectMat * *LWMat;

       tf::Vector3 p2(res.getTrans().x(), res.getTrans().y(), res.getTrans().z());
       tf::Quaternion q2(res.getRotate().x(), res.getRotate().y(), res.getRotate().z(), res.getRotate().w());
       tf::Pose pose2(q2,p2);
       tf::StampedTransform t(pose2, getROSTime(),  "/" + worldRootName_, scene->objects[i]->getName());

       tfpub_->sendTransform(t);
     }

   }



}

WorldToROSTF::~WorldToROSTF()
{
}
