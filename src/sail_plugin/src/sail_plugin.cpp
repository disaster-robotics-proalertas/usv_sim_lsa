#include <algorithm>
#include <string>

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/transport/transport.hh>
#include <sail_plugin/sail_plugin.h>

#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>

using namespace gazebo;


GZ_REGISTER_MODEL_PLUGIN(SailPlugin)

/////////////////////////////////////////////////
SailPlugin::SailPlugin() : cla(1.0), cda(0.01), cma(0.01), rho(1.2041)
{
std::cerr << "-----------------------------------";
ROS_INFO("------------------------------SailPlugin OBJECT CREATED!!!!");
  this->cp = math::Vector3(0, 0, 0);
  this->forward = math::Vector3(1, 0, 0);
  this->upward = math::Vector3(0, 0, 1);
  this->area = 1.0;
  this->alpha0 = 0.0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;
  this->wind = 0.0;
}

/////////////////////////////////////////////////
void SailPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
ROS_INFO("------------------------------SailPlugin loaded!!!!");


  GZ_ASSERT(_model, "SailPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "SailPlugin _sdf pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "SailPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "SailPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "SailPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<math::Vector3>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<math::Vector3>("forward");

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<math::Vector3>("upward");

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    this->linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(this->linkName);
    std::cerr << "Link name: [" << this->linkName << "\n";
    std::cerr << "Link: [" << this->link->GetName() << "\n";
  }

  if (_sdf->HasElement("joint_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("joint_name");
    this->jointName = elem->Get<std::string>();
    this->joint = this->model->GetJoint(this->jointName);
    std::cerr << "Joint name: [" << this->jointName << "\n";
    std::cerr << "Joint: [" << this->joint->GetName() << "\n";
  }

  ros::NodeHandle nh;
  float wind_value_x;  
  float wind_value_y;
  float wind_value_z;
  if (nh.getParam("/uwsim/wind/x", wind_value_x) & nh.getParam("/uwsim/wind/y", wind_value_y))
  {
	this->wind = math::Vector3(wind_value_x, wind_value_y, 0);
  }
  else
  {
	ROS_INFO("Sail plugin error: Cant find value of /uwsim/wind in param server");
  }
}

void SailPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SailPlugin::OnUpdate, this));
  std::string topic = "/" + this->model->GetName() + "/angleLimits";
  this->angleLimits_subscriber = rosnode.subscribe(topic, 1, &SailPlugin::ropeSimulator, this); 
}

/////////////////////////////////////////////////
void SailPlugin::OnUpdate()
{
//  this->joint->SetLowerLimit(0, gazebo::math::Angle(-this->angle));
//  this->joint->SetUpperLimit(0, gazebo::math::Angle(this->angle));
  this->joint->SetLowStop(0, gazebo::math::Angle(-this->angle));
  this->joint->SetHighStop(0, gazebo::math::Angle(this->angle));
  //math::Vector3 vel = math::Vector3(1,0,0)- this->link->GetWorldLinearVel(this->cp);
  //math::Vector3 vel = this->link->GetWorldLinearVel(this->cp) - wind;
  math::Vector3 aw = wind - this->link->GetWorldLinearVel(this->cp); //wind = v_tw
  //math::Vector3 vel = this->link->GetWorldLinearVel(this->cp);

  if (aw.GetLength() <= 0.01)
    return;

  // pose of body
  math::Pose pose = this->link->GetWorldPose();

  // rotate forward and upward vectors into inertial frame
  math::Vector3 forwardI = pose.rot.RotateVector(this->forward); //xb
  math::Vector3 upwardI = pose.rot.RotateVector(this->upward);   //yb

  // ldNormal vector to lift-drag-plane described in inertial frame
  math::Vector3 ldNormal = forwardI.Cross(upwardI).Normalize();
  // TODOS ESSES PRODUTOS VETORIAIS SÃO PRA PEGAR OS VETORES PERPENDICULARES

  //math::Vector3 velInLDPlane = ldNormal.Cross(aw.Cross(ldNormal)); // isso é igual ao vel, só que escalado????
  math::Vector3 velInLDPlane = aw;
  // get direction of drag
  math::Vector3 dragDirection = velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
 // math::Vector3 liftDirection = ldNormal.Cross(velInLDPlane);
  math::Vector3 liftDirection = -ldNormal.Cross(velInLDPlane);
  liftDirection.Normalize();

  // get direction of moment
  math::Vector3 momentDirection = ldNormal;

  double cosAlpha = math::clamp(
    forwardI.Dot(velInLDPlane) /
    (forwardI.GetLength() * velInLDPlane.GetLength()), -1.0, 1.0);

  // get sign of alpha
  // take upwards component of velocity in lift-drag plane.
  // if sign == upward, then alpha is negative
  double alphaSign = -upwardI.Dot(velInLDPlane)/
    (upwardI.GetLength() + velInLDPlane.GetLength());

  // double sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
  if (alphaSign > 0.0)
    this->alpha = acos(cosAlpha);
  else
    this->alpha = -acos(cosAlpha);
  
  // normalize to within +/-180 deg
 // while (fabs(this->alpha) > M_PI)
 //   this->alpha = this->alpha > 0 ? this->alpha - 2*M_PI
  //                                : this->alpha + 2*M_PI;
  
 // this->alpha += M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.GetLength();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;


//std::cerr<<"\n speedInLDPlane: "<<speedInLDPlane<<" q: "<<q;
//std::cerr<<"\n alpha: "<<alpha<<" alphaStall: "<<alphaStall;
//std::cerr<<"\n alpha: "<<alpha;
  // compute cl at cp, check for stall, correct for sweep

  double cl;

  cl = 1.5*sin(2*this->alpha);
  // compute lift force at cp
  math::Vector3 lift = cl * q * this->area * liftDirection;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  // make sure drag is positive
  //cd = fabs(cd);

  cd = 0.5*(1-cos(2*this->alpha));
  // drag at cp
  math::Vector3 drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  double cm;
  // reset cm to zero, as cm needs testing
  cm = 0.0;

  // compute moment (torque) at cp
  math::Vector3 moment = cm * q * this->area * momentDirection;

  // moment arm from cg to cp in inertial plane
  math::Vector3 momentArm = pose.rot.RotateVector(
    this->cp - this->link->GetInertial()->GetCoG());
  // std::cerr << this->cp << " : " << this->link->GetInertial()->GetCoG() << "\n";

  // force and torque about cg in inertial frame
  math::Vector3 force = lift + drag;
  // + moment.Cross(momentArm);

  math::Vector3 torque = moment;


  if (0) {
    std::cerr << "Link: [" << this->link->GetName() << "\n";
    std::cerr << "alpha: " << this->alpha*180/3.1415 << "\n";
    std::cerr << "wind: " << wind << "\n";
    std::cerr << "cl: " << cl << "\n";
    std::cerr << "lift: " << lift << "\n";
    std::cerr << "cd: " << cd << "\n";
    std::cerr << "drag: " << drag << " cd: "
    << cd << "\n";
    std::cerr << "force: " << force << "\n\n";
  }
//std::cerr<<"\n CL: "<<cl<<" CD: "<<cd;
//std::cerr<<"\n"<<this->link->GetName()<<" lift: "<<lift<<" drag: "<<drag<< "total: "<<force;
//std::cerr<<"\n "<<this->link->GetName()<<" total: "<<force;
  // - lift.Cross(momentArm) - drag.Cross(momentArm);

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.GetLength() > 50.0 &&
  //      vel.GetLength() < 50.0))

  if (0)
  {
    std::cerr << "=============================\n";
    std::cerr << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";
    std::cerr << "spd: [" << aw.GetLength() << "] vel: [" << aw << "]\n";
    std::cerr << "spd sweep: [" << velInLDPlane.GetLength()
          << "] vel in LD: [" << velInLDPlane << "]\n";
    std::cerr << "forward (inertial): " << forwardI << "\n";
    std::cerr << "upward (inertial): " << upwardI << "\n";
    std::cerr << "lift dir (inertial): " << liftDirection << "\n";
    std::cerr << "LD Normal: " << ldNormal << "\n";
    std::cerr << "sweep: " << this->sweep << "\n";
    std::cerr << "alpha: " << this->alpha*180/3.1415 << "\n";
    std::cerr << "cl: " << cl << "\n";
    std::cerr << "lift: " << lift << "\n";
    std::cerr << "cd: " << cd << "\n";
    std::cerr << "drag: " << drag << " cd: "
    << cd << "\n";
    std::cerr << "force: " << force << "\n";
  }

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  //this->link->AddTorque(torque);
}
