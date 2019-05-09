/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <string>
#include <vector>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include "water_current/GetSpeed.h"
#include "wind_current/GetSpeed.h"

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class Foil_Dynamics_Plugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: Foil_Dynamics_Plugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();
    protected: virtual void OnUpdateRudder();
    protected: virtual void OnUpdateKeel();
    protected: virtual void OnUpdateSail();

    // Read topic gazebo/current to get water current
    protected: void ReadWaterCurrent(const geometry_msgs::Vector3::ConstPtr& _msg);

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Name of model containing plugin.
    protected: std::string modelName;

    protected: double rho;

    /// \brief effective planeform surface area
    protected: double area;

    /// \brief angle of sweep
    protected: double sweep;

    /// \brief angle of attack
    protected: double alpha;

    protected: double mult_lift;
    protected: double mult_drag;

    /// \brief center of pressure in link local coordinates
    protected: math::Vector3 cp;

    /// \brief forward flight direction in link local coordinates
    protected: math::Vector3 forward;

    /// \brief A vector in the lift/drag plane, anything orthogonal to it
    /// is considered wing sweep.
    protected: math::Vector3 upward;

    /// \brief Names of allowed target links, specified in sdf parameters.
    protected: std::string linkName;
    protected: std::string jointName;
	
    /// \briedf Type of link: rudder, sail, keel
    protected: std::string linkType;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;
    protected: physics::JointPtr joint;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    protected: ros::Subscriber current_subscriber_;

    protected: math::Vector3 waterCurrent;

    protected: ros::NodeHandle rosnode_;
    protected: math::Vector3 wind;
    //protected: float angle;
    protected: ros::ServiceClient velocity_serviceClient_;
    protected: bool running;
    protected: std::string fluidVelocity;

    //protected: ros::Subscriber angleLimits_subscriber;
    std::thread the_thread;
    protected: ros::Time oldTime;

   // public: void ropeSimulator(const std_msgs::Float64::ConstPtr& _angle){
   //     this->angle = _angle->data;
   // }
    public: void WaterThreadLoop();
    public: void WindThreadLoop();
  };

}

