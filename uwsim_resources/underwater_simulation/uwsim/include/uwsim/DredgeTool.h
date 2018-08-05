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


#ifndef DREDGE_TOOL_H_
#define DREDGE_TOOL_H_
#include "SimulatedDevice.h"
using namespace uwsim;

//PARTICLE SYSTEM FUNCTIONS

#include <osgParticle/ModularEmitter>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/BoxPlacer>

#include <osgParticle/FluidFrictionOperator>
#include <osgParticle/ModularProgram>

//This operator attracts the particles to a sink (dredge tool).
//This class could be in UWSimUtils as it is not specific for Dredge Tool, but I left it here because I think
// it will not be used anywhere else
class AttractOperator : public osgParticle::Operator
{
  public:
    AttractOperator() : osgParticle::Operator(), _magnitude(1.0f), _killSink(true){}
    AttractOperator( const AttractOperator& copy, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY )
      : osgParticle::Operator(copy, copyop), _magnitude(copy._magnitude), _killSink(copy._killSink)
      {}
    META_Object( spin, AttractOperator );

    /// Set the acceleration scale
    void setMagnitude( float mag ) { _magnitude = mag; }
    /// Get the acceleration scale
    float getMagnitude() const { return _magnitude; }
    /// Set whether the attractor kills the particles once they arrive
    void setKillSink( bool kill ) { _killSink = kill; }
    /// Get whether the attractor kills the particles once they arrive
    bool getKillSink() const { return _killSink; }
    /// Apply attraction to a particle. Do not call this method manually.
    inline void operate( osgParticle::Particle* P, double dt );

  protected:
    virtual ~AttractOperator() {}
    AttractOperator& operator=( const AttractOperator& ) { return *this; }
    float _magnitude;
    bool _killSink;
};

//Creates a particle system
osgParticle::ParticleSystem* createSmokeParticles( osg::Group* parent, osgParticle::RandomRateCounter * rrc);


//Driver/ROSInterface configuration
class DredgeTool_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  std::string target;
  double offsetp[3];
  double offsetr[3];
  //constructor
  DredgeTool_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
  }
};

//Driver/ROSInterface factory class
class DredgeTool_Factory : public SimulatedDeviceFactory
{
public:
  //this is the only place the device/interface type is set
  DredgeTool_Factory(std::string type_ = "DredgeTool") :
      SimulatedDeviceFactory(type_)
  {
  }
  ;

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
};

//can be a sparate header file for actual implementation classes...

#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include <ros/ros.h>
#include "UWSimUtils.h"

//Driver class
class DredgeTool : public SimulatedDevice, public AbstractDredgeTool
{
  osg::ref_ptr<osg::Node> target;
  osgParticle::ParticleSystem* smoke;
  osgParticle::RandomRateCounter * rrc;

  //Number of particles in the system (this should decrease slowly)
  int particles;

  void applyPhysics(BulletPhysics * bulletPhysics)
  {
  }
public:
  std::string info; //Device's property

  DredgeTool(DredgeTool_Config * cfg, osg::ref_ptr<osg::Node> target);

  virtual boost::shared_ptr<osg::Matrix> getDredgePosition();

  void dredgedParticles(int nparticles);


};


#endif
