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
#include <uwsim/DredgeTool.h>

//PARTICLE SYSTEM FUNCTIONS

inline void AttractOperator::operate( osgParticle::Particle* P, double dt )
{
  osg::Vec3 dir = - P->getPosition();
  if (dir.length()>0.02 )
  {
    //Probable something more realistic can be done, but I think it's not worthy
    P->setVelocity(dir*(1/dir.length())*_magnitude);
  }
  else
  {
    if (_killSink)
    {
      P->kill();
    }
    else
    {
      P->setPosition(osg::Vec3(0,0,0));
      P->setVelocity(osg::Vec3(0,0,0));
    }
  }
}



osgParticle::ParticleSystem* createSmokeParticles( osg::Group* parent, osgParticle::RandomRateCounter * rrc)
{

    //Set particle properties (TODO: take them from config file)
    osg::ref_ptr<osgParticle::ParticleSystem> ps = new osgParticle::ParticleSystem;
    ps->getDefaultParticleTemplate().setLifeTime( 15.0f );
    ps->getDefaultParticleTemplate().setShape( osgParticle::Particle::QUAD );
    ps->getDefaultParticleTemplate().setSizeRange( osgParticle::rangef(0.05f, 0.1f) );
    ps->getDefaultParticleTemplate().setAlphaRange( osgParticle::rangef(1.0f, 0.0f) );
    ps->getDefaultParticleTemplate().setColorRange(
        osgParticle::rangev4(osg::Vec4(0.3f,0.2f,0.01f,1.0f), osg::Vec4(0.15f,0.1f,0.01f,0.5f)) );
    ps->setDefaultAttributes( "smoke.rgb", true, false );
    
    rrc->setRateRange( 0, 0 );

    //Box placer creates particles inside a box
    //TODO: Box size should depend on dredge tool force
    osg::ref_ptr<osgParticle::BoxPlacer> placer = new osgParticle::BoxPlacer;
    placer->setXRange(-0.1,0.1);
    placer->setYRange(-0.1,0.1);
    placer->setZRange(0,0.2);
    
    //Create emitter
    osg::ref_ptr<osgParticle::ModularEmitter> emitter = new osgParticle::ModularEmitter;
    emitter->setPlacer(placer);
    emitter->setParticleSystem( ps.get() );
    emitter->setCounter( rrc );
    parent->addChild( emitter.get() );


    //Create AttractOperator program
    osgParticle::ModularProgram *moveDustInAir = new osgParticle::ModularProgram;
    moveDustInAir->setParticleSystem(ps);
    
    AttractOperator * attOp= new AttractOperator;
    attOp->setMagnitude(0.2);
    moveDustInAir->addOperator(attOp);

    //Add FluidFriction (not sure if it does something)
    osgParticle::FluidFrictionOperator *airFriction = new osgParticle::FluidFrictionOperator;
    airFriction->setFluidToWater();
    moveDustInAir->addOperator(airFriction);

    parent->addChild(moveDustInAir);

    return ps.get();
}




DredgeTool::DredgeTool(DredgeTool_Config * cfg, osg::ref_ptr<osg::Node> target) :
    SimulatedDevice(cfg)
{
  this->target = target;
  particles=0;

  //Create particle system:
  rrc= new osgParticle::RandomRateCounter;

  smoke = createSmokeParticles(target->asGroup(),rrc);

  osg::ref_ptr<osgParticle::ParticleSystemUpdater> updater = new osgParticle::ParticleSystemUpdater;
  updater->addParticleSystem( smoke );
  
  osg::ref_ptr<osg::Geode> smokeGeode = new osg::Geode;
  smokeGeode->getOrCreateStateSet()->setAttributeAndModes(new osg::Program(), osg::StateAttribute::ON);
  smokeGeode->addDrawable( smoke );

  target->asGroup()->addChild( smokeGeode.get() );
  target->asGroup()->addChild( updater.get() );

}

boost::shared_ptr<osg::Matrix> DredgeTool::getDredgePosition()
{
  return getWorldCoords(target);;
}

void DredgeTool::dredgedParticles(int nparticles)
{
  //TODO: use time instead of function calls.
  particles*=0.9;
  particles+=nparticles;

  rrc->setRateRange( min(particles,50), min(particles*2,100) );

}

SimulatedDeviceConfig::Ptr DredgeTool_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{
  DredgeTool_Config * cfg = new DredgeTool_Config(getType());
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

bool DredgeTool_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder,
                                      size_t iteration)
{
  if (iteration > 0)
    return true;
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType())
    {
      DredgeTool_Config * cfg = dynamic_cast<DredgeTool_Config *>(vehicleChars.simulated_devices[i].get());

      osg::ref_ptr<osg::Node> target;
      for(int j=0;j<auv->urdf->link.size();j++)
      {
        if(auv->urdf->link[j]->getName()==cfg->target)
        {
          target=auv->urdf->link[j];
        }
      }
      if(target)
        auv->devices->all.push_back(DredgeTool::Ptr(new DredgeTool(cfg,target)));
      else
        OSG_FATAL << "DredgeTool device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
            << vehicleChars.name << "' has empty info, discarding..." << std::endl;
    }
  return true;
}



#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(DredgeTool_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(DredgeTool_Factory, DredgeTool_Factory, uwsim::SimulatedDeviceFactory)
#endif

