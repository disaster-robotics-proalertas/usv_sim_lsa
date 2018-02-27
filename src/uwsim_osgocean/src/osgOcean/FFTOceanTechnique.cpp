/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include <osgOcean/FFTOceanTechnique>
#include <osgOcean/ShaderManager>
#include <osg/io_utils>
#include <osg/Material>
#include <osg/Timer>
#include <ros/ros.h>

using namespace osgOcean;


FFTOceanTechnique::FFTOceanTechnique( unsigned int FFTGridSize,
                                      unsigned int resolution,
                                      unsigned int numTiles, 
                                      const osg::Vec2f& windDirection,
                                      float windSpeed,
                                      float depth,
                                      float reflectionDamping,
                                      float waveScale,
                                      bool isChoppy,
                                      float choppyFactor,
                                      float animLoopTime,
                                      unsigned int numFrames )
    :_tileSize       ( FFTGridSize )
    ,_noiseTileSize  ( FFTGridSize )
    ,_tileResolution ( resolution )
    ,_tileResInv     ( 1.f / float(resolution) )
    ,_noiseTileRes   ( resolution )
    ,_numTiles       ( numTiles )
    ,_pointSpacing   ( _tileResolution / _tileSize )
    ,_windDirection  ( windDirection )
    ,_noiseWindDir   ( windDirection )
    ,_windSpeed      ( windSpeed )
    ,_noiseWindSpeed ( windSpeed )
    ,_waveScale      ( waveScale )
    ,_noiseWaveScale ( waveScale )
    ,_depth          ( depth )
    ,_reflDampFactor ( reflectionDamping )
    ,_cycleTime      ( animLoopTime )
    ,_choppyFactor   ( choppyFactor )
    ,_isChoppy       ( isChoppy )
    ,_isEndless      ( false )
    ,_oldFrame       ( 0 )
    ,_fresnelMul     ( 0.7 )
    ,_numLevels      ( (unsigned int) ( log( (float)_tileSize) / log(2.f) )+1)
    ,_startPos       ( -float( (_tileResolution+1)*_numTiles) * 0.5f, float( (_tileResolution+1)*_numTiles) * 0.5f )
    ,_THRESHOLD      ( 3.f )
    ,_VRES           ( 1920 )
    ,_NUMFRAMES      ( numFrames )
    ,_waveTopColor   ( 0.192156862f, 0.32549019f, 0.36862745098f )
    ,_waveBottomColor( 0.11372549019f, 0.219607843f, 0.3568627450f )
    ,_useCrestFoam   ( false )
    ,_foamCapBottom  ( 2.2f )
    ,_foamCapTop     ( 3.0f )
    ,_isStateDirty   ( true )
    ,_averageHeight  ( 0.f )
    ,_lightColor     ( 0.411764705f, 0.54117647f, 0.6823529f, 1.f )
{
    _stateset = new osg::StateSet;
    addResourcePaths();
    setUserData( new OceanDataType(*this, _NUMFRAMES, 25) );
    setOceanAnimationCallback( new OceanAnimationCallback );
}

FFTOceanTechnique::FFTOceanTechnique( const FFTOceanTechnique& copy, const osg::CopyOp& copyop ):
    OceanTechnique   ( copy, copyop )
    ,_tileSize       ( copy._tileSize )
    ,_noiseTileSize  ( copy._noiseTileSize )
    ,_tileResolution ( copy._tileResolution )
    ,_tileResInv     ( copy._tileResInv )
    ,_noiseTileRes   ( copy._noiseTileRes )
    ,_numTiles       ( copy._numTiles )
    ,_pointSpacing   ( copy._pointSpacing )
    ,_windDirection  ( copy._windDirection )
    ,_noiseWindDir   ( copy._noiseWindDir )
    ,_windSpeed      ( copy._windSpeed )
    ,_noiseWindSpeed ( copy._noiseWindSpeed )
    ,_waveScale      ( copy._waveScale )
    ,_noiseWaveScale ( copy._noiseWaveScale )
    ,_depth          ( copy._depth )
    ,_cycleTime      ( copy._cycleTime )
    ,_choppyFactor   ( copy._choppyFactor )
    ,_isChoppy       ( copy._isChoppy )
    ,_isEndless      ( copy._isEndless )
    ,_oldFrame       ( copy._oldFrame )
    ,_fresnelMul     ( copy._fresnelMul )
    ,_numLevels      ( copy._numLevels )
    ,_startPos       ( copy._startPos )
    ,_THRESHOLD      ( copy._THRESHOLD )
    ,_VRES           ( copy._VRES )
    ,_NUMFRAMES      ( copy._NUMFRAMES )
    ,_minDist        ( copy._minDist )
    ,_environmentMap ( copy._environmentMap )
    ,_waveTopColor   ( copy._waveTopColor )
    ,_waveBottomColor( copy._waveBottomColor )
    ,_useCrestFoam   ( copy._useCrestFoam )
    ,_foamCapBottom  ( copy._foamCapBottom )
    ,_foamCapTop     ( copy._foamCapTop )
    ,_isStateDirty   ( copy._isStateDirty )
    ,_averageHeight  ( copy._averageHeight )
    ,_lightColor     ( copy._lightColor )
{}

FFTOceanTechnique::~FFTOceanTechnique(void)
{
}

osg::Texture2D* FFTOceanTechnique::createTexture(const std::string& name, osg::Texture::WrapMode wrap)
{
    osg::Texture2D* tex = new osg::Texture2D();

    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setWrap  (osg::Texture::WRAP_S,     wrap);
    tex->setWrap  (osg::Texture::WRAP_T,     wrap);
    tex->setImage (osgDB::readImageFile(name.c_str()));

    return tex;
}

float FFTOceanTechnique::getSurfaceHeightAt(float x, float y, osg::Vec3f* normal)
{
    osg::notify(osg::INFO) << "getSurfaceHeightAt() not implemented." << std::endl;
    return 0.f;
}

void FFTOceanTechnique::setOceanAnimationCallback(FFTOceanTechnique::OceanAnimationCallback* callback)
{
    setUpdateCallback(callback);
    setCullCallback(callback);
}

FFTOceanTechnique::OceanAnimationCallback* FFTOceanTechnique::getOceanAnimationCallback()
{
    return dynamic_cast<OceanAnimationCallback*>(getUpdateCallback());
}

// --------------------------------------------------------
//  OceanDataType implementation
// --------------------------------------------------------

FFTOceanTechnique::OceanDataType::OceanDataType( FFTOceanTechnique& ocean, 
                                                 unsigned int numFrames, 
                                                 unsigned int fps )
    :_oceanSurface  ( ocean )
    ,_NUMFRAMES     ( numFrames )
    ,_time          ( 0.0 )
    ,_FPS           ( fps )
    ,_msPerFrame    ( 1000.0/(double)fps )
    ,_frame         ( 0 )
    ,_oldTime       ( 0 )
    ,_newTime       ( 0 )
{}

FFTOceanTechnique::OceanDataType::OceanDataType( const OceanDataType& copy, const osg::CopyOp& copyop )
    :_oceanSurface  ( copy._oceanSurface )
    ,_NUMFRAMES     ( copy._NUMFRAMES )
    ,_eye           ( copy._eye )
    ,_time          ( copy._time )
    ,_FPS           ( copy._FPS )
    ,_msPerFrame    ( copy._msPerFrame )
    ,_frame         ( copy._frame )
    ,_oldTime       ( copy._oldTime )
    ,_newTime       ( copy._newTime )
{}

void FFTOceanTechnique::OceanDataType::updateOcean( double simulationTime )
{
    _oldTime = _newTime;

    if (simulationTime < 0.0)
    {
        _newTime = osg::Timer::instance()->tick();
    }
    else
    {
        // simulationTime is passed in seconds.
        _newTime = simulationTime / osg::Timer::instance()->getSecondsPerTick();
    }

    double dt = osg::Timer::instance()->delta_m(_oldTime, _newTime);
    _time += dt;

    if( _time >= _msPerFrame )
    {
        _frame += ( _time / _msPerFrame );

        if( _frame >= _NUMFRAMES ) 
            _frame = _frame%_NUMFRAMES; 

        _time = fmod( _time, (double)_msPerFrame );
    }

    _oceanSurface.update( _frame, dt, _eye );
}

// --------------------------------------------------------
//  EventHandler implementation
// --------------------------------------------------------

FFTOceanTechnique::EventHandler::EventHandler(FFTOceanTechnique* oceanSurface)
    :OceanTechnique::EventHandler(oceanSurface)
    ,_autoDirty(true)
{
}

bool FFTOceanTechnique::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object* object, osg::NodeVisitor* nv)
{
    // Call parent class's handle().
    OceanTechnique::EventHandler::handle(ea, aa, object, nv);

    if (ea.getHandled()) return false;

    // Now we can handle this class's events.
    switch(ea.getEventType())
    {
    case(osgGA::GUIEventAdapter::KEYUP):
        {
            // Downcast to the concrete class we're interested in.
            FFTOceanTechnique* fftSurface = dynamic_cast<FFTOceanTechnique*>(_oceanSurface);
            if (!fftSurface) return false;

            // Crest foam
            if (ea.getKey() == 'f' )
            {
                fftSurface->enableCrestFoam(!fftSurface->isCrestFoamEnabled());
                osg::notify(osg::NOTICE) << "Crest foam " << (fftSurface->isCrestFoamEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // isChoppy
            if( ea.getKey() == 'p' )
            {
                fftSurface->setIsChoppy(!fftSurface->isChoppy(), _autoDirty);
                osg::notify(osg::NOTICE) << "Choppy waves " << (fftSurface->isChoppy()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Wind speed + 0.5
            if (ea.getKey() == 'W')
            {
                fftSurface->setWindSpeed(fftSurface->getWindSpeed() + 0.5, _autoDirty);
                osg::notify(osg::NOTICE) << "Wind speed now " << fftSurface->getWindSpeed() << std::endl;
                return true;
            }
            // Wind speed - 0.5
            if (ea.getKey() == 'w')
            {
                fftSurface->setWindSpeed(fftSurface->getWindSpeed() - 0.5, _autoDirty);
                osg::notify(osg::NOTICE) << "Wind speed now " << fftSurface->getWindSpeed() << std::endl;
                return true;
            }
            // Scale factor + 1e-9
            if(ea.getKey() == 'K' )
            {
                float waveScale = fftSurface->getWaveScaleFactor();
                fftSurface->setWaveScaleFactor(waveScale+(1e-9), _autoDirty);
                osg::notify(osg::NOTICE) << "Wave scale factor now " << fftSurface->getWaveScaleFactor() << std::endl;
                return true;
            }
            // Scale factor - 1e-9
            if(ea.getKey() == 'k' )
            {
                float waveScale = fftSurface->getWaveScaleFactor();
                fftSurface->setWaveScaleFactor(waveScale-(1e-9), _autoDirty);
                osg::notify(osg::NOTICE) << "Wave scale factor now " << fftSurface->getWaveScaleFactor() << std::endl;
                return true;
            }
            // Dirty geometry
            if (ea.getKey() == 'd')
            {
                osg::notify(osg::NOTICE) << "Dirtying ocean geometry" << std::endl;
                fftSurface->dirty();
                return true;
            }
            // Toggle autoDirty, if off then individual changes will be 
            // instantaneous but the user will get no feedback until they 
            // dirty manually, if on each change will dirty automatically.
            if (ea.getKey() == 'D')
            {
                _autoDirty = !_autoDirty;
                osg::notify(osg::NOTICE) << "AutoDirty " << (_autoDirty? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Print out all current settings to the console.
            if (ea.getKey() == 'P')
            {
                osg::notify(osg::NOTICE) << "Current FFTOceanTechnique settings are:" << std::endl;
                osg::notify(osg::NOTICE) << "  Endless ocean " << (fftSurface->isEndlessOceanEnabled()? "enabled" : "disabled") << std::endl;
                osg::notify(osg::NOTICE) << "  Crest foam " << (fftSurface->isCrestFoamEnabled()? "enabled" : "disabled") << std::endl;
                osg::notify(osg::NOTICE) << "  Choppy waves " << (fftSurface->isChoppy()? "enabled" : "disabled") << std::endl;
                osg::notify(osg::NOTICE) << "  Choppy factor " << fftSurface->getChoppyFactor() << std::endl;
                osg::notify(osg::NOTICE) << "  Wind direction " << fftSurface->getWindDirection() << std::endl;
                osg::notify(osg::NOTICE) << "  Wind speed " << fftSurface->getWindSpeed() << std::endl;
                osg::notify(osg::NOTICE) << "  Wave scale factor " << fftSurface->getWaveScaleFactor() << std::endl;
                return true;
            }
            break;
        }
    default:
        break;
    }

    return false;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void FFTOceanTechnique::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
    // Add parent class's keys too.
    OceanTechnique::EventHandler::getUsage(usage);

    usage.addKeyboardMouseBinding("f","Toggle crest foam");
    usage.addKeyboardMouseBinding("p","Toggle choppy waves (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("k","Decrease wave scale factor by 1e-9 (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("K","Increase wave scale factor by 1e-9 (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("w","Decrease wind speed by 0.5 (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("W","Increase wind speed by 0.5 (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("d","Dirty geometry manually");
    usage.addKeyboardMouseBinding("D","Toggle autoDirty (if off, changes will require manual dirty)");
    usage.addKeyboardMouseBinding("P","Print out current ocean surface settings");
}

// --------------------------------------------------------
//  OceanAnimationCallback implementation
// --------------------------------------------------------

void FFTOceanTechnique::OceanAnimationCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    update(node, nv, ros::Time::now().toSec());
//    update(node, nv, -1.0);

    traverse(node, nv); 
}

void FFTOceanTechnique::OceanAnimationCallback::update(osg::Node* node, osg::NodeVisitor* nv, double simulationTime)
{
    osg::ref_ptr<OceanDataType> oceanData = dynamic_cast<OceanDataType*> ( node->getUserData() );

    if( oceanData.valid() )
    {
        // If cull visitor update the current eye position
        if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            osg::Camera* currentCamera = cv->getCurrentRenderBin()->getStage()->getCamera();
            if (currentCamera->getName() == "ShadowCamera" ||
                currentCamera->getName() == "AnalysisCamera" )
            {
            }
            else
            {
                oceanData->setEye( cv->getEyePoint() );
            }
        }
        else if( nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR ){
            oceanData->updateOcean(simulationTime);
        }
    }

    traverse(node, nv); 
}
