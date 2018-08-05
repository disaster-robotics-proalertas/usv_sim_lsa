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

#include <osgOcean/OceanTechnique>
#include <osgDB/Registry>

using namespace osgOcean;

OceanTechnique::OceanTechnique(void)
    :_isDirty    ( true )
    ,_isAnimating( true )
{}

OceanTechnique::OceanTechnique( const OceanTechnique& copy, const osg::CopyOp& copyop )
    :osg::Geode  ( copy, copyop )
    ,_isDirty    ( true )
    ,_isAnimating( copy._isAnimating )
{}

void OceanTechnique::build(void)
{
    osg::notify(osg::DEBUG_INFO) << "OceanTechnique::build() Not Implemented" << std::endl;
}

float OceanTechnique::getSurfaceHeight(void) const
{
    osg::notify(osg::DEBUG_INFO) << "OceanTechnique::getSurfaceHeight() Not Implemented" << std::endl;
    return 0.f;
}

float OceanTechnique::getMaximumHeight(void) const
{
    osg::notify(osg::DEBUG_INFO) << "OceanTechnique::getMaximumHeight() Not Implemented" << std::endl;
    return 0.f;
}

/** Check if the ocean surface is visible or not. Basic test is very 
    simple, subclasses can do a better test. */
bool OceanTechnique::isVisible( osgUtil::CullVisitor& cv, bool eyeAboveWater )
{
    if (getNodeMask() == 0) return false;

    // Use a cutoff to unconditionally cull ocean surface if we can't see it.
    // This test is valid when the eye is close to the horizon, but further up
    // it will be too conservative (i.e. it will return true even when the 
    // surface is not visible because it does the test relative to a horizontal
    // plane at the eye position).
    osg::Camera* currentCamera = cv.getCurrentRenderBin()->getStage()->getCamera();
    if (currentCamera->getProjectionMatrix()(3,3) == 0.0)     // Perspective
    {
        double fovy, ratio, zNear, zFar;
        currentCamera->getProjectionMatrixAsPerspective(fovy, ratio, zNear, zFar);

        static const float cutoff = fovy / 2.0;
        osg::Vec3 lookVector = cv.getLookVectorLocal();
        float dotProduct = lookVector * osg::Vec3(0,0,1);
        return ( eyeAboveWater && dotProduct <  cutoff) ||
               (!eyeAboveWater && dotProduct > -cutoff);
    }
    else                                                      // Ortho
    {
        return true;
    }

    // A better way would be to check if any of the frustum corners intersect 
    // the plane at (0,0,ocean_height) with normal (0,0,1), and if not then 
    // return true.
}

void OceanTechnique::addResourcePaths(void)
{
    const std::string shaderPath  = "resources/shaders/";
    const std::string texturePath = "resources/textures/";

    osgDB::FilePathList& pathList = osgDB::Registry::instance()->getDataFilePathList();

    bool shaderPathPresent = false;
    bool texturePathPresent = false;

    for(unsigned int i = 0; i < pathList.size(); ++i )
    {
        if( pathList.at(i).compare(shaderPath) == 0 )
            shaderPathPresent = true;

        if( pathList.at(i).compare(texturePath) == 0 )
            texturePathPresent = true;
    }

    if(!texturePathPresent)
        pathList.push_back(texturePath);

    if(!shaderPathPresent)
        pathList.push_back(shaderPath);
}

// --------------------------------------------------------
//  EventHandler implementation
// --------------------------------------------------------

OceanTechnique::EventHandler::EventHandler(OceanTechnique* oceanSurface)
    :_oceanSurface(oceanSurface)
{
}

bool OceanTechnique::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
{
    if (ea.getHandled()) return false;

    // Nothing to do

    return false;
}

void OceanTechnique::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{}
