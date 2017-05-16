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

#include "SkyDome.h"
#include <osgOcean/ShaderManager>

SkyDome::SkyDome( void )
{
    
}

SkyDome::SkyDome( const SkyDome& copy, const osg::CopyOp& copyop ):
    SphereSegment( copy, copyop )
{

}

SkyDome::SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap )
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
    setupStateSet(cubemap);
}

SkyDome::~SkyDome(void)
{
}

void SkyDome::create( float radius, unsigned int latSteps, unsigned int longSteps, osg::TextureCubeMap* cubemap )
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
    setupStateSet(cubemap);
}

void SkyDome::setupStateSet( osg::TextureCubeMap* cubemap )
{
    osg::StateSet* ss = new osg::StateSet;

    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
    ss->setAttributeAndModes( createShader().get(), osg::StateAttribute::ON );
    ss->addUniform( new osg::Uniform("uEnvironmentMap", 0) );

    setStateSet(ss);
}

osg::ref_ptr<osg::Program> SkyDome::createShader(void)
{
    osg::ref_ptr<osg::Program> program = new osg::Program;

    // Do not use shaders if they were globally disabled.
    if (osgOcean::ShaderManager::instance().areShadersEnabled())
    {
        char vertexSource[]=
            "varying vec3 vTexCoord;\n"
            "\n"
            "void main(void)\n"
            "{\n"
            "    gl_Position = ftransform();\n"
            "    vTexCoord = gl_Vertex.xyz;\n"
            "}\n";

        char fragmentSource[]=
            "uniform samplerCube uEnvironmentMap;\n"
            "varying vec3 vTexCoord;\n"
            "\n"
            "void main(void)\n"
            "{\n"
            "   vec3 texcoord = vec3(vTexCoord.x, vTexCoord.y, -vTexCoord.z);\n"
            "   gl_FragData[0] = textureCube( uEnvironmentMap, texcoord.xzy );\n"
            "   gl_FragData[0].a = 0.0;\n"
            "   gl_FragData[1] = vec4(0.0);\n"
            "}\n";

        program->setName( "sky_dome_shader" );
        program->addShader(new osg::Shader(osg::Shader::VERTEX,   vertexSource));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));
    }

    return program;
}
