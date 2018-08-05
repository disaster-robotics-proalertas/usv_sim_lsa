/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2012 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#include "osgwTools/Version.h"

#include "osgwTools/GeometryModifier.h"
#include "osgwTools/TangentSpaceOp.h"

#include <osg/ArgumentParser>
#include <osg/Texture2D>
#include <osg/io_utils>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include "osgwTools/Shapes.h"

#include <iostream>


////////////////////////////////////////////////////////////////////////////////
void addShaders( osg::Node* node, osg::Image* baseImage, osg::Image* normalImage, osg::Image* heightImage )
{
    osg::ref_ptr< osg::StateSet > stateSet = node->getOrCreateStateSet();

    if( baseImage )
    {
        osg::ref_ptr< osg::Texture2D > baseMap = new osg::Texture2D();
        baseMap->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
        baseMap->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
        baseMap->setWrap( osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT );
        baseMap->setWrap( osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT );
        stateSet->
        setTextureAttributeAndModes( 0, baseMap.get(), osg::StateAttribute::ON );
        baseMap->setImage( baseImage );
    }

    osg::ref_ptr< osg::Uniform > baseMapUniform =
        new osg::Uniform( "baseMap", 0 );
    stateSet->addUniform( baseMapUniform.get() );

    ///Setup the normal map
    osg::ref_ptr< osg::Texture2D > normalMap = new osg::Texture2D();
    normalMap->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
    normalMap->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
    normalMap->setWrap( osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT );
    normalMap->setWrap( osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT );
    stateSet->
        setTextureAttributeAndModes( 1, normalMap.get(), osg::StateAttribute::ON );
    normalMap->setImage( normalImage );

    osg::ref_ptr< osg::Uniform > normalMapUniform =
        new osg::Uniform( "normalMap", 1 );
    stateSet->addUniform( normalMapUniform.get() );

    ///Setup the height map
    osg::ref_ptr< osg::Texture2D > heightMap = new osg::Texture2D();
    heightMap->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
    heightMap->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
    heightMap->setWrap( osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT );
    heightMap->setWrap( osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT );
    stateSet->
        setTextureAttributeAndModes( 2, heightMap.get(), osg::StateAttribute::ON );
    heightMap->setImage( heightImage );

    osg::ref_ptr< osg::Uniform > heightMapUniform =
        new osg::Uniform( "heightMap", 2 );
    stateSet->addUniform( heightMapUniform.get() );

    ///Set the bool to control the height map usage
    osg::ref_ptr< osg::Uniform > heightControl = 
        new osg::Uniform( "useHeightMap", false );
    if( heightImage )
    {
        heightControl->set( true );
    }
    stateSet->addUniform( heightControl.get() );

    ///Setup the shaders and programs
    std::string shaderName = osgDB::findDataFile( "parallax_mapping.fs" );
    osg::ref_ptr< osg::Shader > fragmentShader;
    if( shaderName.empty() )
    {
        std::string fragmentSource =
        "uniform sampler2D baseMap;\n"
        "uniform sampler2D normalMap;\n"
        "uniform sampler2D heightMap;\n"
        "uniform bool useHeightMap;\n"
        "\n"
        "varying vec3 v_lightVector;\n"
        "varying vec3 v_viewVector;\n"
        "\n"
        "void main()\n"
        "{\n"
        //determine if we are going to use the height map
        "float height = 0.;\n"
        "float v = 0.;\n"
        "if( useHeightMap )\n"
        "{\n"
            "height = texture2D( heightMap, gl_TexCoord[ 0 ].st ).r;\n"
            "vec2 scaleBias = vec2( 0.06, 0.03 );\n"
            "v = height * scaleBias.s - scaleBias.t;\n"
        "}\n"
        "\n"
        "vec3 V = normalize( v_viewVector );\n"
        "vec2 texCoords = gl_TexCoord[ 0 ].st + ( V.xy * v );\n"
        "\n"
        //
        "float bumpiness = 1.0;\n"
        "vec3 smoothOut = vec3( 0.5, 0.5, 1.0 );\n"
        "vec3 N = texture2D( normalMap, texCoords ).rgb;\n"
        "N = mix( smoothOut, N, bumpiness );\n"
        "N = normalize( ( N * 2.0 ) - 1.0 );\n"
        "\n"
        //
        "vec3 L = normalize( v_lightVector );\n"
        "float NdotL = max( dot( N, L ), 0.0 );\n"
        "\n"
        //
        "vec3 R = reflect( V, N );\n"
        "float RdotL = max( dot( R, L ), 0.0 );\n"
        "\n"
        //
        "float specularPower = 50.0;\n"
        "vec3 base = texture2D( baseMap, texCoords ).rgb;\n"
        "vec3 ambient = vec3( 0.368627, 0.368421, 0.368421 ) * base;\n"
        "vec3 diffuse = vec3( 0.886275, 0.885003, 0.885003 ) * base * NdotL;\n"
        "vec3 specular = vec3( 0.490196, 0.488722, 0.488722 ) * pow( RdotL, specularPower );\n"
        "vec3 color = ambient + diffuse + specular;\n"
        "\n"
        //
        "gl_FragColor = vec4( color, 1.0 );\n"
        "}\n";
        fragmentShader = new osg::Shader();
        fragmentShader->setType( osg::Shader::FRAGMENT );
        fragmentShader->setShaderSource( fragmentSource );
        osg::notify( osg::ALWAYS ) << "Using the inline fragment shader." << std::endl;
    }
    else
    {
        fragmentShader = 
            osg::Shader::readShaderFile( osg::Shader::FRAGMENT, shaderName );
        osg::notify( osg::ALWAYS ) << "Using the file fragment shader." << std::endl;
    }
    fragmentShader->setName( "parallax frag shader" );

    shaderName = osgDB::findDataFile( "parallax_mapping.vs" );
    osg::ref_ptr< osg::Shader > vertexShader;
    if( shaderName.empty() )
    {
        std::string vertexSource =
        "attribute vec4 a_tangent; \n"
        "attribute vec4 a_binormal;\n"
        
        "varying vec3 v_lightVector;\n"
        "varying vec3 v_viewVector;\n"
        "\n"
        "void main()\n"
        "{\n"
        //
        "gl_Position = ftransform();\n"
        "\n"
        //Get the texture coordinates
        "gl_TexCoord[ 0 ] = gl_TextureMatrix[ 0 ] * gl_MultiTexCoord0;\n"
        "\n"
        //Convert the vertex position into eye coordinates
        "vec3 ecPosition = vec3( gl_ModelViewMatrix * gl_Vertex );\n"
        "\n"
        //Convert tangent, binormal, and normal into eye coordinates
        "mat3 TBNMatrix = mat3( gl_ModelViewMatrix[0].xyz,gl_ModelViewMatrix[1].xyz,gl_ModelViewMatrix[2].xyz ) *\n"
        "    mat3( a_tangent.xyz, a_binormal.xyz, gl_Normal );\n"
        "\n"
        //Convert light vector into tangent space
        "v_lightVector = gl_LightSource[ 0 ].position.xyz - ecPosition;\n"
        "v_lightVector *= TBNMatrix;\n"
        "\n"
        //Convert view vector into tangent space
        "v_viewVector = ecPosition;\n"
        "v_viewVector *= TBNMatrix;\n"
        "}\n";
        vertexShader = new osg::Shader();
        vertexShader->setType( osg::Shader::VERTEX );
        vertexShader->setShaderSource( vertexSource );
        osg::notify( osg::ALWAYS ) << "Using the inline vertex shader." << std::endl;
    }
    else
    {
        vertexShader = osg::Shader::readShaderFile( osg::Shader::VERTEX, shaderName );
        osg::notify( osg::ALWAYS ) << "Using the file vertex shader." << std::endl;
    }
    vertexShader->setName( "parallax vertex shader" );

    osg::ref_ptr< osg::Program > program = new osg::Program();
    program->addShader( vertexShader.get() );
    program->addShader( fragmentShader.get() );
    program->addBindAttribLocation( "a_tangent", 6 );
    program->addBindAttribLocation( "a_binormal", 7 );
    program->addBindAttribLocation( "a_normal", 15 );
    stateSet->setAttribute( program.get(), osg::StateAttribute::ON );
    
    ///Now process the geometry
    //TSGVisitor tsgVisitor( node, 0 );    
    osgwTools::TangentSpaceOp* tso = new osgwTools::TangentSpaceOp;
    osgwTools::GeometryModifier gm( tso );
    node->accept( gm );
}
////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " map the normal texture to the loaded model." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "--height", "Path to the height texture." );
    arguments.getApplicationUsage()->addCommandLineOption( "--normal", "Path to the normal texture." );
    arguments.getApplicationUsage()->addCommandLineOption( "--base", "Path to the base texture if texture 0 is not the base texture." );
    arguments.getApplicationUsage()->addCommandLineOption( "-v/--version", "Display the osgWorks version string." );

    if( arguments.read( "-h" ) || arguments.read( "--help" ) )
    {
        osg::notify( osg::ALWAYS ) << arguments.getApplicationUsage()->getDescription() << std::endl;
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    ///Grab the normal map name
    if( arguments.find( "--normal" ) == -1 )
    {
        osg::notify( osg::FATAL ) << "There were no normal textures specified. Please supply a normal texture." << std::endl;
        arguments.getApplicationUsage()->
            write( osg::notify( osg::FATAL ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }
    std::string normalMapName;
    arguments.read( "--normal", normalMapName );

    ///Grab the height map name
    bool heightMap( false );
    std::string heightMapName;
    if( arguments.find( "--height" ) > 0 )
    {
        heightMap = true;
        arguments.read( "--height", heightMapName );
    }
    
    ///Grab the base map name
    bool baseMap( false );
    std::string baseMapName;
    if( arguments.find( "--base" ) > 0 )
    {
        baseMap = true;
        arguments.read( "--base", baseMapName );
    }
    
    if( arguments.read( "-v" ) || arguments.read( "--version" ) )
    {
        osg::notify( osg::ALWAYS ) << osgwTools::getVersionString() << std::endl << std::endl;
    }

    ///Now lets make the model
    osg::ref_ptr< osg::Group > root = new osg::Group();

    osg::ref_ptr< osg::Node > model = osgDB::readNodeFiles( arguments );
    if( model.get() == NULL )
    {
        osg::notify( osg::FATAL ) << "Unable to load model." << std::endl;
        arguments.getApplicationUsage()->
            write( osg::notify( osg::FATAL ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return( 1 );
    }

    root->addChild( model.get() );

    osg::ref_ptr< osg::Image > normalImage = osgDB::readImageFile( normalMapName );

    osg::ref_ptr< osg::Image > baseImage;
    if( baseMap )
    { 
        baseImage = osgDB::readImageFile( baseMapName );
    }

    osg::ref_ptr< osg::Image > heightImage;
    if( heightMap )
    {
       heightImage = osgDB::readImageFile( heightMapName );
    }

    ///Now setup the shaders
    addShaders( model.get(), baseImage.get(), normalImage.get(), heightImage.get() );

    osgViewer::Viewer viewer(arguments);
    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    viewer.setSceneData( root.get() );
    return viewer.run();
}



/** \page osgwnormalmap The osgwnormalmap Application
osgwnormalmap maps the specified normal map to the loaded file. We're assuming
that texture unit 0 contains the base texture for the model.

\code
C:\Projects>osgwnormalmap --normal file.dds --height file2.dds file.ive
\endcode

\section clp Command Line Parameters
<table border="0">
  <tr>
    <td><b>--normal <filename></b></td>
    <td>Specifies a normal texture. This is required.</td>
  </tr>
  <tr>
    <td><b>--height <filename></b></td>
    <td>Specifies a height map if desired.</td>
  </tr>
  <tr>
    <td><b>--base <filename></b></td>
    <td>Specifies a base map if the loaded model doesn't contain a base texture in unit 0.</td>
  </tr>
  <tr>
    <td><b>-v/--version <filename></b></td>
    <td>Displays the osgWorks version string.</td>
  </tr>
</table>
*/
