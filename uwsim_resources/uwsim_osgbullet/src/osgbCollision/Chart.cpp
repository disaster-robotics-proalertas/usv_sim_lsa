/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2012 by Kenneth Mark Bryden
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

#include <osgbCollision/Chart.h>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Texture1D>
#include <osg/Uniform>
#include <osg/Shader>
#include <osgDB/FileUtils>


namespace osgbCollision
{

////////////////////////////////////////////////////////////////////////////////
Chart::Chart()
  : _x( .05f ),
    _y( .05f ),
    _w( .25f ),
    _h( .1f ),
    _yScale( 30.f ),
    _texW( 256 ),
    _bg( osg::Vec4( 0.f, 0.f, 0.f, .33f ) ),
    _fg( osg::Vec4( 1.f, 1.f, 1.f, .5f ) ),
    _overrun( osg::Vec4( 1.f, 0.1f, 0.1f, .5f ) )
{
    _fgUniform = new osg::Uniform( "foreground", _fg );
    _bgUniform = new osg::Uniform( "background", _bg );
}
////////////////////////////////////////////////////////////////////////////////
Chart::~Chart()
{
    // We create the Image with USE_NEW_DELETE so we do *not* need
    // to explitly delete this image data.
    //delete[] _xValues;
}
////////////////////////////////////////////////////////////////////////////////
void
Chart::setValue( int idx, float value )
{
    if( idx >= _texW )
    {
        const int n( idx / _texW );
        idx = idx - n * _texW;
    }
    _xValues[ idx ] = value / _yScale;
    // Unfortunately, OSG doesn't support just updating a single value.
    // Dirty the whole image.
    _image->dirty();
}
////////////////////////////////////////////////////////////////////////////////
osg::Geode*
Chart::get() const
{
    return( _geode.get() );
}
////////////////////////////////////////////////////////////////////////////////
void Chart::setBackgroundColor( osg::Vec4& bgColor )
{
    _bg = bgColor;
    _bgUniform->set( _bg );
}
////////////////////////////////////////////////////////////////////////////////
void Chart::setForegroundColor( osg::Vec4& fgColor )
{
    _fg = fgColor;
    _fgUniform->set( _fg );
}
////////////////////////////////////////////////////////////////////////////////
void Chart::setChartLocationAndSize( float x, float y, float w, float h )
{
    _x = x;
    _y = y;
    _w = w;
    _h = h;

    if( _verts.valid() )
    {
        (*_verts)[ 0 ] = osg::Vec3( _x, _y, 0. );
        (*_verts)[ 1 ] = osg::Vec3( _x+_w, _y, 0. );
        (*_verts)[ 2 ] = osg::Vec3( _x+_w, _y+_h, 0. );
        (*_verts)[ 3 ] = osg::Vec3( _x, _y+_h, 0. );
    }
}
////////////////////////////////////////////////////////////////////////////////
void Chart::createChart()
{
    // Init the 1D array of texture values. This will be the 1D array sampled in the fragment shader.
    _xValues = new float[ _texW ];
    int idx;
    for( idx=0; idx<_texW; idx++ )
        _xValues[ idx ] = 0.f;
    
    _geode = new osg::Geode;
    _geom = new osg::Geometry;
    _geom->setUseDisplayList( false );
    _geom->setUseVertexBufferObjects( false );
    _geom->setDataVariance( osg::Object::DYNAMIC );
    {
        // Create stateset to draw the chart quad. Just a single texture mapped quad with blending and no lighting, depth test ALWAYS.
        osg::StateSet* ss = _geom->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
        ss->setAttributeAndModes( new osg::BlendFunc );
        ss->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ) );
        
        // Load vert and frag shaders.
        osg::ref_ptr< osg::Program > program = new osg::Program();
        ss->setAttribute( program.get(),
                         osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );
        
        std::string shaderName = osgDB::findDataFile( "hud.vs" );
        if( !shaderName.empty() )
        {
            osg::ref_ptr< osg::Shader > vertShader = osg::Shader::readShaderFile( osg::Shader::VERTEX, shaderName );
            program->addShader( vertShader.get() );
        }
        else
        {
            osg::notify( osg::WARN ) << "Chart::createChart(): Cannot find hud.vs." << std::endl;
        }
        
        shaderName = osgDB::findDataFile( "hud.fs" );
        if( !shaderName.empty() )
        {
            osg::ref_ptr< osg::Shader > fragShader = osg::Shader::readShaderFile( osg::Shader::FRAGMENT, shaderName );
            program->addShader( fragShader.get() );
        }
        else
        {
            osg::notify( osg::WARN ) << "Chart::createChart(): Cannot find hud.fs." << std::endl;
        }
        
        // Uniforms for color values and 1D texture width.
        ss->addUniform( _fgUniform.get() );
        ss->addUniform( _bgUniform.get() );
        ss->addUniform( new osg::Uniform( "overrun", _overrun ) );
        ss->addUniform( new osg::Uniform( "texwidth", _texW ) );
        
        // Create a 1D texture object for the values array and create a corresponding sampler uniform.
        _image = new osg::Image;
        _image->setImage( _texW, 1, 1, GL_INTENSITY32F_ARB, GL_RED, GL_FLOAT,
                         (unsigned char*) _xValues, osg::Image::USE_NEW_DELETE );
        osg::Texture1D* texVal = new osg::Texture1D;
        texVal->setImage( _image.get() );
        texVal->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
        texVal->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
        texVal->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
        ss->setTextureAttributeAndModes( 0, texVal );
        
        ss->addUniform( new osg::Uniform( "texVal", 0 ) );
    }
    _geode->addDrawable( _geom.get() );
    
    _verts = new osg::Vec3Array;
    _verts->resize( 4 );
    _geom->setVertexArray( _verts.get() );
    _tc = new osg::Vec2Array;
    _tc->resize( 4 );
    _geom->setTexCoordArray( 0, _tc.get() );
    
    
    (*_verts)[ 0 ] = osg::Vec3( _x, _y, 0. );
    (*_verts)[ 1 ] = osg::Vec3( _x+_w, _y, 0. );
    (*_verts)[ 2 ] = osg::Vec3( _x+_w, _y+_h, 0. );
    (*_verts)[ 3 ] = osg::Vec3( _x, _y+_h, 0. );
    (*_tc)[ 0 ] = osg::Vec2( 0., 0. );
    (*_tc)[ 1 ] = osg::Vec2( 1., 0. );
    (*_tc)[ 2 ] = osg::Vec2( 1., 1. );
    (*_tc)[ 3 ] = osg::Vec2( 0., 1. );
    
    _geom->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );
}
////////////////////////////////////////////////////////////////////////////////


// osgbCollision
}
