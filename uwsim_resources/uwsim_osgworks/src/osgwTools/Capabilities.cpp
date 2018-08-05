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

#include <osgwTools/Capabilities.h>
#include <osgwTools/Version.h>

#include <osgViewer/GraphicsWindow>
#include <osgViewer/Version>

#include <osg/Version>
#include <osg/GL>
#include <osg/GL2Extensions>
#include <osg/Texture3D>
#include <osg/TextureCubeMap>
#include <osg/FragmentProgram>


namespace osgwTools
{


Capabilities::Capabilities()
{
    query();
}
Capabilities::~Capabilities()
{
}

void Capabilities::query()
{
    // Error check. Most OpenGL implementations return an error
    // GL_INVALID_OPERATION if there is no current context.
    GLenum error( glGetError() );
    if( error != GL_NO_ERROR )
    {
        osg::notify( osg::WARN ) << "Error " << std::hex << error << " in osgwTools::Capabilities constructor." << std::endl;
        if( error == GL_INVALID_OPERATION )
        {
            osg::notify( osg::WARN ) << "This could indicate that there is no current OpenGL context." << std::endl;
        }
    }

    _osgVersion = osgGetVersion();

    _glVersion = (const char*)( glGetString( GL_VERSION ) );
    _glVendor = (const char*)( glGetString( GL_VENDOR ) );
    _glRenderer = (const char*)( glGetString( GL_RENDERER ) );
    _glslVersion = (const char*)( glGetString( GL_SHADING_LANGUAGE_VERSION ) );

    glGetIntegerv( GL_MAX_TEXTURE_SIZE, &_texSize );
    glGetIntegerv( GL_MAX_3D_TEXTURE_SIZE, &_3DTexSize );
    glGetIntegerv( GL_MAX_CUBE_MAP_TEXTURE_SIZE, &_cubeMapTexSize );
    glGetIntegerv( GL_MAX_TEXTURE_UNITS, &_maxTexUnits );

    glGetIntegerv( GL_MAX_TEXTURE_IMAGE_UNITS_ARB, &_shaderUnits );
    glGetIntegerv( GL_MAX_TEXTURE_COORDS_ARB, &_texCoords );
    glGetIntegerv( GL_MAX_VERTEX_ATTRIBS, &_vertexAttribs );
    glGetIntegerv( GL_MAX_DRAW_BUFFERS, &_drawBuffers );
}

void Capabilities::dump( std::ostream& ostr ) const
{
    ostr << getVersionString() << std::endl;
    ostr << "OSG version: " << _osgVersion << std::endl;

    ostr << "  Queryable strings ----------------------------------------" << std::endl;
    ostr << "    GL_VERSION: " << _glVersion << std::endl;
    ostr << "    GL_VENDOR: " << _glVendor << std::endl;
    ostr << "    GL_RENDERER: " << _glRenderer << std::endl;
    ostr << "    GL_SHADING_LANGUAGE_VERSION: " << _glslVersion << std::endl;

    ostr << "  Texture implementation constants -------------------------" << std::endl;
    ostr << "    GL_MAX_TEXTURE_SIZE: " << _texSize << std::endl;
    ostr << "    GL_MAX_3D_TEXTURE_SIZE: " << _3DTexSize << std::endl;
    ostr << "    GL_MAX_CUBE_MAP_TEXTURE_SIZE: " << _cubeMapTexSize << std::endl;
    ostr << "    GL_MAX_TEXTURE_UNITS: " << _maxTexUnits << std::endl;

    ostr << "  Shader implementation constants --------------------------" << std::endl;
    ostr << "    GL_MAX_TEXTURE_IMAGE_UNITS: " << _shaderUnits << std::endl;
    ostr << "    GL_MAX_TEXTURE_COORDS: " << _texCoords << std::endl;
    ostr << "    GL_MAX_VERTEX_ATTRIBS: " << _vertexAttribs << std::endl;
    ostr << "    GL_DRAW_BUFFERS: " << _drawBuffers << std::endl;
}



CapabilitiesSingleton* CapabilitiesSingleton::instance()
{
    static CapabilitiesSingleton* s_instance( new CapabilitiesSingleton() );
    return( s_instance );
}

const Capabilities* CapabilitiesSingleton::getCaps() const
{
    return( _caps );
}

CapabilitiesSingleton::CapabilitiesSingleton()
{
    struct SimpleContext
    {
        SimpleContext()
        {
            osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
            traits->width = 1;
            traits->height = 1;
            traits->pbuffer = true;

            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());

            if (!_gc)
            {
                osg::notify(osg::INFO)<<"Failed to create pbuffer, failing back to normal graphics window."<<std::endl;
                
                traits->pbuffer = false;
                _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
            }

            if (_gc.valid()) 
            {
                _gc->realize();
                _gc->makeCurrent();
                if (dynamic_cast<osgViewer::GraphicsWindow*>(_gc.get()))
                {
                    osg::notify(osg::INFO)<<"Realized graphics window for OpenGL operations."<<std::endl;
                }
                else
                {
                    osg::notify(osg::INFO)<<"Realized pbuffer for OpenGL operations."<<std::endl;
                }
            }
            else
            {
                osg::notify(osg::WARN)<<"Failed to create GC."<<std::endl;
            }
        }
        
        osg::ref_ptr<osg::GraphicsContext> _gc;
    };


    // Bizarre... Must call this function, or context creation will fail.
    ::osgViewerGetVersion();


    SimpleContext context;
    _caps = new Capabilities();
}
CapabilitiesSingleton::~CapabilitiesSingleton()
{
    delete _caps;
}


// namespace osgwTools
}
