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

#ifndef __OSGWORKS_OSGWTOOLS_CAPABILITIES_H__
#define __OSGWORKS_OSGWTOOLS_CAPABILITIES_H__ 1


#include <osgwTools/Export.h>

#include <osg/GraphicsContext>

#include <iostream>
#include <string>


namespace osgwTools
{


/** \class Capabilities Capabilities.h <osgwTools/Capabilities.h>
\brief Queries and displays graphivs capabilities.
\details The constructor must be invoked while a context is current.
The constructor queries the OSG version and various OpenGL parameters
and stores them in the Capabilities struct. The app can examine each
field directly, or call Capabilities::dump() to display the values to
a std::stream.

See CapabilitiesSingleton.
*/
struct OSGWTOOLS_EXPORT Capabilities
{
    Capabilities();
    ~Capabilities();

    /** \brief Query values.
    \details Apps should not need to call this function directly, as
    it is invoked by the constructor. */
    void query();

    /** \brief Dump queried values to the specified std::ostream.
    */
    void dump( std::ostream& ostr ) const;


    std::string _osgVersion;

    std::string _glVersion;
    std::string _glVendor;
    std::string _glRenderer;
    std::string _glslVersion;

    GLint _texSize;
    GLint _3DTexSize;
    GLint _cubeMapTexSize;
    GLint _maxTexUnits;

    GLint _shaderUnits;
    GLint _texCoords;
    GLint _vertexAttribs;
    GLint _drawBuffers;
};


/** \class CapabilitiesSingleton Capabilities.h <osgwTools/Capabilities.h>
\brief Obtain Capabilities values without requiring a current context.
\details When the singleton instance is first allocated and constructed,
it creates its own pbuffer context to initialize its Capabilities member
variable. */
class OSGWTOOLS_EXPORT CapabilitiesSingleton
{
public:
    static CapabilitiesSingleton* instance();

    const Capabilities* getCaps() const;

protected:
    CapabilitiesSingleton();
    ~CapabilitiesSingleton();

    Capabilities* _caps;
};


// namespace osgwTools
}


// __OSGWORKS_OSGWTOOLS_CAPABILITIES_H__
#endif
