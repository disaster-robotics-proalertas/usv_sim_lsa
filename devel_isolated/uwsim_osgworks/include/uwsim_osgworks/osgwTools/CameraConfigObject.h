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

#ifndef __OSGWORKS_CAMERA_CONFIG_OBJECT_H__
#define __OSGWORKS_CAMERA_CONFIG_OBJECT_H__ 1


#include <osgwTools/Export.h>
#include <osg/Object>
#include <osgViewer/Viewer>
#include <osg/Matrix>

#include <vector>


namespace osgwTools {



/** This is a convenience routine for loading a viewer configuration file and
configuring a viewer from it. If the configFile name is empty,
this function looks for OSGW_VIEWER_CONFIG in the environment and
attempts to load the file that it specifies. If vonfigFile is empty
and OSGW_VIEWER_CONFIG is not set, this function displays a message
to osg::NOTICE and returns false.

\test cameraconfig
*/
bool OSGWTOOLS_EXPORT configureViewer( osgViewer::Viewer& viewer, const std::string& configFile="" );


/** \class CameraConfigInfo CameraConfigObject.h <osgwTools/CameraConfigObject.h>
\brief A chunk of info for configuring a single slave camera, as found in osgViewer::View.
*/
class OSGWTOOLS_EXPORT CameraConfigInfo : public osg::Object
{
public:
    CameraConfigInfo();
    CameraConfigInfo( const osgwTools::CameraConfigInfo& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );

    META_Object( osgwTools, CameraConfigInfo );

    osg::Matrix _viewOffset;
    osg::Matrix _projectionOffset;

    unsigned int getVersion() const { return( _version ); }

protected:
    ~CameraConfigInfo();

    unsigned int _version;
};
typedef std::vector< osg::ref_ptr< CameraConfigInfo > > CameraConfigList;


/** \class CameraConfigObject CameraConfigObject.h <osgwTools/CameraConfigObject.h>
\brief A chunk of information for configuring a viewer. 

Your application can read and write this object to .osg 
files to save and restore Viewer configurations.
\code
    // For example...
    osgwTools::CameraConfigInfo* cci;
    osgwTools::writeObjectFile( *cci, "config.osg" );
    // or...
    cci = static_cast< osgwTools::CameraConfigInfo >(
        osgDB::readObjectFile( "config.osg" ) );
\endcode
*/
class OSGWTOOLS_EXPORT CameraConfigObject : public osg::Object
{
public:
    CameraConfigObject();
    CameraConfigObject( const osgwTools::CameraConfigObject& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );

    META_Object( osgwTools, CameraConfigObject );

    /** Cause the class to take (copy) the \c viewer parameter's current
    configuration. */
    void take( const osgViewer::Viewer& viewer );
    /** Cause the class to configure the \c viewer parameter from the
    configuration data stored in this class. */
    void store( osgViewer::Viewer& viewer );

    CameraConfigList _slaveConfigInfo;

    unsigned int getVersion() const { return( _version ); }

protected:
    ~CameraConfigObject();

    unsigned int _version;
};


// namespace osgwTools
}

// __OSGWORKS_CAMERA_CONFIG_OBJECT_H__
#endif
