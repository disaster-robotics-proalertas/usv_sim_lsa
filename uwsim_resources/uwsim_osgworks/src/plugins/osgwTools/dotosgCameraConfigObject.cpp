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

#include "osgwTools/CameraConfigObject.h"

#include <iostream>

#include <osg/Matrix>
#include <osg/io_utils>

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osgDB/ParameterOutput>

#include <vector>


/** \addtogroup Plugins
@{*/

/** \addtogroup DotOSGSupport
@{*/


bool CCObject_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool CCObject_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy CCObject_Proxy
(
    new osgwTools::CameraConfigObject,
    "CameraConfigObject",
    "Object CameraConfigObject",
    CCObject_readLocalData,
    CCObject_writeLocalData
);

bool CCInfo_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool CCInfo_writeLocalData( const osg::Object& obj, osgDB::Output& fw );
osgDB::RegisterDotOsgWrapperProxy CCInfo_Proxy
(
    new osgwTools::CameraConfigInfo,
    "CameraConfigInfo",
    "Object CameraConfigInfo",
    CCInfo_readLocalData,
    CCInfo_writeLocalData
);


bool readMatrix( osg::Matrix& matrix, osgDB::Input& fr, const char* keyword="Matrix" );
bool writeMatrix( const osg::Matrixd& matrix, osgDB::Output& fw, const char* keyword="Matrix" );


bool
CCInfo_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgwTools::CameraConfigInfo& cci = static_cast< osgwTools::CameraConfigInfo& >( obj );
    bool advance( false );

    unsigned int version( 0 );
    if( fr.matchSequence( "Version %i" ) )
    {
        fr[1].getUInt( version );
        fr+=2;
        advance = true;
    }

    osg::Matrix m;
    if( readMatrix( m, fr, "ViewOffset" ))
        cci._viewOffset = m;
    if( readMatrix( m, fr, "ProjectionOffset" ))
        cci._projectionOffset = m;

    return( advance );
}
bool
CCInfo_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgwTools::CameraConfigInfo& cci = static_cast< const osgwTools::CameraConfigInfo& >( obj );

    fw.indent() << "Version " << cci.getVersion() << std::endl;
    writeMatrix( cci._viewOffset, fw, "ViewOffset" );
    writeMatrix( cci._projectionOffset, fw, "ProjectionOffset" );

    return( true );
}

bool
CCObject_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgwTools::CameraConfigObject& cco = static_cast< osgwTools::CameraConfigObject& >( obj );
    bool advance( false );

    unsigned int version( 0 );
    if( fr.matchSequence( "Version %i" ) )
    {
        fr[1].getUInt( version );
        fr+=2;
        advance = true;
    }

    if( fr[0].getStr() == std::string( "SlaveCameraCount" ) )
    {
        int sz;
        fr[1].getInt( sz );
        cco._slaveConfigInfo.resize( sz );
        fr+=2;
        advance = true;
    }
    unsigned int idx;
    for( idx=0; idx<cco._slaveConfigInfo.size(); idx++ )
    {
        osgwTools::CameraConfigInfo* cci = static_cast< osgwTools::CameraConfigInfo* >( fr.readObject() );
        if( idx > cco._slaveConfigInfo.size() )
        {
            osg::notify( osg::WARN ) << "Camera config data contains too many slaves; resizing..." << std::endl;
            cco._slaveConfigInfo.resize( idx );
        }
        cco._slaveConfigInfo[ idx ] = cci;
    }
    
    return( advance );
}

bool
CCObject_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgwTools::CameraConfigObject& cco = static_cast< const osgwTools::CameraConfigObject& >( obj );

    fw.indent() << "Version " << cco.getVersion() << std::endl;
    fw.indent() << "SlaveCameraCount " << cco._slaveConfigInfo.size() << std::endl;
    unsigned int idx;
    for( idx=0; idx<cco._slaveConfigInfo.size(); idx++ )
    {
        fw.writeObject( *( cco._slaveConfigInfo[ idx ] ) );
    }

    return( true );
}

/*@}*/

/*@}*/
