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

#include <osgbDynamics/CreationRecord.h>

#include <iostream>
#include <string>

#include <osg/io_utils>

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>


bool Creation_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool Creation_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy Creation_Proxy
(
    new osgbDynamics::CreationRecord,
    "CreationRecord",
    "Object CreationRecord",
    Creation_readLocalData,
    Creation_writeLocalData
);




bool Creation_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::CreationRecord& cr = static_cast< osgbDynamics::CreationRecord& >( obj );
    bool advance( false );

    if( fr.matchSequence( "Version %i" ) )
    {
        fr[1].getUInt( cr._version );
        fr+=2;
        advance = true;
    }
    else if( fr.matchSequence( "COM %f %f %f" ) )
    {
        float x, y, z;
        fr[1].getFloat( x );
        fr[2].getFloat( y );
        fr[3].getFloat( z );
        cr._com = osg::Vec3( x, y, z );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Use COM" ) )
    {
        cr._comSet = ( fr[2].matchString( "true" ) );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Scale %f %f %f" ) )
    {
        float x, y, z;
        fr[1].getFloat( x );
        fr[2].getFloat( y );
        fr[3].getFloat( z );
        cr._scale = osg::Vec3( x, y, z );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Collision shape %i" ) )
    {
        unsigned int uint;
        fr[2].getUInt( uint );
        cr._shapeType = (BroadphaseNativeTypes)( uint );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Mass %f" ) )
    {
        fr[1].getFloat( cr._mass );
        fr+=2;
        advance = true;
    }
    else if( fr.matchSequence( "Decimator percent %f" ) )
    {
        // No longer supported.
        //fr[2].getFloat( cr._decimatorPercent );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Decimator max error %f" ) )
    {
        // No longer supported.
        //fr[3].getFloat( cr._decimatorMaxError );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Decimator ignore boundaries" ) )
    {
        // No longer supported.
        //cr._decimatorIgnoreBoundaries = ( fr[3].matchString( "true" ) );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Simplify percent %f" ) )
    {
        // No longer supported.
        //fr[2].getFloat( cr._simplifyPercent );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "VertexAgg max verts %i" ) )
    {
        // No longer supported.
        //fr[3].getUInt( cr._vertexAggMaxVerts );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "VertexAgg min cell size %f %f %f" ) )
    {
        // No longer supported.
        /*
        float x, y, z;
        fr[4].getFloat( x );
        fr[5].getFloat( y );
        fr[6].getFloat( z );
        cr._vertexAggMinCellSize = osg::Vec3( x, y, z );
        */
        fr+=7;
        advance = true;
    }
    else if( fr.matchSequence( "Reducer group threshold %f" ) )
    {
        // No longer supported.
        //fr[3].getFloat( cr._reducerGroupThreshold );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Reducer max edge error %f" ) )
    {
        // No longer supported.
        //fr[4].getFloat( cr._reducerMaxEdgeError );
        fr+=5;
        advance = true;
    }
    else if( fr.matchSequence( "Cylinder axis %i" ) )
    {
        unsigned int uint;
        fr[2].getUInt( uint );
        cr._axis = (osgbCollision::AXIS)( uint );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Reduction level %i" ) )
    {
        unsigned int uint;
        fr[2].getUInt( uint );
        cr._reductionLevel = (osgbDynamics::CreationRecord::ReductionLevel)( uint );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Overall" ) )
    {
        cr._overall = ( fr[1].matchString( "true" ) );
        fr+=2;
        advance = true;
    }

    return( advance );
}

bool Creation_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::CreationRecord& cr = static_cast< const osgbDynamics::CreationRecord& >( obj );

    fw.indent() << "Version " << 1 << std::endl;
    fw.indent() << "COM " << cr._com << std::endl;
    fw.indent() << "Use COM " << std::boolalpha << cr._comSet << std::endl;
    fw.indent() << "Scale " << cr._scale << std::endl;
    fw.indent() << "Collision shape " << (unsigned int)( cr._shapeType ) << std::endl;
    fw.indent() << "Mass " << cr._mass << std::endl;
    fw.indent() << "Cylinder axis " << cr._axis << std::endl;
    fw.indent() << "Reduction level " << cr._reductionLevel << std::endl;
    fw.indent() << "Overall " << std::boolalpha << cr._overall << std::endl;

    return( true );
}
