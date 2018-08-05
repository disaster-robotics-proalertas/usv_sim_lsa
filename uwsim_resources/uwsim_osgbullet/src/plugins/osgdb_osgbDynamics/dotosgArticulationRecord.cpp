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

#include <osgbInteraction/ArticulationRecord.h>

#include <iostream>
#include <string>

#include <osg/io_utils>

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>


bool Articulation_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool Articulation_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy Articulation_Proxy
(
    new osgbInteraction::ArticulationRecord,
    "ArticulationRecord",
    "Object ArticulationRecord",
    Articulation_readLocalData,
    Articulation_writeLocalData
);




bool Articulation_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbInteraction::ArticulationRecord& ar = static_cast< osgbInteraction::ArticulationRecord& >( obj );
    bool advance( false );

    if( fr.matchSequence( "Version %i" ) )
    {
        fr[1].getUInt( ar._version );
        fr+=2;
        advance = true;
    }
    else if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        double x, y, z;
        fr[1].getFloat( x );
        fr[2].getFloat( y );
        fr[3].getFloat( z );
        ar._axis = osg::Vec3d( x, y, z );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "PivotPoint %f %f %f" ) )
    {
        double x, y, z;
        fr[1].getFloat( x );
        fr[2].getFloat( y );
        fr[3].getFloat( z );
        ar._pivotPoint = osg::Vec3d( x, y, z );
        fr+=4;
        advance = true;
    }

    return( advance );
}

bool Articulation_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbInteraction::ArticulationRecord& ar = static_cast< const osgbInteraction::ArticulationRecord& >( obj );

    fw.indent() << "Version " << ar._version << std::endl;
    fw.indent() << "Axis " << ar._axis << std::endl;
    fw.indent() << "PivotPoint " << ar._pivotPoint << std::endl;

    return( true );
}
