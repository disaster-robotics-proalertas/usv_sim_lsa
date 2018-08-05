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

#include "osgwTools/RefID.h"

#include <iostream>
#include <string>

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osgDB/ParameterOutput>


/** \addtogroup Plugins
@{*/

/** \addtogroup DotOSGSupport
@{*/


bool RefID_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool RefID_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy RefID_Proxy
(
    new osgwTools::RefID,
    "RefID",
    "Object RefID",
    RefID_readLocalData,
    RefID_writeLocalData
);



bool RefID_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgwTools::RefID& rid = static_cast< osgwTools::RefID& >( obj );
    bool advance( false );

    if( fr[0].getStr() == std::string( "String" ) )
    {
        rid.set( fr[1].getStr() );
        fr+=2;
        advance = true;
    }
    
    return( advance );
}

bool RefID_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgwTools::RefID& rid = static_cast< const osgwTools::RefID& >( obj );

    fw.indent() << "String \"" << rid.str() << "\"" << std::endl;

    return( true );
}

/*@}*/

/*@}*/
