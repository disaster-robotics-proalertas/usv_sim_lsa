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

#include "osgwMx/MxUtils.h"

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


bool FunctionalMap_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool FunctionalMap_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy FunctionalMap_Proxy
(
    new osgwMx::FunctionalMap,
    "FunctionalMap",
    "Object FunctionalMap",
    FunctionalMap_readLocalData,
    FunctionalMap_writeLocalData
);



bool FunctionalMap_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgwMx::FunctionalMap& map = static_cast< osgwMx::FunctionalMap& >( obj );

    int idx;
    for( idx=0; idx<32; idx++ )
    {
        unsigned int key;
        fr[ 0 ].getUInt( key );
        const std::string& funcName( fr[ 1 ].getStr() );
        map.configure( key, osgwMx::FunctionalMap::asFunctionType( funcName ) );
        fr += 2;
    }
    
    return( true );
}

bool FunctionalMap_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgwMx::FunctionalMap& map = static_cast< const osgwMx::FunctionalMap& >( obj );

    unsigned int key( 1 );
    int idx;
    for( idx=0; idx<32; idx++, key<<=1 )
        fw.indent() << std::hex << "0x" << key << " " <<
            osgwMx::FunctionalMap::asString( map.getConfiguration( key ) ) << std::endl;

    return( true );
}

/*@}*/

/*@}*/
