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

#include "osgwTools/ReadFile.h"
#include <osgDB/ReadFile>
#include <osg/Node>
#include <osg/Group>

#include <string>
#include <vector>


namespace osgwTools
{


osg::Node*
readNodeFiles( const std::string& fileNames )
{
    std::vector< std::string > files;
    std::string localFileNames( fileNames );
    while( !localFileNames.empty() )
    {
        std::string::size_type pos = localFileNames.find( ' ' );
        files.push_back( localFileNames.substr( 0, pos ) );
        if( pos != std::string::npos )
            localFileNames = localFileNames.substr( pos+1, localFileNames.size() );
        else
            localFileNames = std::string( "" );
    }

    return( osgDB::readNodeFiles( files ) );
}



// namespace osgwTools
}
