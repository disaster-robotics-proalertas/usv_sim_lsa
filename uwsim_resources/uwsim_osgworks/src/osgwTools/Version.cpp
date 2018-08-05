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

#include "osgwTools/Version.h"
#include <string>
#include <sstream>

namespace osgwTools {


unsigned int
getVersionNumber()
{
    return( OSGWORKS_VERSION );
}


static std::string s_osgworks_version( "" );

std::string
getVersionString()
{
    if( s_osgworks_version.empty() )
    {
        std::ostringstream oStr;
        oStr << std::string( "osgWorks version " ) <<
            OSGWORKS_MAJOR_VERSION << "." <<
            OSGWORKS_MINOR_VERSION << "." <<
            OSGWORKS_SUB_VERSION << " (" <<
            getVersionNumber() << ").";
        s_osgworks_version = oStr.str();
    }
    return( s_osgworks_version );
}


// namespace osgwTools
}
