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


int main( int argc, char** argv )
{
    osgwTools::CapabilitiesSingleton::instance()->getCaps()->dump( osg::notify( osg::ALWAYS ) );

    return( 0 );
}



/** \page osgwcaps The osgwcaps Application
osgwcaps queries the graphics system for basic capabilities
and displays the results to OSG's ALWAYS notification level.

Internally, osgwcaps uses the osgwTools::CapabilitiesSingleton. See also
the osgwTools::Capabilities struct.

*/
