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

#include <string>


namespace osgwTools
{


RefID::RefID()
{
}
RefID::RefID( const std::string& id )
{
    set( id );
}

RefID::RefID( const RefID& rhs, const osg::CopyOp copyop )
{
    _str = rhs._str;
}
RefID::~RefID()
{
}

bool
RefID::operator==( const RefID& rhs ) const
{
    return( _str == rhs._str );
}
bool
RefID::operator<( const RefID& rhs ) const
{
    return( _str < rhs._str );
}

void
RefID::set( const std::string& id )
{
    _str = id;
}
const std::string&
RefID::str() const
{
    return( _str );
}


// osgwTools
}
