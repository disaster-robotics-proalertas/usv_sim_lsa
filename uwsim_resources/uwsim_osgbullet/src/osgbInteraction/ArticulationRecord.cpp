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
#include <osg/Object>
#include <osg/Vec3d>


namespace osgbInteraction
{


ArticulationRecord::ArticulationRecord()
  : _version( 2 )
{
}
ArticulationRecord::ArticulationRecord( const osg::Vec3d& axis, const osg::Vec3d& pivotPoint )
  : _axis( axis ),
    _pivotPoint( pivotPoint ),
    _version( 2 )
{
}

ArticulationRecord::ArticulationRecord( const ArticulationRecord& rhs, const osg::CopyOp& copyop )
  : _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _version( rhs._version )
{
}


ArticulationRecord::~ArticulationRecord()
{
}


// osgbInteraction
}
