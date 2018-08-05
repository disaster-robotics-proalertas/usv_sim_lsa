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

#ifndef __OSGWORKS_OSGWTOOLS_SERIALIZER_SUPPORT_H__
#define __OSGWORKS_OSGWTOOLS_SERIALIZER_SUPPORT_H__ 1

#include <osg/Vec2f>
#include <osg/Vec2d>
#include <osg/Vec3f>
#include <osg/Vec3d>
#include <osg/Vec4f>
#include <osg/Vec4d>
#include <boost/archive/basic_archive.hpp>


namespace boost {
namespace serialization {


/** \defgroup BoostSerializer Boost Serializer Support
\brief Classes that allow OSG data types to work with Boost Serializer
\details This header is not used internally. It is always available
in an osgWorks distribution regardless of whether osgWorks was built with
the optional Boost dependency. It is up to the application to include
or not include this header as needed.
*/
/**@{*/


template< class Archive >
void serialize( Archive& ar, osg::Vec2f& vec, const unsigned int version )
{
    ar & vec[0];
    ar & vec[1];
}
template< class Archive >
void serialize( Archive& ar, osg::Vec2d& vec, const unsigned int version )
{
    ar & vec[0];
    ar & vec[1];
}

template< class Archive >
void serialize( Archive& ar, osg::Vec3f& vec, const unsigned int version )
{
    ar & vec[0];
    ar & vec[1];
    ar & vec[2];
}
template< class Archive >
void serialize( Archive& ar, osg::Vec3d& vec, const unsigned int version )
{
    ar & vec[0];
    ar & vec[1];
    ar & vec[2];
}

template< class Archive >
void serialize( Archive& ar, osg::Vec4f& vec, const unsigned int version )
{
    ar & vec[0];
    ar & vec[1];
    ar & vec[2];
    ar & vec[3];
}
template< class Archive >
void serialize( Archive& ar, osg::Vec4d& vec, const unsigned int version )
{
    ar & vec[0];
    ar & vec[1];
    ar & vec[2];
    ar & vec[3];
}


/**@}*/


// serialization
}
// boost
}


// __OSGWORKS_OSGWTOOLS_SERIALIZER_SUPPORT_H__
#endif
