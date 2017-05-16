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

#ifndef __OSGBINTERACTION_GESTURE_HANDLER_H__
#define __OSGBINTERACTION_GESTURE_HANDLER_H__


#include <osgbInteraction/Export.h>
#include <functional>
#include <osg/Referenced>
#include <osg/ref_ptr>
#include <vector>


// Forward
class btGeneric6DofConstraint;


namespace osgbInteraction
{


// Forward
class HandNode;


/** \class GestureHandler GestureHandler.h <osgbInteraction/GestureHandler.h>
\brief Allows data gloves to trigger interations via gesture codes.

*/
class OSGBINTERACTION_EXPORT GestureHandler : public osg::Referenced
{
public:
    virtual bool operator()( const unsigned int gestureCode, HandNode& handNode ) = 0;

    static const unsigned int Unknown;
    static const unsigned int Default;
    static const unsigned int Point;
    static const unsigned int Fist;
};

typedef std::vector< osg::ref_ptr< GestureHandler > > GestureHandlerVector;


class OSGBINTERACTION_EXPORT GripRelease : public GestureHandler
{
public:
    GripRelease();

    virtual bool operator()( const unsigned int gestureCode, HandNode& handNode );

protected:
    ~GripRelease();

    btGeneric6DofConstraint* _constraint;
};


// osgbInteraction
}

// __OSGBINTERACTION_GESTURE_HANDLER_H__
#endif
