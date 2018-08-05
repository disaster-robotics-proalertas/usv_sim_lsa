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

#ifndef __OSGBCOLLISION_COMPUTECYLINDERVISITOR_H__
#define __OSGBCOLLISION_COMPUTECYLINDERVISITOR_H__ 1

#include <osgbCollision/Export.h>
#include <osg/NodeVisitor>
#include <osgwTools/Version.h>

#include <osgbCollision/BoundingCylinder.h>

namespace osgbCollision
{


/** \class ComputeCylinderVisitor ComputeCylinderVisitor.h <osgbCollision/ComputeCylinderVisitor.h>
\brief Computes the extents of a cylinder around specified OSG data.

TBD Consider using OSG localtoworld method instead of keeping a matrix stack.
*/
class OSGBCOLLISION_EXPORT ComputeCylinderVisitor : public osg::NodeVisitor
{
public:
    ComputeCylinderVisitor( osg::NodeVisitor::TraversalMode traversalMode = TRAVERSE_ALL_CHILDREN );

#if( OSGWORKS_OSG_VERSION >= 20800 )
    META_NodeVisitor(osgbCollision,ComputeCylinderVisitor);
#endif

    virtual void reset();


    virtual void setAxis( const osg::Vec3 a )
    {
        axis = a;
        axis.normalize();
        bc.setAxis( axis );
    }

    osgbCollision::BoundingCylinder& getBoundingCylinder()
    {
        return( bc );
    }

    void apply( osg::Transform & transform );
    void apply( osg::Geode & geode );


    inline void pushMatrix( osg::Matrix & matrix )
    {
        stack.push_back( matrix );
    }

    inline void popMatrix()
    {
        stack.pop_back();
    }

    void applyDrawable( osg::Drawable * drawable );

protected:
    typedef std::vector< osg::Matrix >   MatrixStack;

    MatrixStack stack;
    BoundingCylinder bc;
    osg::Vec3 axis;
};


// osgbCollision
}


// __OSGBCOLLISION_COMPUTECYLINDERVISITOR_H__
#endif
