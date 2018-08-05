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

#ifndef __SIMPLIFIER_OP_H__
#define __SIMPLIFIER_OP_H__


#include <osgwTools/GeometryOperation.h>
#include <osg/CopyOp>
#include <osg/Object>
#include <osgUtil/Simplifier>
#include <osgwTools/Export.h>

namespace osgwTools {


/** \brief A geometry reduction wrapper around the osgUtil::Simplifier.*/

class OSGWTOOLS_EXPORT SimplifierOp : public GeometryOperation
{
public:
    SimplifierOp();
    SimplifierOp( const SimplifierOp& rhs, const osg::CopyOp& copyOp=osg::CopyOp::SHALLOW_COPY );

    META_Object(osgwTools,SimplifierOp);

    virtual osg::Geometry* operator()( osg::Geometry& geom );

    // Make this public as a quick and dirty way to configure the simplification.
    osg::ref_ptr< osgUtil::Simplifier > _simplifier;

protected:
    ~SimplifierOp();

};

}

#endif

