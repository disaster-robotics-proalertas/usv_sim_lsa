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

#ifndef __OSGWTOOLS_FORCE_FLATTEN_TRANSFORMS_H__
#define __OSGWTOOLS_FORCE_FLATTEN_TRANSFORMS_H__ 1

#include <osgwTools/Export.h>
#include <osg/NodeVisitor>
#include <osgwTools/Version.h>


namespace osgwTools
{


/** class ForceFlattenTransforms ForceFlattenTransforms.h <osgwTools/ForceFlattenTransforms.h>
\brief Force flattening of all transforms

This is more heavy-handed than osgUtil::Optimizer. It really flattens
the transforms, with no regard to static or dynamic variance, update
callbacks, or other things that would prevent the Optimizer from
flattening the transforms.

TBD How is multiparenting handled?
*/
class OSGWTOOLS_EXPORT ForceFlattenTransforms : public osg::NodeVisitor
{
public:
    ForceFlattenTransforms();

#if( OSGWORKS_OSG_VERSION > 20800 )
    META_NodeVisitor(osgwTools,ForceFlattenTransforms);
#endif

    void apply( osg::Transform& node );
    void apply( osg::MatrixTransform& node );
    void apply( osg::PositionAttitudeTransform& node );
    void apply( osg::Geode& node );

protected:
    void flattenDrawable( osg::Drawable* drawable, const osg::Matrix& matrix );
};


// osgwTools
}


// __OSGWTOOLS_FORCE_FLATTEN_TRANSFORMS_H__
#endif
