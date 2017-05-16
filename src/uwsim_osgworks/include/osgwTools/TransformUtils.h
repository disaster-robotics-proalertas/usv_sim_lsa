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

#ifndef __OSGWTOOLS_TRANSFORM_UTILS_H__
#define __OSGWTOOLS_TRANSFORM_UTILS_H__ 1


#include "osgwTools/Export.h"

#include <osg/NodeVisitor>


namespace osgwTools
{


/** \defgroup MatrixUtils Utilities for working with Matrices

*/
/*@{*/

/** \brief Same as the osg::computeLocalToWorld except that this accepts a 
osg::NodeMask so that hiden nodes can be transoformed.
*/
OSGWTOOLS_EXPORT osg::Matrix computeLocalToWorldWithNodeMask(const osg::NodePath& nodePath,  unsigned int mask = 0, bool ignoreCameras = true );
    
/*@}*/


// osgwTools
}

// __OSGWTOOLS_GRID_H__
#endif
