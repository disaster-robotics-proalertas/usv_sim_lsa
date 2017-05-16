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

#ifndef __OSGWTOOLS_REMOVE_LOD__
#define __OSGWTOOLS_REMOVE_LOD__ 1


#include "osgwTools/Export.h"
#include <osg/NodeVisitor>

#include <string>

namespace osg {
    class Geometry;
}


namespace osgwTools
{


/** \brief Finds LOD Nodes and collapses them to have only one child (the highest LOD)

\deprecated Please use \ref CollapseLOD instead.

*/
class OSGWTOOLS_EXPORT RemoveLOD : public osg::NodeVisitor
{
public:
    osgwDEPRECATED( RemoveLOD( const osg::NodeVisitor::TraversalMode travMode=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ) );
    ~RemoveLOD();

    virtual void apply( osg::LOD& node );
};


// osgwTools
}

// __OSGWTOOLS_REMOVE_LOD__
#endif
