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

#ifndef __OSGWTOOLS_REMOVE_DATA__
#define __OSGWTOOLS_REMOVE_DATA__ 1


#include "osgwTools/Export.h"
#include <osg/NodeVisitor>

#include <vector>
#include <string>

namespace osg {
    class Geometry;
}


namespace osgwTools
{


/** \brief Removes specified data from the scene graph
In v1.1.52, this visitor supports deleting empty StateSets.
This task was formerly performed by the (deprecated) CountStateSets visitor.
*/
class OSGWTOOLS_EXPORT RemoveData : public osg::NodeVisitor
{
public:
    RemoveData( unsigned int flags=DEFAULT, const osg::NodeVisitor::TraversalMode travMode=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
    ~RemoveData();

    enum RemovalFlags {
        STATESETS = (0x1 << 0),
        STATESET_TEXTURES = (0x1 << 1),
        EMPTY_STATESETS = (0x1 << 2),
        DRAWABLES = (0x1 << 3),
        GEOMETRY_ARRAYS = (0x1 << 4),
        GEOMETRY_PRIMITIVESETS = (0x1 << 5),
        GEODES = (0x1 << 6),
        USERDATA = (0x1 << 7),
        DESCRIPTIONS = (0x1 << 8),
        DEFAULT = ( STATESETS |
            DRAWABLES |
            DESCRIPTIONS ),
        ALL = ( STATESETS | STATESET_TEXTURES |
            DRAWABLES | GEOMETRY_ARRAYS | GEOMETRY_PRIMITIVESETS |
            GEODES | USERDATA |
            DESCRIPTIONS )
    };
    void setRemovalFlags( unsigned int flags );
    unsigned int getRemovalFlags() const;

    void addRemoveMode( GLenum mode );
    void addRemoveAttribute( osg::StateAttribute::Type attribute );

    static std::string flagsToString( unsigned int flags );
    static unsigned int stringToFlags( const std::string& str );

    virtual void apply( osg::Node& node );
    virtual void apply( osg::Group& node );
    virtual void apply( osg::Geode& node );

protected:
    unsigned int _removalFlags;

    void apply( osg::StateSet* ss );
    void apply( osg::Geometry& geom );

    typedef std::vector< GLenum > ModeVector;
    typedef std::vector< osg::StateAttribute::Type > AttributeTypeVector;
    ModeVector _removeModes;
    AttributeTypeVector _removeAttrs;
};


// osgwTools
}

// __OSGWTOOLS_REMOVE_DATA__
#endif
