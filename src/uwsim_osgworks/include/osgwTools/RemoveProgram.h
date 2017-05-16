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

#ifndef __OSGWTOOLS_REMOVE_PROGRAM_H__
#define __OSGWTOOLS_REMOVE_PROGRAM_H__ 1


#include "osgwTools/Export.h"
#include <osg/NodeVisitor>

#include <string>



namespace osgwTools
{


/** \brief Visitor strips osg::Program objects and osg::Uniform objects from 
scene graph StateSet objects

This visitor is particularly useful for performance analysis and debugging.
If you suspect there is a bug in your shaders and wish to render your scene
using FFP instead, use this visitor to remove the programs. You can compare
the visual result between the shader- and FFP-renderings, as well as compare
the performance using the OSG state handler.
*/
class OSGWTOOLS_EXPORT RemoveProgram : public osg::NodeVisitor
{
public:
    /**
    Contructor to configure the RemoveProgram visitor. By default, RemoveProgram
    removes programs, doesn't remove uniforms, and traverses all children. This
    behavior is configurable by specifying your own values for the default parameters.
    @param removePrograms The default is true.
    @param removePrograms The default is false.
    @param travMode The traversal mode. The default is osg::NodeVisitor::TRAVERSE_ALL_CHILDREN.
    */
    RemoveProgram( bool removePrograms=true, bool removeUniforms=false, const osg::NodeVisitor::TraversalMode travMode=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
    ~RemoveProgram();

    /**
    Sets _programCount and _uniformCount to 0. /see _programCount. /see _uniformCount.
    */
    void reset();

    /**
    Overrides for base class apply() method.
    */
    virtual void apply( osg::Node& node );
    /**
    Overrides for base class apply() method.
    */
    virtual void apply( osg::Geode& node );

    /**
    Specifies whether to remove programs. The default is true. (Removes programs.)
    */
    void setRemovePrograms( bool removePrograms );
    /**
    Specifies whether to remove uniforms. The default is false. (Doesn't remove uniforms.)
    */
    void setRemoveUniforms( bool removeUniforms );

    /**
    During traversal, these counters track the total number of programs
    and uniforms removed. They are public, so calling code can access
    them directly following the traversal.
    */
    unsigned int _programCount, _uniformCount;

protected:
    void processStateSet( osg::StateSet* ss );

    bool _removePrograms, _removeUniforms;
};


// osgwTools
}

// __OSGWTOOLS_REMOVE_PROGRAM_H__
#endif
