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

#include "osgwTools/RemoveProgram.h"
#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/Notify>



namespace osgwTools
{


RemoveProgram::RemoveProgram( bool removePrograms, bool removeUniforms, const osg::NodeVisitor::TraversalMode travMode )
  : osg::NodeVisitor( travMode ),
    _programCount( 0 ),
    _uniformCount( 0 ),
    _removePrograms( removePrograms ),
    _removeUniforms( removeUniforms )
{
}

RemoveProgram::~RemoveProgram()
{
}

void
RemoveProgram::reset()
{
    _programCount = 0;
    _uniformCount = 0;
}

void
RemoveProgram::setRemovePrograms( bool removePrograms )
{
    _removePrograms = removePrograms;
}
void
RemoveProgram::setRemoveUniforms( bool removeUniforms )
{
    _removeUniforms = removeUniforms;
}


void
RemoveProgram::apply( osg::Node& node )
{
    processStateSet( node.getStateSet() );
    traverse( node );
}
void
RemoveProgram::apply( osg::Geode& node )
{
    processStateSet( node.getStateSet() );

    unsigned int idx;
    for( idx=0; idx<node.getNumDrawables(); idx++ )
    {
        osg::Drawable* draw = node.getDrawable( idx );
        processStateSet( draw->getStateSet() );
    }

    traverse( node );
}

void
RemoveProgram::processStateSet( osg::StateSet* ss )
{
    if( ss == NULL )
        return;

    if( _removePrograms && ( ss->getAttribute( osg::StateAttribute::PROGRAM ) ) )
    {
        _programCount++;
        ss->removeAttribute( osg::StateAttribute::PROGRAM );
    }

    if( _removeUniforms )
    {
        _uniformCount += ss->getUniformList().size();
        ss->getUniformList().clear();
    }
}



// osgwTools
}
