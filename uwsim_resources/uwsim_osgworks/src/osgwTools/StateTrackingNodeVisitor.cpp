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

#include <osgwTools/StateTrackingNodeVisitor.h>
#include <osgwTools/StateSetUtils.h>
#include <osg/StateSet>
#include <osg/Notify>


namespace osgwTools
{




StateTrackingNodeVisitor::StateTrackingNodeVisitor( osg::NodeVisitor::TraversalMode mode )
  : osg::NodeVisitor( mode )
{
}
StateTrackingNodeVisitor::~StateTrackingNodeVisitor()
{
}


void StateTrackingNodeVisitor::pushStateSet( osg::StateSet* ss )
{
    if( ss == NULL )
        ss = new osg::StateSet;

    if( _stateStack.size() > 0 )
    {
        osg::StateSet* oldTop = _stateStack.back().get();
        osg::StateSet* newTop = new osg::StateSet( *oldTop );
        newTop->merge( *ss );
        _stateStack.push_back( newTop );
    }
    else
    {
        _stateStack.push_back( ss );
    }
}
void StateTrackingNodeVisitor::popStateSet()
{
    if( _stateStack.size() > 0 )
        _stateStack.pop_back();
    else
        osg::notify( osg::WARN ) << "osgwTools: StateTrackingNodeVisitor: State stack underflow." << std::endl;
}
void StateTrackingNodeVisitor::pushTraversePop( osg::StateSet* ss, osg::Node& node )
{
    pushStateSet( ss );
    traverse( node );
    popStateSet();
}


// osgwTools
}
