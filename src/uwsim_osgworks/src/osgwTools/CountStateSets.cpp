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

#include "osgwTools/CountStateSets.h"
#include <osgwTools/StateSetUtils.h>
#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/Notify>



namespace osgwTools
{


CountStateSets::CountStateSets( bool removeEmptyStateSets, const osg::NodeVisitor::TraversalMode travMode )
  : osg::NodeVisitor( travMode ),
    _uniqueStateSets( 0 ),
    _sharedStateSets( 0 ),
    _emptyStateSets( 0 ),
    _removedStateSets( 0 ),
    _removeEmptyStateSets( removeEmptyStateSets )
{
}

CountStateSets::~CountStateSets()
{
}

void
CountStateSets::reset()
{
    _uniqueStateSets = 0;
    _sharedStateSets = 0;
    _emptyStateSets = 0;
    _removedStateSets = 0;
}

void
CountStateSets::setRemoveEmptyStateSets( bool removeEmptyStateSets )
{
    _removeEmptyStateSets = removeEmptyStateSets;
}


void
CountStateSets::apply( osg::Node& node )
{
    if( !processStateSet( node.getStateSet() ) && _removeEmptyStateSets )
    {
        node.setStateSet( NULL );
        _removedStateSets++;
    }
    traverse( node );
}
void
CountStateSets::apply( osg::Geode& node )
{
    if( !processStateSet( node.getStateSet() ) && _removeEmptyStateSets )
    {
        node.setStateSet( NULL );
        _removedStateSets++;
    }

    unsigned int idx;
    for( idx=0; idx<node.getNumDrawables(); idx++ )
    {
        osg::Drawable* draw = node.getDrawable( idx );
        if( !processStateSet( draw->getStateSet() ) && _removeEmptyStateSets )
        {
            draw->setStateSet( NULL );
            _removedStateSets++;
        }
    }

    traverse( node );
}

bool
CountStateSets::processStateSet( osg::StateSet* ss )
{
    if( ss == NULL )
        return( true );

    if( ss->referenceCount() == 1 )
        _uniqueStateSets++;
    else
        _sharedStateSets++;

    bool empty = osgwTools::isEmpty( *ss );
    if( empty )
        _emptyStateSets++;
    return( !empty );
}



// osgwTools
}
