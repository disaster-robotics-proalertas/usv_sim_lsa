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

#include <osgwTools/ParallelVisitor.h>
#include <osg/Node>
#include <osg/Group>
#include <osg/Notify>

#include <stdlib.h>

namespace osgwTools
{


ParallelVisitor::ParallelVisitor( osg::Node* sgA, osg::Node* sgB )
  : _compareResult( true ),
    _sgA( sgA ),
    _sgB( sgB ),
    _pvcb( NULL )
{
}
ParallelVisitor::~ParallelVisitor()
{
}

bool
ParallelVisitor::compare()
{
    return( recurseCompare( _sgA.get(), _sgB.get() ) );
}

void
ParallelVisitor::setCallback( ParallelVisitorCallback* pvcb )
{
    _pvcb = pvcb;
}
ParallelVisitor::ParallelVisitorCallback*
ParallelVisitor::getCallback() const
{
    return( _pvcb );
}

bool
ParallelVisitor::isMatch( const osg::Node& nodeA, const osg::Node& nodeB ) const
{
    return( nodeA.className() == nodeB.className() );
}

bool
ParallelVisitor::recurseCompare( osg::Node* nodeA, osg::Node* nodeB )
{
    osg::Group* grpA( nodeA->asGroup() );
    osg::Group* grpB( nodeB->asGroup() );
    if( (grpA == NULL) || (grpB == NULL) )
        return( true );

    bool matched( true );
    unsigned int idx;
    for( idx=0;
         idx < osg::minimum( grpA->getNumChildren(), grpB->getNumChildren() );
         idx++ )
    {
        osg::ref_ptr< osg::Node > childA( grpA->getChild( idx ) );
        osg::ref_ptr< osg::Node > childB( grpB->getChild( idx ) );
        if( !isMatch( *childA, *childB ) )
        {
            matched = false;
            if( _pvcb != NULL )
            {
                bool advance( (*_pvcb)( *childA, *childB ) );
                if( !advance )
                    idx--;
            }
        }
    }

    const unsigned int numChildren( osg::minimum( grpA->getNumChildren(), grpB->getNumChildren() ) );
    for( idx=0; idx<numChildren; idx++ )
    {
        if( !recurseCompare( grpA->getChild( idx ), grpB->getChild( idx ) ) )
            matched = false;
    }
    return( matched );
}


// osgwTools
}
