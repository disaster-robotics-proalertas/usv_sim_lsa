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

#include "osgwTools/InsertRemove.h"
#include <osg/Node>
#include <osg/Group>


namespace osgwTools
{

void
insertAbove( osg::Node* node, osg::Group* newParent )
{
    // Don't let the node get deleted when we remove it from all its parents.
    // Equivalent to explicit call to node->ref(), then node->unref() at end of function.
    osg::ref_ptr< osg::Node > nodeHolder( node );

    osg::Node::ParentList pl = node->getParents();
    osg::Node::ParentList::iterator it;
    for( it = pl.begin(); it != pl.end(); it++ )
    {
        osg::Group* oldParent( *it );
        oldParent->addChild( newParent );
        oldParent->removeChild( node );
    }
    newParent->addChild( node );
}

void
insertBelow( osg::Group* parent, osg::Group* newChild )
{
    unsigned int idx;
    for( idx=0; idx<parent->getNumChildren(); idx++ )
        newChild->addChild( parent->getChild( idx ) );
    parent->removeChildren( 0, parent->getNumChildren() );
    parent->addChild( newChild );
}

void
removeNode( osg::Node* node )
{
    osg::Group* asGrp = node->asGroup();

    // Don't let the node get deleted when we remove it from all its parents.
    // Equivalent to explicit call to node->ref(), then node->unref() at end of function.
    osg::ref_ptr< osg::Node > nodeHolder( node );

    osg::Node::ParentList pl = node->getParents();
    osg::Node::ParentList::iterator it;
    for( it = pl.begin(); it != pl.end(); it++ )
    {
        // Remove 'node' from its parent.
        osg::Group* parent( *it );
        parent->removeChild( node );

        if( asGrp != NULL )
        {
            // Add all of node's children to the parent.
            unsigned int idx;
            for( idx=0; idx < asGrp->getNumChildren(); idx++ )
                parent->addChild( asGrp->getChild( idx ) );
        }
    }
}



// namespace osgwTools
}
