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

#include <osgwTools/Uniqifier.h>
#include <osg/Group>
#include <osg/Geode>


namespace osgwTools
{


osg::Node* uniqify( osg::Node* child, osg::Group* parent )
{
    if( ( child == NULL ) || ( parent == NULL ) )
    {
        osg::notify( osg::WARN ) << "uniqify: One or more NULL parameters." << std::endl;
        return( NULL );
    }

    osg::Node* newChild = static_cast< osg::Node* >( child->clone( osg::CopyOp::SHALLOW_COPY ) );
    parent->replaceChild( child, newChild );
    return( newChild );
}


osg::NodePath uniqify( const osg::NodePath& np )
{
    if( np.size() < 2 )
    {
        osg::notify( osg::WARN ) << "uniqify: NodePath has size < 2." << std::endl;
        return( np );
    }

    osg::NodePath newPath;
    newPath.push_back( np.front() );

    unsigned int idx;
    for( idx=1; idx < np.size(); idx++ )
    {
        osg::Node* currentNode( np[ idx ] );
        if( currentNode->getNumParents() > 1 )
        {
            osg::Node* newNode = uniqify( np[ idx ], np[ idx-1 ]->asGroup() );
            if( newNode != NULL )
                newPath.push_back( newNode );
        }
        else
            newPath.push_back( currentNode );
    }

    return( newPath );
}


Uniqifier::Uniqifier( osg::NodeVisitor::TraversalMode mode )
  : osg::NodeVisitor( mode )
{
}

void Uniqifier::apply( osg::Node& node )
{
    // Unusual. A node that is neither a Group nor a Geode.
    osg::notify( osg::WARN ) << "Uniqifier: apply(osg::Node&)" << std::endl;

    traverse( node );
}
void Uniqifier::apply( osg::Group& node )
{
    // We need to iterate over all children. If a child has multiple
    // parents, we need to make individual unique copies of that child
    // and attach the copies to each parent.
    unsigned int idx;
    for( idx=0; idx<node.getNumChildren(); idx++ )
    {
        // sourceCandidate is the current child we might make copies of.
        osg::Node* sourceCandidate = node.getChild( idx );

        // Only need to make copies if getNumParents() > 1.
        while( sourceCandidate->getNumParents() > 1 )
        {
            // Make a shallow copy of sourceCandidate and replace parent 1's
            // sourceCandidate child with the new shallow copy.
            uniqify( sourceCandidate, sourceCandidate->getParent( 1 ) );

            // sourceCandidate's parent count has now decreased by 1.
            // Continue looping until the parent count equals 1.
        }
    }

    traverse( node );
}
void Uniqifier::apply( osg::Geode& node )
{
    // Nothing to do here.
    traverse( node );
}


// osgwTools
}
