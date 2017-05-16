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
#include "osgwTools/NodeUtils.h"

#include <osgDB/WriteFile>
#include <osg/Node>
#include <osg/Group>


void
testSimpleInsertAbove()
{
    osg::ref_ptr< osg::Group > oldParent( new osg::Group );
    oldParent->setName( "oldParent" );

    osg::ref_ptr< osg::Node > child( new osg::Node );
    child->setName( "child" );
    oldParent->addChild( child.get() );

    osgDB::writeNodeFile( *oldParent, "simpleinsertabove0.dot" );

    osg::ref_ptr< osg::Group > newParent( new osg::Group );
    newParent->setName( "newParent" );

    osgwTools::insertAbove( child.get(), newParent.get() );

    osgDB::writeNodeFile( *oldParent, "simpleinsertabove1.dot" );
}
void
testComplexInsertAbove()
{
    osg::ref_ptr< osg::Group > oldParent0( new osg::Group );
    oldParent0->setName( "oldParent0" );
    osg::ref_ptr< osg::Group > oldParent1( new osg::Group );
    oldParent1->setName( "oldParent1" );
    osg::ref_ptr< osg::Group > oldParent2( new osg::Group );
    oldParent2->setName( "oldParent2" );

    osg::ref_ptr< osg::Node > child( new osg::Node );
    child->setName( "child" );
    oldParent0->addChild( child.get() );
    oldParent1->addChild( child.get() );
    oldParent2->addChild( child.get() );

    osg::ref_ptr< osg::Group > root( new osg::Group );
    root->addChild( oldParent0.get() );
    root->addChild( oldParent1.get() );
    root->addChild( oldParent2.get() );
    osgDB::writeNodeFile( *root, "complexinsertabove0.dot" );

    osg::ref_ptr< osg::Group > newParent( new osg::Group );
    newParent->setName( "newParent" );

    osgwTools::insertAbove( child.get(), newParent.get() );

    osgDB::writeNodeFile( *root, "complexinsertabove1.dot" );
}
void
testSimpleInsertBelow()
{
    osg::ref_ptr< osg::Group > oldParent( new osg::Group );
    oldParent->setName( "oldParent" );

    osg::ref_ptr< osg::Node > child( new osg::Node );
    child->setName( "child" );
    oldParent->addChild( child.get() );

    osgDB::writeNodeFile( *oldParent, "simpleinsertbelow0.dot" );

    osg::ref_ptr< osg::Group > newChild( new osg::Group );
    newChild->setName( "newChild" );

    osgwTools::insertBelow( oldParent.get(), newChild.get() );

    osgDB::writeNodeFile( *oldParent, "simpleinsertbelow1.dot" );
}
void
testComplexInsertBelow()
{
    osg::ref_ptr< osg::Group > oldParent( new osg::Group );
    oldParent->setName( "oldParent" );

    osg::ref_ptr< osg::Node > child0( new osg::Node );
    child0->setName( "child0" );
    oldParent->addChild( child0.get() );
    osg::ref_ptr< osg::Node > child1( new osg::Node );
    child1->setName( "child1" );
    oldParent->addChild( child1.get() );
    osg::ref_ptr< osg::Node > child2( new osg::Node );
    child2->setName( "child2" );
    oldParent->addChild( child2.get() );

    osgDB::writeNodeFile( *oldParent, "complexinsertbelow0.dot" );

    osg::ref_ptr< osg::Group > newChild( new osg::Group );
    newChild->setName( "newChild" );

    osgwTools::insertBelow( oldParent.get(), newChild.get() );

    osgDB::writeNodeFile( *oldParent, "complexinsertbelow1.dot" );
}


void
testSimpleRemove()
{
    osg::ref_ptr< osg::Group > root( new osg::Group );
    root->setName( "root" );

    osg::ref_ptr< osg::Group > removalTarget( new osg::Group );
    removalTarget->setName( "removalTarget" );
    root->addChild( removalTarget.get() );

    osg::ref_ptr< osg::Node > child( new osg::Node );
    child->setName( "child" );
    removalTarget->addChild( child.get() );

    osgDB::writeNodeFile( *root, "simpleremove0.dot" );

    osgwTools::removeNode( removalTarget.get() );

    osgDB::writeNodeFile( *root, "simpleremove1.dot" );
}
void
testComplexRemove()
{
    osg::ref_ptr< osg::Group > root( new osg::Group );
    root->setName( "root" );

    osg::ref_ptr< osg::Group > parent0( new osg::Group );
    parent0->setName( "parent0" );
    root->addChild( parent0.get() );
    osg::ref_ptr< osg::Group > parent1( new osg::Group );
    parent1->setName( "parent1" );
    root->addChild( parent1.get() );

    osg::ref_ptr< osg::Group > removalTarget( new osg::Group );
    removalTarget->setName( "removalTarget" );
    parent0->addChild( removalTarget.get() );
    parent1->addChild( removalTarget.get() );

    osg::ref_ptr< osg::Node > child0( new osg::Node );
    child0->setName( "child0" );
    removalTarget->addChild( child0.get() );
    osg::ref_ptr< osg::Node > child1( new osg::Node );
    child1->setName( "child1" );
    removalTarget->addChild( child1.get() );
    osg::ref_ptr< osg::Node > child2( new osg::Node );
    child2->setName( "child2" );
    removalTarget->addChild( child2.get() );

    osgDB::writeNodeFile( *root, "complexremove0.dot" );

    osgwTools::removeNode( removalTarget.get() );

    osgDB::writeNodeFile( *root, "complexremove1.dot" );
}

void
testSimpleReplace()
{
    osg::ref_ptr< osg::Group > root( new osg::Group );
    root->setName( "root" );

    osg::ref_ptr< osg::Group > parent0( new osg::Group );
    parent0->setName( "parent0" );
    root->addChild( parent0.get() );
    osg::ref_ptr< osg::Group > parent1( new osg::Group );
    parent1->setName( "parent1" );
    root->addChild( parent1.get() );

    // original
	osg::ref_ptr< osg::Group > replaceSource( new osg::Group );
    replaceSource->setName( "replaceSource" );
    parent0->addChild( replaceSource.get() );
    parent1->addChild( replaceSource.get() );

    osg::ref_ptr< osg::Node > child0S( new osg::Node );
    child0S->setName( "child0S" );
    replaceSource->addChild( child0S.get() );
    osg::ref_ptr< osg::Node > child1S( new osg::Node );
    child1S->setName( "child1S" );
    replaceSource->addChild( child1S.get() );
    osg::ref_ptr< osg::Node > child2S( new osg::Node );
    child2S->setName( "child2S" );
    replaceSource->addChild( child2S.get() );

	// replacement
    osg::ref_ptr< osg::Group > replaceTarget( new osg::Group );
    replaceTarget->setName( "replaceTarget" );

    osg::ref_ptr< osg::Node > child0R( new osg::Node );
    child0R->setName( "child0R" );
    replaceTarget->addChild( child0R.get() );
    osg::ref_ptr< osg::Node > child1R( new osg::Node );
    child1R->setName( "child1R" );
    replaceTarget->addChild( child1R.get() );
    osg::ref_ptr< osg::Node > child2R( new osg::Node );
    child2R->setName( "child2R" );
    replaceTarget->addChild( child2R.get() );


    osgDB::writeNodeFile( *root, "simplereplace_before.dot" );

    osgwTools::replaceSubgraph( replaceTarget.get(), replaceSource.get() );

    osgDB::writeNodeFile( *root, "simplereplace_after.dot" );
}

int
main( int argc, char ** argv )
{
    testSimpleInsertAbove();
    testComplexInsertAbove();
    testSimpleInsertBelow();
    testComplexInsertBelow();

    testSimpleRemove();
    testComplexRemove();

	testSimpleReplace();

    osg::notify( osg::ALWAYS ) << "Test results written as .dot files. To view them, try:" << std::endl;
    osg::notify( osg::ALWAYS ) << "\tdot -Tpng -O simpleremove1.dot" << std::endl;
    osg::notify( osg::ALWAYS ) << "\tosgviewer --image simpleremove1.dot.png" << std::endl;

    return( 0 );
}

