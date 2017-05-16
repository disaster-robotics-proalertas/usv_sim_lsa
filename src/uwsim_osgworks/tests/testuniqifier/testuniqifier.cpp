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
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>
#include <osg/Notify>

#include <string>
#include <set>


//#define CREATE_TEST_FILE
#ifdef CREATE_TEST_FILE

#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/ProxyNode>
#include <osg/Material>

void createTestFile()
{
    osg::ref_ptr< osg::Group > root = new osg::Group;
    root->setName( "testuniqifier" );

    osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( -.8, -.8, 0. ) );
    osg::Material* mat = new osg::Material;
    mat->setDiffuse( osg::Material::FRONT, osg::Vec4( 1., 0., 0., 1. ) );
    mt->getOrCreateStateSet()->setAttributeAndModes( mat );
    root->addChild( mt );
    osg::ProxyNode* proxy = new osg::ProxyNode;
    proxy->setFileName( 0, "teapot.osg" );
    mt->addChild( proxy );

    mt = new osg::MatrixTransform( osg::Matrix::translate( 0., 0., 0. ) );
    mat = new osg::Material;
    mat->setDiffuse( osg::Material::FRONT, osg::Vec4( 0., .8, 0., 1. ) );
    mt->getOrCreateStateSet()->setAttributeAndModes( mat );
    root->addChild( mt );
    mt->addChild( proxy );

    mt = new osg::MatrixTransform( osg::Matrix::translate( .8, .8, 0. ) );
    mat = new osg::Material;
    mat->setDiffuse( osg::Material::FRONT, osg::Vec4( .2, .2, 1., 1. ) );
    mt->getOrCreateStateSet()->setAttributeAndModes( mat );
    root->addChild( mt );
    mt->addChild( proxy );

    osgDB::writeNodeFile( *root, "testuniqifier.osg" );
}

#endif

/* \cond */
class NodeCounter : public osg::NodeVisitor
{
public:
    NodeCounter()
      : osg::NodeVisitor( TRAVERSE_ACTIVE_CHILDREN )
    {
        reset();
    }
    void reset()
    {
        _totalNodes = 0;
        _uniqueNodes.clear();
    }

    virtual void apply( osg::Node& node )
    {
        _totalNodes++;
        _uniqueNodes.insert( &node );
        traverse( node );
    }

    unsigned int _totalNodes;

    typedef std::set< osg::Node* > NodeSet;
    NodeSet _uniqueNodes;
};
/* \endcond */


int main( int argc, char** argv )
{
#ifdef CREATE_TEST_FILE
    createTestFile();
#endif

    osg::notify( osg::ALWAYS ) <<
        "This is a CTest regression test. To launch under Visual Studio, build the" << std::endl <<
        "RUN_TESTS target. Under Linux, enter 'make test' at a shell prompty." << std::endl <<
        std::endl;


    const std::string testFile( "testuniqifier.osg" );
    osg::Node* root = osgDB::readNodeFile( testFile );
    if( root == NULL )
    {
        osg::notify( osg::FATAL ) << "ERROR: Unable to load \"" << testFile << "\"." << std::endl;
        return( 1 );
    }

    // Replace (the single) ProxyNode with a Group.
    // (After the test completes, we write the uniqified scene graph.
    // However, ProxyNodes never write their children, so the output
    // file doesn't give us an accurate representation of the uniqify
    // process. Replacing ProxyNodes with Groups resolves that issue.)
    osgUtil::Optimizer optimizer;
    optimizer.optimize( root,
        osgUtil::Optimizer::REMOVE_LOADED_PROXY_NODES );


    // Get the total number of nodes, and number of unique nodes.
    NodeCounter nc;
    root->accept( nc );
    const unsigned int preUniqifyNumNodes = nc._totalNodes;
    const unsigned int preUniqifyUniqueNodes = nc._uniqueNodes.size();
    // Double check to make sure we have the input scene graph
    // that we are expecting to have.
    if( preUniqifyNumNodes == preUniqifyUniqueNodes )
    {
        osg::notify( osg::FATAL ) << "ERROR: Problem with input data." << std::endl;
        return( 1 );
    }


    osgwTools::Uniqifier uniqifier;
    root->accept( uniqifier );


    // Get node counts a second time.
    nc.reset();
    root->accept( nc );

    // Pass/fail condition. The total number of nodes should not
    // have changed. But the Uniqifier should have changed the
    // number of unique nodes so that it is not the same as the
    // total number of nodes.
    bool passed( ( nc._totalNodes == preUniqifyNumNodes ) &&
        ( nc._uniqueNodes.size() == preUniqifyNumNodes ) );


    if( !passed )
        osg::notify( osg::FATAL ) << "ERROR: Test failed." << std::endl;
    else
        osg::notify( osg::ALWAYS ) << "Test Passed." << std::endl;

    const std::string resultFile( "testuniqifier-result.osg" );
    osgDB::writeNodeFile( *root, resultFile );
    osg::notify( osg::ALWAYS ) << "Result of Uniqification written to \"" << resultFile << "\"." << std::endl;

    return( passed ? 0 : 1 );
}
