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

#include <osgwTools/FindNamedNode.h>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <iostream>


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    std::string nodeName;
    arguments.read( "-n", nodeName );

    if( argc != 3 )
    {
        std::cerr << "osgwconv [-n <nodename>] <infile> <outfile>" << std::endl;
        return( 1 );
    }
    
    const std::string inFile( arguments.argv()[ 1 ] );
    const std::string outFile( arguments.argv()[ 2 ] );

    osg::ref_ptr< osg::Node > root( osgDB::readNodeFile( inFile ) );
    if( root == NULL )
    {
        std::cerr << "Can't read file \"" << inFile << "\"." << std::endl;
        return( 1 );
    }

    osg::Node* candidate( root.get() );
    if( !( nodeName.empty() ) )
    {
        osgwTools::FindNamedNode fnn( nodeName );
        root->accept( fnn );
        if( !( fnn._napl.empty() ) )
        {
            std::cout << "Found " << fnn._napl.size() << " candidate nodes." << std::endl;
            candidate = fnn._napl[ 0 ].first;
        }
        else
        {
            std::cout << "Cound not find node names \"" << nodeName << "\"." << std::endl;
            return( 1 );
        }
    }

    std::cout << "Writing to \"" << outFile << "\"..." << std::endl;
    osgDB::writeNodeFile( *candidate, outFile );

    return( 0 );
}



/** \page osgwconv The osgwconv Application
osgwconv converts models from one format to another. Unlike
osgconv, osgwconv does not run the osgUtil::Optimizer.

I -n <nodename> is present, osgwconv searches for the first node
with the specified name, and outputs the subgraph rooted at
that node. This is especially useful for examining portions of
large models in a human readable format such as .osg.

*/
