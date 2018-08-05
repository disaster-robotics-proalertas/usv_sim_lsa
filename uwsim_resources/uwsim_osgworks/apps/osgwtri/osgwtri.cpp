/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2013 by Kenneth Mark Bryden
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

#include <osgwTools/GeometryModifier.h>
#include <osgwTools/Trianglizer.h>

#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Notify>


int main( int argc, char** argv )
{
    if( argc != 3 )
    {
        OSG_FATAL << "Usage: osgwtri <inputfile> <outputfile>" << std::endl;
        return( 1 );
    }

    osg::ref_ptr< osg::Node > model( osgDB::readNodeFile( std::string( argv[ 1 ] ) ) );
    if( model == NULL )
    {
        OSG_FATAL << "Can't read input file \"" << argv[ 1 ] << "\"" << std::endl;
        return( 1 );
    }

    osgwTools::GeometryModifier gm( new osgwTools::Trianglizer() );
    model->accept( gm );

    osgDB::writeNodeFile( *model, std::string( argv[ 2 ] ) );

    return( 0 );
}



/** \page osgwtri The osgwtri Application
osgwtri runs the Trianglizer GeometryOperation on the specified
input model, and writes the processed model to disk.

For each osg::Geometry object in the input model, osgwtri
performs the following modifications:
\li All GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN, GL_QUADS, and
GL_QUAD_STRIP PrimitiveSets are converted to plain GL_TRIANGLES.
\li All DrawArrays, DrawElementsUByte, and DrawElementsUShort
PrimitiveSets are converted to DrawElementsUInt.
\li Multiple DrawElementsUInt / GL_TRIANGLES PrimitiveSets
are combined into a single DrawElementsUInt / GL_TRIANGLES
PrimitiveSet.

*/
