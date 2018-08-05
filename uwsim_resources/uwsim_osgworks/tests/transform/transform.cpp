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

#include "osgwTools/Transform.h"

#include <osg/BoundingSphere>
#include <osg/BoundingBox>
#include <osg/Matrix>
#include <osg/Notify>
#include <osg/io_utils>


static osg::BoundingSphere sphere( osg::Vec3( 0., 0., 0. ), 1. );
static osg::BoundingBox box( -1., -1., -1., 1., 1., 1. );

void print( std::ostream& ostr, const osg::BoundingSphere& s )
{
    ostr << "    center: " << s._center << std::endl;
    ostr << "    radius: " << s._radius << std::endl;
}
void pretest( const osg::BoundingSphere& s )
{
    osg::notify( osg::ALWAYS ) << "----------------------------------------" << std::endl;
    osg::notify( osg::ALWAYS ) << "  Input:" << std::endl;
    print( osg::notify( osg::ALWAYS ), s );
}
void posttest( const osg::BoundingSphere& s )
{
    osg::notify( osg::ALWAYS ) << "  Result:" << std::endl;
    print( osg::notify( osg::ALWAYS ), s );
    osg::notify( osg::ALWAYS ) << std::endl;
}

void print( std::ostream& ostr, const osg::BoundingBox& b )
{
    ostr << "    min: " << b._min << std::endl;
    ostr << "    max: " << b._max << std::endl;
}
void pretest( const osg::BoundingBox& b )
{
    osg::notify( osg::ALWAYS ) << "----------------------------------------" << std::endl;
    osg::notify( osg::ALWAYS ) << "  Input:" << std::endl;
    print( osg::notify( osg::ALWAYS ), b );
}
void posttest( const osg::BoundingBox& b )
{
    osg::notify( osg::ALWAYS ) << "  Result:" << std::endl;
    print( osg::notify( osg::ALWAYS ), b );
    osg::notify( osg::ALWAYS ) << std::endl;
}


void
testScaleSphere()
{
    pretest( sphere );

    osg::Vec3 scale( 3., 3., 3. );
    osg::notify( osg::ALWAYS ) << "  Scaling by: " << scale << std::endl;
    osg::Matrix m( osg::Matrix::scale( scale ) );
    osg::BoundingSphere newSphere = osgwTools::transform( m, sphere );

    posttest( newSphere );
}
void
testScaleTranslateSphere()
{
    pretest( sphere );

    osg::Vec3 scale( 2., 2., 2. );
    osg::Vec3 trans( 0., -5., 0. );
    osg::notify( osg::ALWAYS ) << "  Scaling by: " << scale << std::endl;
    osg::notify( osg::ALWAYS ) << "  Translating by: " << trans << std::endl;
    osg::notify( osg::ALWAYS ) << "  Order: result = input * scale * trans" << std::endl;
    osg::Matrix m( osg::Matrix::scale( scale ) * osg::Matrix::translate( trans ) );
    osg::BoundingSphere newSphere = osgwTools::transform( m, sphere );

    posttest( newSphere );
}
void
testScaleRotateSphere()
{
    pretest( sphere );

    osg::Vec3 scale( 2., 2., 2. );
    float degrees( 45.f );
    osg::Vec3 axis( 1., 0., 0. );
    osg::notify( osg::ALWAYS ) << "  Scaling by: " << scale << std::endl;
    osg::notify( osg::ALWAYS ) << "  Rotating " << degrees << " degrees around: " << axis << std::endl;
    osg::Matrix m( osg::Matrix::scale( scale ) * osg::Matrix::rotate( osg::DegreesToRadians( degrees ), axis ) );
    osg::BoundingSphere newSphere = osgwTools::transform( m, sphere );

    posttest( newSphere );
}
void
testNonUniformScale()
{
    pretest( sphere );

    osg::Vec3 scale( 4., 5., 6. );
    osg::notify( osg::ALWAYS ) << "  Scaling by: " << scale << std::endl;
    osg::Matrix m( osg::Matrix::scale( scale ) );
    osg::BoundingSphere newSphere = osgwTools::transform( m, sphere );

    posttest( newSphere );
}


void
testScaleBox()
{
    pretest( box );

    osg::Vec3 scale( 3., 3., 3. );
    osg::notify( osg::ALWAYS ) << "  Scaling by: " << scale << std::endl;
    osg::Matrix m( osg::Matrix::scale( scale ) );
    osg::BoundingBox newBox = osgwTools::transform( m, box );

    posttest( newBox );
}
void
testRotateBox()
{
    pretest( box );

    float degrees( 45.f );
    osg::Vec3 axis( 1., 0., 0. );
    osg::notify( osg::ALWAYS ) << "  Rotating " << degrees << " degrees around: " << axis << std::endl;
    osg::Matrix m( osg::Matrix::rotate( osg::DegreesToRadians( degrees ), axis ) );
    osg::BoundingBox newBox = osgwTools::transform( m, box );

    posttest( newBox );
}


int
main( int argc, char ** argv )
{
    testScaleSphere();
    testScaleTranslateSphere();
    testScaleRotateSphere();
    testNonUniformScale();

    testScaleBox();
    testRotateBox();

    return( 0 );
}

