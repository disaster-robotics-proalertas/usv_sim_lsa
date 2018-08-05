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

#include <osgwTools/Trianglizer.h>
#include <osg/Geometry>
#include <osg/PrimitiveSet>


namespace osgwTools
{


Trianglizer::Trianglizer()
    : GeometryOperation()
{
}
Trianglizer::Trianglizer( const Trianglizer& rhs, const osg::CopyOp& copyop )
    : GeometryOperation()
{
}
Trianglizer::~Trianglizer()
{
}


void convertDA( osg::ref_ptr< osg::DrawElementsUInt > master,
               osg::Geometry::PrimitiveSetList& newPSL,
               osg::ref_ptr< osg::PrimitiveSet > primSet )
{
    osg::DrawArrays* da( static_cast< osg::DrawArrays* >( primSet.get() ) );
    GLint first( da->getFirst() );
    GLsizei count( da->getCount() );
    switch( da->getMode() )
    {
    case GL_TRIANGLES:
        for( GLsizei idx=0; idx < count; ++idx )
            master->push_back( (GLuint)first++ );
        break;
    case GL_TRIANGLE_STRIP:
        for( GLsizei idx=2; idx < count; ++idx )
        {
            master->push_back( (GLuint)( first + idx ) );
            if( ( idx & 0x1 ) == 0 )
            {
                // Even: n, n-2, n-1
                master->push_back( (GLuint)( first + idx - 2 ) );
                master->push_back( (GLuint)( first + idx - 1 ) );
            }
            else
            {
                // Odd: n, n-1, n-2
                master->push_back( (GLuint)( first + idx - 1 ) );
                master->push_back( (GLuint)( first + idx - 2 ) );
            }
        }
        break;
    case GL_TRIANGLE_FAN:
        for( GLsizei idx=2; idx < count; ++idx )
        {
            master->push_back( (GLuint)( first ) );
            master->push_back( (GLuint)( first + idx - 1 ) );
            master->push_back( (GLuint)( first + idx ) );
        }
        break;
    case GL_QUADS:
        for( GLsizei idx=3; idx < count; idx += 4 )
        {
            master->push_back( (GLuint)( first + idx ) );
            master->push_back( (GLuint)( first + idx - 3 ) );
            master->push_back( (GLuint)( first + idx - 2 ) );

            master->push_back( (GLuint)( first + idx ) );
            master->push_back( (GLuint)( first + idx - 2 ) );
            master->push_back( (GLuint)( first + idx - 1 ) );
        }
        break;
    case GL_QUAD_STRIP:
        for( GLsizei idx=3; idx < count; idx += 4 )
        {
            master->push_back( (GLuint)( first + idx ) );
            master->push_back( (GLuint)( first + idx - 3 ) );
            master->push_back( (GLuint)( first + idx - 2 ) );

            master->push_back( (GLuint)( first + idx ) );
            master->push_back( (GLuint)( first + idx - 1 ) );
            master->push_back( (GLuint)( first + idx - 3 ) );
        }
        break;
    default:
        // Line, point, or adjacency prim type. Pass-through unaltered.
        newPSL.push_back( primSet );
        break;
    }
}
void convertDEUB( osg::ref_ptr< osg::DrawElementsUInt > master,
                 osg::Geometry::PrimitiveSetList& newPSL,
                 osg::ref_ptr< osg::PrimitiveSet > primSet )
{
    osg::DrawElementsUByte* deub( static_cast< osg::DrawElementsUByte* >( primSet.get() ) );
    const osg::VectorGLubyte& indices( *deub );
    GLsizei count( deub->size() );
    switch( deub->getMode() )
    {
    case GL_TRIANGLES:
        for( GLsizei idx=0; idx < count; ++idx )
            master->push_back( (GLuint)( indices[ idx ] ) );
        break;
    case GL_TRIANGLE_STRIP:
        for( GLsizei idx=2; idx < count; ++idx )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            if( ( idx & 0x1 ) == 0 )
            {
                // Even: n, n-2, n-1
                master->push_back( (GLuint)( indices[ idx - 2 ] ) );
                master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            }
            else
            {
                // Odd: n, n-1, n-2
                master->push_back( (GLuint)( indices[ idx - 1 ] ) );
                master->push_back( (GLuint)( indices[ idx - 2 ] ) );
            }
        }
        break;
    case GL_TRIANGLE_FAN:
        for( GLsizei idx=2; idx < count; ++idx )
        {
            master->push_back( (GLuint)( indices[ 0 ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            master->push_back( (GLuint)( indices[ idx ] ) );
        }
        break;
    case GL_QUADS:
        for( GLsizei idx=3; idx < count; idx += 4 )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );

            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
        }
        break;
    case GL_QUAD_STRIP:
        for( GLsizei idx=3; idx < count; idx += 4 )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );

            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
        }
        break;
    default:
        // Line, point, or adjacency prim type. Pass-through unaltered.
        newPSL.push_back( primSet );
        break;
    }
}
void convertDEUS( osg::ref_ptr< osg::DrawElementsUInt > master,
                 osg::Geometry::PrimitiveSetList& newPSL,
                 osg::ref_ptr< osg::PrimitiveSet > primSet )
{
    osg::DrawElementsUShort* deus( static_cast< osg::DrawElementsUShort* >( primSet.get() ) );
    const osg::VectorGLushort& indices( *deus );
    GLsizei count( deus->size() );
    switch( deus->getMode() )
    {
    case GL_TRIANGLES:
        for( GLsizei idx=0; idx < count; ++idx )
            master->push_back( (GLuint)( indices[ idx ] ) );
        break;
    case GL_TRIANGLE_STRIP:
        for( GLsizei idx=2; idx < count; ++idx )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            if( ( idx & 0x1 ) == 0 )
            {
                // Even: n, n-2, n-1
                master->push_back( (GLuint)( indices[ idx - 2 ] ) );
                master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            }
            else
            {
                // Odd: n, n-1, n-2
                master->push_back( (GLuint)( indices[ idx - 1 ] ) );
                master->push_back( (GLuint)( indices[ idx - 2 ] ) );
            }
        }
        break;
    case GL_TRIANGLE_FAN:
        for( GLsizei idx=2; idx < count; ++idx )
        {
            master->push_back( (GLuint)( indices[ 0 ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            master->push_back( (GLuint)( indices[ idx ] ) );
        }
        break;
    case GL_QUADS:
        for( GLsizei idx=3; idx < count; idx += 4 )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );

            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
        }
        break;
    case GL_QUAD_STRIP:
        for( GLsizei idx=3; idx < count; idx += 4 )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );

            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
        }
        break;
    default:
        // Line, point, or adjacency prim type. Pass-through unaltered.
        newPSL.push_back( primSet );
        break;
    }
}
void convertDEUI( osg::ref_ptr< osg::DrawElementsUInt > master,
                 osg::Geometry::PrimitiveSetList& newPSL,
                 osg::ref_ptr< osg::PrimitiveSet > primSet )
{
    osg::DrawElementsUInt* deui( static_cast< osg::DrawElementsUInt* >( primSet.get() ) );
    const osg::VectorGLuint& indices( *deui );
    GLsizei count( deui->size() );
    switch( deui->getMode() )
    {
    case GL_TRIANGLES:
        for( GLsizei idx=0; idx < count; ++idx )
            master->push_back( (GLuint)( indices[ idx ] ) );
        break;
    case GL_TRIANGLE_STRIP:
        for( GLsizei idx=2; idx < count; ++idx )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            if( ( idx & 0x1 ) == 0 )
            {
                // Even: n, n-2, n-1
                master->push_back( (GLuint)( indices[ idx - 2 ] ) );
                master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            }
            else
            {
                // Odd: n, n-1, n-2
                master->push_back( (GLuint)( indices[ idx - 1 ] ) );
                master->push_back( (GLuint)( indices[ idx - 2 ] ) );
            }
        }
        break;
    case GL_TRIANGLE_FAN:
        for( GLsizei idx=2; idx < count; ++idx )
        {
            master->push_back( (GLuint)( indices[ 0 ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            master->push_back( (GLuint)( indices[ idx ] ) );
        }
        break;
    case GL_QUADS:
        for( GLsizei idx=3; idx < count; idx += 4 )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );

            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
        }
        break;
    case GL_QUAD_STRIP:
        for( GLsizei idx=3; idx < count; idx += 4 )
        {
            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
            master->push_back( (GLuint)( indices[ idx - 2 ] ) );

            master->push_back( (GLuint)( indices[ idx ] ) );
            master->push_back( (GLuint)( indices[ idx - 1 ] ) );
            master->push_back( (GLuint)( indices[ idx - 3 ] ) );
        }
        break;
    default:
        // Line, point, or adjacency prim type. Pass-through unaltered.
        newPSL.push_back( primSet );
        break;
    }
}


osg::Geometry* Trianglizer::operator()( osg::Geometry& geom )
{
    if( !( needsConversion( geom ) ) )
        return( &geom );

    osg::ref_ptr< osg::DrawElementsUInt > master( new osg::DrawElementsUInt( GL_TRIANGLES ) );
    osg::Geometry::PrimitiveSetList newPSL;
    newPSL.push_back( master );

    const osg::Geometry::PrimitiveSetList& oldPSL( geom.getPrimitiveSetList() );
    for( unsigned int idx=0; idx < geom.getNumPrimitiveSets(); ++idx )
    {
        osg::ref_ptr< osg::PrimitiveSet > primSet( oldPSL[ idx ] );
        if( primSet->getType() == osg::PrimitiveSet::DrawArraysPrimitiveType )
            convertDA( master, newPSL, primSet );
        else if( primSet->getType() == osg::PrimitiveSet::DrawElementsUBytePrimitiveType )
            convertDEUB( master, newPSL, primSet );
        else if( primSet->getType() == osg::PrimitiveSet::DrawElementsUShortPrimitiveType )
            convertDEUS( master, newPSL, primSet );
        else if( primSet->getType() == osg::PrimitiveSet::DrawElementsUIntPrimitiveType )
            convertDEUI( master, newPSL, primSet );
        else
            // DrawArrayLengths
            newPSL.push_back( primSet );
    }

    geom.setPrimitiveSetList( newPSL );
    return( &geom );
}


bool Trianglizer::needsConversion( const osg::Geometry& geom )
{
    unsigned int count( 0 );
    const osg::Geometry::PrimitiveSetList& psl( geom.getPrimitiveSetList() );
    for( unsigned int idx=0; idx < geom.getNumPrimitiveSets(); ++idx )
    {
        if( psl[ idx ]->getType() == osg::PrimitiveSet::DrawArrayLengthsPrimitiveType )
            // We don't bother processing DrawArrayLengths.
            continue;

        if( psl[ idx ]->getType() != osg::PrimitiveSet::DrawElementsUIntPrimitiveType )
            return( true ); // Needs conversion.

        const GLenum mode( psl[ idx ]->getMode() );
        switch( mode )
        {
        case GL_TRIANGLE_STRIP:
        case GL_TRIANGLE_FAN:
        case GL_QUADS:
        case GL_QUAD_STRIP:
            return( true ); // Needs conversion.
            break;
        case GL_TRIANGLES:
            ++count; // Find out how many GL_TRIANGLES PrimitiveSets we have.
            break;
        default:
            // A line, point, or new adjacency type. Don't convert.
            continue;
        }
    }
    if( count > 1 )
        // More than one GL_TRIANGLES PrimitiveSet. Needs convertion.
        return( true );
    else
        return( false );
}


// osgwTools
}
