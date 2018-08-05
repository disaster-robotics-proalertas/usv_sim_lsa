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

#include <osgwTools/PrimitiveSetConversion.h>


namespace osgwTools
{


osg::DrawElementsUInt* daToDeuiInternal( unsigned int start, unsigned int count, GLenum mode )
{
    osg::ref_ptr< osg::DrawElementsUInt > deui = new osg::DrawElementsUInt;
    deui->setMode( mode );
    deui->resize( count );

    unsigned int deuiIdx = 0;
    unsigned int srcIdx = start;
    while( deuiIdx < count )
        (*deui)[ deuiIdx++ ] = srcIdx++;

    return( deui.release() );
}

osg::DrawElementsUInt* convertToDEUI( const osg::DrawArrays* da )
{
    if( da == NULL ) return( NULL );
    return( daToDeuiInternal( da->getFirst(), da->getCount(), da->getMode() ) );
}

osg::Geometry::PrimitiveSetList convertToDEUI( const osg::DrawArrayLengths* dal )
{
    osg::Geometry::PrimitiveSetList psl;

    unsigned int first = dal->getFirst();
    unsigned int idx;
    for( idx=0; idx<dal->size(); idx++ )
    {
        unsigned int count = (*dal)[ idx ];
        psl.push_back( daToDeuiInternal( first, count, dal->getMode() ) );
        first += count;
    }

    return( psl );
}

osg::DrawElementsUInt* convertToDEUI( const osg::DrawElementsUByte* deub )
{
    if( deub == NULL ) return( NULL );

    osg::ref_ptr< osg::DrawElementsUInt > deui = new osg::DrawElementsUInt;
    deui->setMode( deub->getMode() );
    deui->resize( deub->size() );

    unsigned int deuiIdx = 0;
    const unsigned char* srcIdx = static_cast< const unsigned char* >( deub->getDataPointer() );
    while( deuiIdx < deub->size() )
        (*deui)[ deuiIdx++ ] = *srcIdx++;

    return( deui.release() );
}

osg::DrawElementsUInt* convertToDEUI( const osg::DrawElementsUShort* deus )
{
    if( deus == NULL ) return( NULL );

    osg::ref_ptr< osg::DrawElementsUInt > deui = new osg::DrawElementsUInt;
    deui->setMode( deus->getMode() );
    deui->resize( deus->size() );

    unsigned int deuiIdx = 0;
    const unsigned short* srcIdx = static_cast< const unsigned short* >( deus->getDataPointer() );
    while( deuiIdx < deus->size() )
        (*deui)[ deuiIdx++ ] = *srcIdx++;

    return( deui.release() );
}

osg::DrawElementsUInt* convertAllFilledToTriangles( const osg::DrawElementsUInt* deuiIn )
{
    if( ( deuiIn == NULL ) || ( deuiIn->size() < 3 ) )
        return( NULL );

    osg::ref_ptr< osg::DrawElementsUInt > deui = new osg::DrawElementsUInt;
    deui->setMode( GL_TRIANGLES );

    const unsigned int* dataPtr = static_cast< const unsigned int* >( deuiIn->getDataPointer() );
    unsigned int v0, v1, v2, v3;
    unsigned int indexCount( 0 );
    switch( deuiIn->getMode() )
    {
    case GL_TRIANGLE_STRIP:
    case GL_QUAD_STRIP:
        // tri strip and wuad strip are *almost* the same...
        v0 = *dataPtr++;
        v1 = *dataPtr++;
        indexCount += 2;
        while( indexCount + 2 <= deuiIn->size() )
        {
            v2 = *dataPtr++;
            v3 = *dataPtr++;
            indexCount += 2;
            deui->push_back( v0 );
            deui->push_back( v1 );
            deui->push_back( v2 );
            deui->push_back( v2 );
            deui->push_back( v1 );
            deui->push_back( v3 );
            v0 = v2;
            v1 = v3;
        }
        // If the vertex count is odd, proces last triangle
        // *only* if it's a tri strip. Ignore it if it's a quad strip.
        if( ( deuiIn->getMode() == GL_TRIANGLE_STRIP ) &&
            ( indexCount + 1 <= deuiIn->size() ) )
        {
            v2 = *dataPtr++;
            deui->push_back( v0 );
            deui->push_back( v1 );
            deui->push_back( v2 );
        }
        break;
    case GL_TRIANGLE_FAN:
    case GL_POLYGON:
        // polygon and tri fan are triangulated identically.
        v0 = *dataPtr++;
        v1 = *dataPtr++;
        indexCount += 2;
        while( indexCount < deuiIn->size() )
        {
            v2 = *dataPtr++;
            indexCount += 1;
            deui->push_back( v0 );
            deui->push_back( v1 );
            deui->push_back( v2 );
            v1 = v2;
        }
        break;
    case GL_QUADS:
        while( indexCount + 4 <= deuiIn->size() )
        {
            v0 = *dataPtr++;
            v1 = *dataPtr++;
            v2 = *dataPtr++;
            v3 = *dataPtr++;
            indexCount += 4;
            deui->push_back( v0 );
            deui->push_back( v1 );
            deui->push_back( v2 );
            deui->push_back( v2 );
            deui->push_back( v1 );
            deui->push_back( v3 );
        }
        break;

    default:
        // GL_POINTS, GL_LINES, GL_LINE_LOOP, or GL_LINE_STRIP.
        // Not an error, just ignore them.
    case GL_TRIANGLES:
        // Nothing to do. Just return ptr to input.
        return( const_cast< osg::DrawElementsUInt* >( deuiIn ) );
        break;
    }

    return( deui.release() );
}


// osgwTools
}
