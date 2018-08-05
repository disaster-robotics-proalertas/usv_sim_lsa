/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2010 by Kenneth Mark Bryden
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

#include <osgwTools/Shapes.h>
#include <osgwTools/Transform.h>
#include <osg/Geometry>

#include <math.h>
#include <cstring>
#include <osg/io_utils>



#ifdef WIN32
#  pragma warning( disable : 4305 )
#endif


/** Generates an array of xyz vertices that for a circle.

Note that calling code is responsible for ensuring the returned array is deleted,
typically by storing the returned array in sn osg::ref_ptr.

\param approx Number of vertices in the circle. For example, '4' would create a square
(vertex vectors at 90 degree angles to each other).
\param radius Circle radius.
\param plane Equation for the plane to hold the vertices. The circle will be centered on
the plane's origin.
\param close If true, the first vertex is repeated as the last vertex, and the total
number of vertices will be ( \c approx + 1 ).

Possible future change: Export this as an API routine. */
static osg::Vec3Array*
generateCircleVertices( unsigned int approx, double radius, osg::Vec4 plane, bool close=false )
{
    // Find ideal base vector (at 90 degree angle to normalVec)
    osg::Vec3 normalVec( plane[0], plane[1], plane[2] );
    normalVec.normalize();
    osg::Vec3 crossVec( 1., 0., 0. );
    if( osg::absolute( normalVec * crossVec ) > .9 )
        crossVec = osg::Vec3( 0., 1., 0. );
    osg::Vec3 baseVec = normalVec ^ crossVec;
    baseVec.normalize();
    baseVec = ( baseVec * radius ) + ( normalVec * plane[ 3 ] );

    osg::ref_ptr< osg::Vec3Array > verts = new osg::Vec3Array;
    unsigned int totalVerts = approx + ( close ? 1 : 0 );
    verts->resize( totalVerts );

    unsigned int idx;
	for( idx=0; idx < approx; idx++ )
    {
        const double angle( 2. * osg::PI * (double)idx / (double)approx );
        osg::Matrix m( osg::Matrix::rotate( angle, normalVec ) );
        ( *verts )[ idx ] = baseVec * m;
    }
    if( close )
        ( *verts )[ idx ] = ( *verts )[ 0 ];

    return( verts.release() );
}

inline unsigned int makeKey( const unsigned short a, const unsigned short b )
{
	if ( a < b )
		return( ( a << 16) | b );
	else
		return( ( b << 16) | a );
};
inline void average3fv( float* r, const float* a, const float* b )
{
	r[0] = (a[0] + b[0]) * .5f;
	r[1] = (a[1] + b[1]) * .5f;
	r[2] = (a[2] + b[2]) * .5f;
};

static bool
buildGeodesicSphereData( const float radius, const unsigned int subdivisions, osg::Geometry* geom )
{
    unsigned int subdivide( subdivisions );
    if( subdivisions > 5 )
    {
        // Would create index array too large for use with DrawElementsUShort.
        // For now, clamp. In the future, just use DrawElementsUInt.
        osg::notify( osg::WARN ) << "makeGeodesicSphere: Clamping subdivisions to 5." << std::endl;
        subdivide = 5;
    }

    GLfloat vertData[] = {
        0.000000, 0.850651, 0.525731,
        0.000000, 0.850651, -0.525731,
        0.000000, -0.850651, -0.525731,
        0.000000, -0.850651, 0.525731,
        0.525731, 0.000000, 0.850651,
        0.525731, 0.000000, -0.850651,
        -0.525731, 0.000000, -0.850651,
        -0.525731, 0.000000, 0.850651,
        0.850651, 0.525731, 0.000000,
        0.850651, -0.525731, 0.000000,
        -0.850651, -0.525731, 0.000000,
        -0.850651, 0.525731, 0.000000 };

    int faces = 20;
    int _numVerts = 12;
    int _numIndices = faces * 3;

    // Data is initially in "golden mean" coordinate system.
    // Rotate around y so that 2 verts exist at (0,0,+/-1).
    {
        //osg::Vec3 v0( 0.525731, 0.000000, 0.850651 );
        //osg::Vec3 v1( 0, 0, 1 );
        //const double angle( acos( v0 * v1 ) );
        const double sinAngle( 0.525731 );
        const double cosAngle( 0.850651 );

        int idx;
        for( idx=0; idx<_numVerts*3; idx+=3 )
        {
            double x( vertData[ idx ] );
            double z( vertData[ idx+2 ] );
            vertData[ idx ] = x * cosAngle + z * -sinAngle;
            vertData[ idx+2 ] = x * sinAngle + z * cosAngle;
        }
    }


    int vertsSize = _numVerts * 3;
    GLfloat* _vertices = new GLfloat[ vertsSize ];
    memcpy( _vertices, vertData, sizeof( vertData ) );

    GLushort* _indices = new GLushort[ _numIndices ];
    GLushort* indexPtr = _indices;
    *indexPtr++ = 0;
    *indexPtr++ = 7;
    *indexPtr++ = 4;
    *indexPtr++ = 0;
    *indexPtr++ = 4;
    *indexPtr++ = 8;
    *indexPtr++ = 0;
    *indexPtr++ = 8;
    *indexPtr++ = 1;
    *indexPtr++ = 0;
    *indexPtr++ = 1;
    *indexPtr++ = 11;
    *indexPtr++ = 0;
    *indexPtr++ = 11;
    *indexPtr++ = 7;
    *indexPtr++ = 2;
    *indexPtr++ = 6;
    *indexPtr++ = 5;
    *indexPtr++ = 2;
    *indexPtr++ = 5;
    *indexPtr++ = 9;
    *indexPtr++ = 2;
    *indexPtr++ = 9;
    *indexPtr++ = 3;
    *indexPtr++ = 2;
    *indexPtr++ = 3;
    *indexPtr++ = 10;
    *indexPtr++ = 2;
    *indexPtr++ = 10;
    *indexPtr++ = 6;
    *indexPtr++ = 7;
    *indexPtr++ = 3;
    *indexPtr++ = 4;
    *indexPtr++ = 4;
    *indexPtr++ = 3;
    *indexPtr++ = 9;
    *indexPtr++ = 4;
    *indexPtr++ = 9;
    *indexPtr++ = 8;
    *indexPtr++ = 8;
    *indexPtr++ = 9;
    *indexPtr++ = 5;
    *indexPtr++ = 8;
    *indexPtr++ = 5;
    *indexPtr++ = 1;
    *indexPtr++ = 1;
    *indexPtr++ = 5;
    *indexPtr++ = 6;
    *indexPtr++ = 1;
    *indexPtr++ = 6;
    *indexPtr++ = 11;
    *indexPtr++ = 11;
    *indexPtr++ = 6;
    *indexPtr++ = 10;
    *indexPtr++ = 11;
    *indexPtr++ = 10;
    *indexPtr++ = 7;
    *indexPtr++ = 7;
    *indexPtr++ = 10;
    *indexPtr++ = 3;

    GLuint _idxEnd = 11;


    // Subdivide as requested
    int idx;
    for (idx = subdivide; idx; idx--)
    {
        // Make a map of edges
        typedef std::map< unsigned int, GLushort> EdgeMapType;
        EdgeMapType edgeMap;
        indexPtr = _indices;
        int f;
        for (f=faces; f; f--)
        {
            unsigned int key = makeKey(indexPtr[0], indexPtr[1]);
            if (edgeMap.find( key ) == edgeMap.end())
                edgeMap[key] = ++_idxEnd;

            key = makeKey(indexPtr[1], indexPtr[2]);
            if (edgeMap.find( key ) == edgeMap.end())
                edgeMap[key] = ++_idxEnd;
            
            key = makeKey(indexPtr[2], indexPtr[0]);
            if (edgeMap.find( key ) == edgeMap.end())
                edgeMap[key] = ++_idxEnd;

            indexPtr += 3;
        }

        GLfloat* oldVerts = _vertices;
        GLushort* oldIndices = _indices;

        _numVerts += (int)(faces * 1.5f);
        int newFaces = faces * 4;
        _numIndices = newFaces * 3;

        // Create new indices
        _indices = new GLushort[ _numIndices ];
        GLushort* oldIdxPtr = oldIndices;
        indexPtr = _indices;
        for (f=faces; f; f--)
        {
            GLushort vertA = *oldIdxPtr++;
            GLushort vertB = *oldIdxPtr++;
            GLushort vertC = *oldIdxPtr++;
            GLushort edgeAB = edgeMap[ makeKey(vertA,vertB) ];
            GLushort edgeBC = edgeMap[ makeKey(vertB,vertC) ];
            GLushort edgeCA = edgeMap[ makeKey(vertC,vertA) ];

            *indexPtr++ = vertA;
            *indexPtr++ = edgeAB;
            *indexPtr++ = edgeCA;
            *indexPtr++ = edgeAB;
            *indexPtr++ = vertB;
            *indexPtr++ = edgeBC;
            *indexPtr++ = edgeAB;
            *indexPtr++ = edgeBC;
            *indexPtr++ = edgeCA;
            *indexPtr++ = edgeCA;
            *indexPtr++ = edgeBC;
            *indexPtr++ = vertC;
        }

        // Copy old vertices into new vertices
        _vertices = new GLfloat[ _numVerts * 3 ];
        memcpy( _vertices, oldVerts, vertsSize * sizeof( GLfloat ) );

        // Create new vertices at midpoint of each edge
        EdgeMapType::const_iterator it = edgeMap.begin();
        while (it != edgeMap.end())
        {
            GLushort idxA, idxB;
            idxA = ((*it).first) >> 16;
            idxB = ((*it).first) & 0xffff;

            GLfloat* dest = &(_vertices[ ((*it).second * 3) ]);
            GLfloat* srcA = &(_vertices[idxA*3]);
            GLfloat* srcB = &(_vertices[idxB*3]);
            average3fv( dest, srcA, srcB );

            it++;
        }


        faces = newFaces;
        vertsSize = _numVerts * 3;
        delete[] oldVerts;
        delete[] oldIndices;
    }


    //
    // Create normal array by making vertices unit length
    GLfloat* _normals = new GLfloat[ _numVerts * 3 ];
    GLfloat* vertPtr = _vertices;
    GLfloat* normPtr = _normals;
    for (idx = _numVerts; idx; idx--)
    {
        osg::Vec3 v( vertPtr[0], vertPtr[1], vertPtr[2] );
        float lengthInv = (float)( 1. / v.length() );
        *normPtr++ = *vertPtr++ * lengthInv;
        *normPtr++ = *vertPtr++ * lengthInv;
        *normPtr++ = *vertPtr++ * lengthInv;
    }

    //
    // Scale vertices out to the specified radius
    vertPtr = _vertices;
    normPtr = _normals;
    for (idx = _numVerts*3; idx; idx--)
        *vertPtr++ = *normPtr++ * radius;

    //
    // Texture coordinates are identical to normals for cube mapping
    GLfloat* _texCoords = new GLfloat[ _numVerts * 3 ];
    memcpy( _texCoords, _normals, _numVerts * 3 * sizeof( GLfloat) );


    // Convert to OSG
    {
        osg::Vec3Array* osgV = new osg::Vec3Array;
        osg::Vec3Array* osgN = new osg::Vec3Array;
        osg::Vec3Array* osgTC = new osg::Vec3Array;
        osgV->resize( _numVerts );
        osgN->resize( _numVerts );
        osgTC->resize( _numVerts );

        geom->setVertexArray( osgV );
        geom->setNormalArray( osgN );
        geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
        geom->setTexCoordArray( 0, osgTC );

        osg::Vec4Array* osgC = new osg::Vec4Array;
        osgC->push_back( osg::Vec4( 1., 1., 1., 1. ) );
        geom->setColorArray( osgC );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        vertPtr = _vertices;
        normPtr = _normals;
        GLfloat* tcPtr = _texCoords;
        int idx;
        for( idx=0; idx<_numVerts; idx++ )
        {
            (*osgV)[ idx ].x() = *vertPtr++;
            (*osgV)[ idx ].y() = *vertPtr++;
            (*osgV)[ idx ].z() = *vertPtr++;
            (*osgN)[ idx ].x() = *normPtr++;
            (*osgN)[ idx ].y() = *normPtr++;
            (*osgN)[ idx ].z() = *normPtr++;
            (*osgTC)[ idx ].x() = *tcPtr++;
            (*osgTC)[ idx ].y() = *tcPtr++;
            (*osgTC)[ idx ].z() = *tcPtr++;
        }

        osg::UShortArray* osgIdx = new osg::UShortArray;
        osgIdx->resize( _numIndices );
        indexPtr = _indices;
        for( idx=0; idx<_numIndices; idx++ )
        {
            (*osgIdx)[ idx ] = *indexPtr++;
        }

        geom->addPrimitiveSet( new osg::DrawElementsUShort( GL_TRIANGLES, _numIndices, _indices ) );
    }


    delete[] _indices;
    delete[] _vertices;
    delete[] _normals;
    delete[] _texCoords;

    osg::notify( osg::INFO ) << "makeGeodesicSphere: numVertices: " << _numVerts << std::endl;
    return( true );
}

osg::Geometry*
osgwTools::makeGeodesicSphere( const float radius, const unsigned int subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildGeodesicSphereData( radius, subdivisions, geom.get() );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeGeodesicSphere: Error during sphere build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}

osg::Geometry*
osgwTools::makeGeodesicSphere( const osg::Matrix& m, const float radius, const unsigned int subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeGeodesicSphere( radius, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}




static bool
buildAltAzSphereData( const float radius, const unsigned int subLat, const unsigned int subLong, osg::Geometry* geom, const bool wire )
{
    unsigned int numLat( subLat );
    unsigned int numLong( subLong );
    if( numLat < 2 )
        numLat = 2;
    if( numLong < 4 )
        numLong = 4;
    unsigned int totalVerts( (numLat+1) * (numLong+1) );
    if( totalVerts > 65535 )
    {
        // Would create index array too large for use with DrawElementsUShort.
        // For now, clamp. In the future, just use DrawElementsUInt.
        osg::notify( osg::WARN ) << "makeAltAzSphere: Clamping subdivisions to 128x256." << std::endl;
        numLat = 128;
        numLong = 256;
        totalVerts = ( (numLat+1) * (numLong+1) );
    }
    osg::notify( osg::INFO ) << "makeAltAzSphere: totalVerts: " << totalVerts << std::endl;

    // Create data arrays and configure the Geometry
    osg::ref_ptr< osg::Vec3Array > vertices( new osg::Vec3Array );
    vertices->resize( totalVerts );
    geom->setVertexArray( vertices.get() );

    osg::ref_ptr< osg::Vec3Array > normals;
    osg::ref_ptr< osg::Vec2Array > texCoords;
    if( !wire )
    {
        normals = new osg::Vec3Array;
        normals->resize( totalVerts );
        geom->setNormalArray( normals.get() );
        geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

        texCoords = new osg::Vec2Array;
        texCoords->resize( totalVerts );
        geom->setTexCoordArray( 0, texCoords.get() );
    }

    {
        osg::Vec4Array* osgC = new osg::Vec4Array;
        osgC->push_back( osg::Vec4( 1., 1., 1., 1. ) );
        geom->setColorArray( osgC );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );
    }

    // Create the vertices, normals, and tex coords.
    unsigned int idx( 0 );
    unsigned int latCounter;
    for( latCounter=numLat+1; latCounter>0; latCounter-- )
    {
        const double t( (double)(latCounter-1) / (double)numLat );
        const double latAngle( (t-0.5) * osg::PI ); // latAngle is in range (-pi/2,pi/2) radians.
        const osg::Vec3 baseVec( cos( latAngle ), 0., sin( latAngle ) );

        unsigned int longCounter;
        for( longCounter=0; longCounter<numLong; longCounter++ )
        {
            const double s( (double)longCounter / (double)numLong );
            const double longAngle( s * 2.0 * osg::PI ); // longAngle is in range (0,2pi) radians.
            const double sinAngle( sin( longAngle ) );
            const double cosAngle( cos( longAngle ) );

            osg::Vec3 v;
            v[ 0 ] = baseVec.x() * cosAngle + baseVec.y() * -sinAngle;
            v[ 1 ] = baseVec.x() * sinAngle + baseVec.y() * cosAngle;
            v[ 2 ] = baseVec.z();

            (*vertices)[ idx ] = ( v * radius );
            //osg::notify( osg::ALWAYS ) << v << std::endl;

            if( !wire )
            {
                (*normals)[ idx ] = v;
                (*texCoords)[ idx ].set( s, t );
            }

            idx++;
        }
        {
            // Close (required for texture mapping)
            osg::Vec3 v( baseVec );

            (*vertices)[ idx ] = ( v * radius );
            //osg::notify( osg::ALWAYS ) << v << std::endl;

            if( !wire )
            {
                (*normals)[ idx ] = v;
                (*texCoords)[ idx ].set( 1., t );
            }

            idx++;
        }
    }

    if( idx != totalVerts )
    {
        osg::notify( osg::WARN ) << "AltAzSphere: Error creating vertices." << std::endl;
        osg::notify( osg::WARN ) << "  idx " << idx << " != totalVerts " << totalVerts << std::endl;
    }


    // Create PrimitiveSets.

    if( !wire )
    {
        // Solid -- Use GL_TRIANGLE_STRIPS

        // Create indices -- top group of triangles
        osg::DrawElementsUShort* fan( new osg::DrawElementsUShort( GL_TRIANGLES ) );
        fan->resize( numLong*3 );
        for( idx=0; idx<numLong; idx++ )
        {
            (*fan)[ idx*3 ] = idx;
            (*fan)[ idx*3+1 ] = numLong + idx + 1;
            (*fan)[ idx*3+2 ] = numLong + idx + 2;
        }
        geom->addPrimitiveSet( fan );

        // Create indices -- body
        osg::DrawElementsUShort* body;
        unsigned int baseIdx( numLong+1 );
        unsigned int stripIdx;
        for( stripIdx=0; stripIdx<numLat-2; stripIdx++ )
        {
            body = new osg::DrawElementsUShort( GL_TRIANGLE_STRIP );
            body->resize( (numLong+1) * 2 );

            unsigned int longCounter;
            for( longCounter=0; longCounter<numLong; longCounter++ )
            {
                (*body)[ longCounter*2 ] = baseIdx;
                (*body)[ longCounter*2+1 ] = baseIdx + numLong+1;
                baseIdx++;
            }
            // Close strip
            (*body)[ longCounter*2 ] = baseIdx;
            (*body)[ longCounter*2+1 ] = baseIdx + numLong+1;
            baseIdx++;

            geom->addPrimitiveSet( body );
        }

        // Create indices -- bottom group of triangles
        fan = new osg::DrawElementsUShort( GL_TRIANGLES );
        fan->resize( numLong*3 );
        for( idx=0; idx<numLong; idx++ )
        {
            // 14 9 8, 13 8 7, 12 7 6, 11 6 5
            (*fan)[ idx*3 ] = totalVerts - 1 - idx;
            (*fan)[ idx*3+1 ] = totalVerts - 1 - numLong - idx - 1;
            (*fan)[ idx*3+2 ] = totalVerts - 1 - numLong - idx - 2;
        }
        geom->addPrimitiveSet( fan );
    }
    else
    {
        // Wire -- Use GL_LINE_LOOP and GL_LINE_STRIP

        // Create indices -- alt (latitude)
        osg::DrawElementsUShort* deus;
        unsigned int baseIdx( numLong+1 );
        unsigned int loopIdx;
        for( loopIdx=0; loopIdx<numLat-1; loopIdx++ )
        {
            deus = new osg::DrawElementsUShort( GL_LINE_LOOP );
            deus->resize( numLong );

            unsigned int longCounter;
            for( longCounter=0; longCounter<numLong; longCounter++ )
            {
                (*deus)[ longCounter ] = baseIdx++;
            }
            // Skip closing vertex.
            baseIdx++;
            geom->addPrimitiveSet( deus );
        }

        // Create indices -- az (longitude)
        const unsigned int vertsPerLat( numLong+1 );
        unsigned int longCounter;
        for( longCounter=0; longCounter<numLong; longCounter++ )
        {
            deus = new osg::DrawElementsUShort( GL_LINE_STRIP );
            deus->resize( numLat+1 );

            unsigned int latIdx;
            for( latIdx=0; latIdx<numLat+1; latIdx++ )
            {
                (*deus)[ latIdx ] = longCounter + (vertsPerLat * latIdx);
            }
            geom->addPrimitiveSet( deus );
        }
    }

    return( true );
}


static bool
buildCircleData( float radius, const unsigned int subdivisions, const osg::Vec4& plane, osg::Geometry* geom, const bool wire )
{
    unsigned int numSub( subdivisions );
    unsigned int totalVerts( 0 );
    if( numSub < 3 )
        numSub = 3;
    if( numSub > 65530 )
    {
        // Would create index array too large for use with DrawElementsUShort. Clamp.
		numSub = 65530; // leave headroom for a few spares
        osg::notify( osg::WARN ) << "buildCircleData: Clamping subdivisions to " << numSub << std::endl;
    }
    totalVerts = numSub;
    if( !wire )
        totalVerts += 2; // Closing and center vertices.
    osg::notify( osg::INFO ) << "buildCircleData: totalVerts: " << totalVerts << std::endl;

    // Create data arrays and configure the Geometry
    osg::ref_ptr< osg::Vec3Array > vertices;
    if( geom->getVertexArray() != NULL )
    {
        vertices = dynamic_cast< osg::Vec3Array* >( geom->getVertexArray() );
        if( !( vertices.valid() ) )
            return( false );
    }
    else
    {
        vertices = new osg::Vec3Array;
        geom->setVertexArray( vertices.get() );
    }
    vertices->reserve( totalVerts );

    osg::ref_ptr< osg::Vec3Array > normals;
    osg::ref_ptr< osg::Vec2Array > texCoords;
    if( !wire )
    {
        if( geom->getNormalArray() != NULL )
        {
            normals = dynamic_cast< osg::Vec3Array* >( geom->getNormalArray() );
            if( !( normals.valid() ) )
                return( false );
        }
        else
        {
            normals = new osg::Vec3Array;
            geom->setNormalArray( normals.get() );
        }
        normals->resize( normals->size() + totalVerts );
        geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

        if( geom->getTexCoordArray( 0 ) != NULL )
        {
            texCoords = dynamic_cast< osg::Vec2Array* >( geom->getTexCoordArray( 0 ) );
            if( !( texCoords.valid() ) )
                return( false );
        }
        else
        {
            texCoords = new osg::Vec2Array;
            geom->setTexCoordArray( 0, texCoords.get() );
        }
        texCoords->resize( texCoords->size() + totalVerts );
    }

    osg::Vec4Array* osgC = new osg::Vec4Array;
    osgC->push_back( osg::Vec4( 1., 1., 1., 1. ) );
    geom->setColorArray( osgC );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );


    unsigned int oldSize = vertices->size();
    osg::Vec3 normalVec( plane[0], plane[1], plane[2] );
    normalVec.normalize();
    {
        osg::ref_ptr< osg::Vec3Array > circle = generateCircleVertices( numSub, radius, plane );
        vertices->insert( vertices->end(), circle->begin(), circle->end() );
    }
    unsigned int newSize = vertices->size();
    if( newSize - oldSize != numSub )
        osg::notify( osg::WARN ) << "buildCircleData: Problem generating circle vertices." << std::endl;
    if( !wire )
    {
        vertices->resize( vertices->size() + 2 );
        (*vertices)[ newSize ] = (*vertices)[ oldSize ]; // Repeat first vert as closing vert.
        (*vertices)[ newSize+1 ] = osg::Vec3( plane[0], plane[1], plane[2] ) * plane[3];
        newSize += 2;
    }

    // Create the normals and tex coords.
    unsigned int idx( oldSize );
    unsigned int subCounter;
    if( !wire )
    {
	    for( subCounter=0; subCounter < totalVerts; subCounter++, idx++ )
        {
            (*normals)[ idx ] = normalVec;

            const osg::Vec3& v( (*vertices)[ idx ] );
			const osg::Vec3 vRad( v / radius );
            (*texCoords)[ idx ].set( vRad.x(), vRad.y() );
        } // for
    }


    // Create PrimitiveSets.

    if( !wire )
    {
        // Solid -- Use GL_TRIANGLE_FAN

        // Create indices -- top group of triangles
        osg::DrawElementsUShort* fan( new osg::DrawElementsUShort( GL_TRIANGLE_FAN ) );
        fan->resize( totalVerts );

		// push center
        (*fan)[ 0 ] = newSize - 1;

		// circle loop
        unsigned int idx, loopIdx;
		for( loopIdx=0, idx=oldSize; loopIdx < numSub+1; loopIdx++, idx++ )
        {
            (*fan)[ loopIdx+1 ] = idx;
        }
        geom->addPrimitiveSet( fan );
    } // if !wire
    else
    {
        // Wire -- Use GL_LINE_LOOP

        // Create indices
        osg::DrawElementsUShort* ring = new osg::DrawElementsUShort( GL_LINE_LOOP );
        ring->resize( numSub );
        unsigned int idx, loopIdx;
        for( loopIdx=0, idx=oldSize; loopIdx < numSub; loopIdx++, idx++ )
        {
            (*ring)[ loopIdx ] = idx;
        }
		geom->addPrimitiveSet( ring );
    } // wire

    return( true );
} // buildCircleData

osg::Geometry*
osgwTools::makeAltAzSphere( const float radius, const unsigned int subLat, const unsigned int subLong, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildAltAzSphereData( radius, subLat, subLong, geom.get(), false );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeAltAzSphere: Error during sphere build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}

osg::Geometry*
osgwTools::makeAltAzSphere( const osg::Matrix& m, const float radius, const unsigned int subLat, const unsigned int subLong, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeAltAzSphere( radius, subLat, subLong, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}


osg::Geometry*
osgwTools::makeWireAltAzSphere( const float radius, const unsigned int subLat, const unsigned int subLong, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildAltAzSphereData( radius, subLat, subLong, geom.get(), true );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeWireAltAzSphere: Error during sphere build." << std::endl;
        return( NULL );
    }
    else
    {
        // Disable lighting and texture mapping for wire primitives.
        osg::StateSet* stateSet = geom->getOrCreateStateSet();
        stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
        stateSet->setTextureMode( 0, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

        return( geom.release() );
    }
}

osg::Geometry*
osgwTools::makeWireAltAzSphere( const osg::Matrix& m, const float radius, const unsigned int subLat, const unsigned int subLong, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeWireAltAzSphere( radius, subLat, subLong, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}



osg::Geometry* osgwTools::makeCircle( const osg::Vec4& plane, const float radius, const unsigned int subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildCircleData( radius, subdivisions, plane, geom.get(), false );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeCircle: Error during circle build." << std::endl;
        return( NULL );
    } // if
    else
    {
        return( geom.release() );
    } // else
}

osg::Geometry* osgwTools::makeCircle( const osg::Matrix& m, const osg::Vec4& plane, const float radius, const unsigned int subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeCircle( plane, radius, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}

osg::Geometry* osgwTools::makeWireCircle( const osg::Vec4& plane, const float radius, const unsigned int subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildCircleData( radius, subdivisions, plane, geom.get(), true );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeWireCircle: Error during circle build." << std::endl;
        return( NULL );
    } // if
    else
    {
        // Disable lighting and texture mapping for wire primitives.
        osg::StateSet* stateSet = geom->getOrCreateStateSet();
        stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
        stateSet->setTextureMode( 0, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

        return( geom.release() );
    } // else
}

osg::Geometry* osgwTools::makeWireCircle( const osg::Matrix& m, const osg::Vec4& plane, const float radius, const unsigned int subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeWireCircle( plane, radius, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}


osg::Geometry*
osgwTools::makeCircle( const float radius, const unsigned int subdivisions, const osg::Vec3& orientation, osg::Geometry* geometry)
{
    const osg::Vec4 plane( orientation[0], orientation[1], orientation[2], 0. );
    return( makeCircle( plane, radius, subdivisions, geometry ) );
}

osg::Geometry*
osgwTools::makeCircle( const osg::Matrix& m, const float radius, const unsigned int subdivisions, const osg::Vec3& orientation, osg::Geometry* geometry)
{
    const osg::Vec4 plane( orientation[0], orientation[1], orientation[2], 0. );
    return( makeCircle( m, plane, radius, subdivisions, geometry ) );
}

osg::Geometry*
osgwTools::makeWireCircle( const float radius, const unsigned int subdivisions, const osg::Vec3& orientation, osg::Geometry* geometry)
{
    const osg::Vec4 plane( orientation[0], orientation[1], orientation[2], 0. );
    return( makeWireCircle( plane, radius, subdivisions, geometry ) );
}

osg::Geometry*
osgwTools::makeWireCircle( const osg::Matrix& m, const float radius, const unsigned int subdivisions, const osg::Vec3& orientation, osg::Geometry* geometry)
{
    const osg::Vec4 plane( orientation[0], orientation[1], orientation[2], 0. );
    return( makeWireCircle( m, plane, radius, subdivisions, geometry ) );
}



static void
addPlaneData( const osg::Vec3& corner,
             const osg::Vec3& u, unsigned short uSteps,
             const osg::Vec3& v, unsigned short vSteps,
             const osg::Vec3& normal, osg::Geometry* geom )
{
    osg::Vec3Array* vert( static_cast< osg::Vec3Array* >( geom->getVertexArray() ) );
    osg::Vec3Array* norm( static_cast< osg::Vec3Array* >( geom->getNormalArray() ) );
    osg::Vec2Array* texc( static_cast< osg::Vec2Array* >( geom->getTexCoordArray( 0 ) ) );

    unsigned short uIdx, vIdx;
    for( vIdx=0; vIdx<=vSteps; vIdx++ )
    {
        const float vPct( (float)vIdx / (float)vSteps );
        const osg::Vec3 vVec( v * vPct );

        osg::ref_ptr< osg::DrawElementsUInt > deui;
        if( vIdx < vSteps )
            deui = new osg::DrawElementsUInt( GL_TRIANGLE_STRIP );

        unsigned int startIdx( vert->size() ), idx( 0 );
        for( uIdx=0; uIdx<=uSteps; uIdx++ )
        {
            const float uPct( (float)uIdx / (float)uSteps );
            osg::Vec3 vertex( corner + vVec + (u * uPct) );

            vert->push_back( vertex );
            norm->push_back( normal );
            texc->push_back( osg::Vec2( uPct, vPct ) );
            if( deui.valid() )
            {
                deui->push_back( startIdx + idx + uSteps + 1 );
                deui->push_back( startIdx + idx );
                idx++;
            }
        }
        if( deui.valid() )
            geom->addPrimitiveSet( deui.get() );
    }
}

static bool
buildPlaneData( const osg::Vec3& corner, const osg::Vec3& u, const osg::Vec3& v, const osg::Vec2s& subdivisions, osg::Geometry* geom )
{
    if( ( subdivisions.x() <= 0. ) || ( subdivisions.y() <= 0. ) )
    {
        osg::notify( osg::WARN ) << "osgwTools: makePlane: Invalid subdivisions." << std::endl;
        return( false );
    }
    const unsigned short subX( (unsigned short)( subdivisions.x() ) );
    const unsigned short subY( (unsigned short)( subdivisions.y() ) );

    geom->setVertexArray( new osg::Vec3Array );
    geom->setNormalArray( new osg::Vec3Array );
    geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    geom->setTexCoordArray( 0, new osg::Vec2Array );
    {
        osg::Vec4Array* osgC = new osg::Vec4Array;
        osgC->push_back( osg::Vec4( 1., 1., 1., 1. ) );
        geom->setColorArray( osgC );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );
    }

    osg::Vec3 normal( u ^ v );
    normal.normalize();
    addPlaneData( corner, u, subX, v, subY,
        normal, geom );

    return( true );
}

osg::Geometry*
osgwTools::makePlane( const osg::Vec3& corner, const osg::Vec3& u, const osg::Vec3& v, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildPlaneData( corner, u, v, subdivisions, geom.get() );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makePlane: Error during plane build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}



static bool
buildWirePlaneData( const osg::Vec3& corner, const osg::Vec3& u, const osg::Vec3& v, const osg::Vec2s& subdivisions, osg::Geometry* geom )
{
    osg::Vec3Array* verts = new osg::Vec3Array;

    osg::Vec3 end( corner + v );
    int idx;
    for( idx=0; idx <= subdivisions.x(); idx++ )
    {
        const float percent( (float)idx / (float)(subdivisions.x()) );
        const osg::Vec3 strut( u * percent );
        verts->push_back( corner+strut );
        verts->push_back( end+strut );
    }
    end.set( corner + u );
    for( idx=0; idx <= subdivisions.y(); idx++ )
    {
        const float percent( (float)idx / (float)(subdivisions.y()) );
        const osg::Vec3 strut( v * percent );
        verts->push_back( corner+strut );
        verts->push_back( end+strut );
    }

    osg::Vec4Array* c = new osg::Vec4Array;
    c->push_back( osg::Vec4( 1., 1., 1., 1. ) );

    geom->setVertexArray( verts );
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    geom->addPrimitiveSet( new osg::DrawArrays( GL_LINES, 0, verts->getNumElements() ) );

    // Disable lighting and texture mapping for wire primitives.
    osg::StateSet* stateSet = geom->getOrCreateStateSet();
    stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    stateSet->setTextureMode( 0, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

    return( true );
}

osg::Geometry*
osgwTools::makeWirePlane( const osg::Vec3& corner, const osg::Vec3& u, const osg::Vec3& v, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildWirePlaneData( corner, u, v, subdivisions, geom.get() );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeWirePlane: Error during plane build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}




static bool
buildBoxData( const osg::Vec3& halfExtents, const osg::Vec3s& subdivisions, osg::Geometry* geom )
{
    if( ( subdivisions.x() <= 0. ) || ( subdivisions.y() <= 0. ) || ( subdivisions.z() <= 0. ) )
    {
        osg::notify( osg::WARN ) << "osgwTools: makeBox: Invalid subdivisions." << std::endl;
        return( false );
    }
    const unsigned short subX( (unsigned short)( subdivisions.x() ) );
    const unsigned short subY( (unsigned short)( subdivisions.y() ) );
    const unsigned short subZ( (unsigned short)( subdivisions.z() ) );

    const float xMin( -halfExtents[ 0 ] );
    const float xMax( halfExtents[ 0 ] );
    const float yMin( -halfExtents[ 1 ] );
    const float yMax( halfExtents[ 1 ] );
    const float zMin( -halfExtents[ 2 ] );
    const float zMax( halfExtents[ 2 ] );

    geom->setVertexArray( new osg::Vec3Array );
    geom->setNormalArray( new osg::Vec3Array );
    geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    geom->setTexCoordArray( 0, new osg::Vec2Array );
    {
        osg::Vec4Array* osgC = new osg::Vec4Array;
        osgC->push_back( osg::Vec4( 1., 1., 1., 1. ) );
        geom->setColorArray( osgC );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );
    }

    // +x
    addPlaneData( osg::Vec3( xMax, yMin, zMin ),
        osg::Vec3( 0., yMax-yMin, 0. ), subY,
        osg::Vec3( 0., 0., zMax-zMin ), subZ,
        osg::Vec3( 1., 0., 0. ), geom );

    // -x
    addPlaneData( osg::Vec3( xMin, yMax, zMin ),
        osg::Vec3( 0., yMin-yMax, 0. ), subY,
        osg::Vec3( 0., 0., zMax-zMin ), subZ,
        osg::Vec3( -1., 0., 0. ), geom );

    // +y
    addPlaneData( osg::Vec3( xMax, yMax, zMin ),
        osg::Vec3( xMin-xMax, 0., 0. ), subX,
        osg::Vec3( 0., 0., zMax-zMin ), subZ,
        osg::Vec3( 0., 1., 0. ), geom );

    // -y
    addPlaneData( osg::Vec3( xMin, yMin, zMin ),
        osg::Vec3( xMax-xMin, 0., 0. ), subX,
        osg::Vec3( 0., 0., zMax-zMin ), subZ,
        osg::Vec3( 0., -1., 0. ), geom );

    // +z
    addPlaneData( osg::Vec3( xMin, yMin, zMax ),
        osg::Vec3( xMax-xMin, 0., 0. ), subX,
        osg::Vec3( 0., yMax-yMin, 0. ), subY,
        osg::Vec3( 0., 0., 1. ), geom );

    // -z
    addPlaneData( osg::Vec3( xMax, yMin, zMin ),
        osg::Vec3( xMin-xMax, 0., 0. ), subX,
        osg::Vec3( 0., yMax-yMin, 0. ), subY,
        osg::Vec3( 0., 0., -1. ), geom );

    return( true );
}

static bool
buildPlainBoxData( const osg::Vec3& halfExtents, osg::Geometry* geom )
{
    const float xMin( -halfExtents[ 0 ] );
    const float xMax( halfExtents[ 0 ] );
    const float yMin( -halfExtents[ 1 ] );
    const float yMax( halfExtents[ 1 ] );
    const float zMin( -halfExtents[ 2 ] );
    const float zMax( halfExtents[ 2 ] );

    geom->setNormalBinding( osg::Geometry::BIND_OFF );
    geom->setColorBinding( osg::Geometry::BIND_OFF );

    osg::Vec3Array* v = new osg::Vec3Array;
    geom->setVertexArray( v );
    v->push_back( osg::Vec3( xMin, yMax, zMin ) );
    v->push_back( osg::Vec3( xMax, yMax, zMin ) );
    v->push_back( osg::Vec3( xMin, yMin, zMin ) );
    v->push_back( osg::Vec3( xMax, yMin, zMin ) );
    v->push_back( osg::Vec3( xMin, yMin, zMax ) );
    v->push_back( osg::Vec3( xMax, yMin, zMax ) );
    v->push_back( osg::Vec3( xMin, yMax, zMax ) );
    v->push_back( osg::Vec3( xMax, yMax, zMax ) );

    osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLE_STRIP );
    for( unsigned int idx = 0; idx<8; idx++ )
        deui->push_back( idx );
    deui->push_back( 0 );
    deui->push_back( 1 );
    geom->addPrimitiveSet( deui );

    deui = new osg::DrawElementsUInt( GL_TRIANGLE_STRIP );
    deui->push_back( 0 );
    deui->push_back( 2 );
    deui->push_back( 6 );
    deui->push_back( 4 );
    geom->addPrimitiveSet( deui );

    deui = new osg::DrawElementsUInt( GL_TRIANGLE_STRIP );
    deui->push_back( 1 );
    deui->push_back( 7 );
    deui->push_back( 3 );
    deui->push_back( 5 );
    geom->addPrimitiveSet( deui );

    return( true );
}

osg::Geometry*
osgwTools::makeBox( const osg::Vec3& halfExtents, const osg::Vec3s& subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildBoxData( halfExtents, subdivisions, geom.get() );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeBox: Error during box build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}

osg::Geometry*
osgwTools::makeBox( const osg::Matrix& m, const osg::Vec3& halfExtents, const osg::Vec3s& subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeBox( halfExtents, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}

osg::Geometry*
osgwTools::makePlainBox( const osg::Vec3& halfExtents, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildPlainBoxData( halfExtents, geom.get() );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeBox: Error during box build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}

osg::Geometry*
osgwTools::makePlainBox( const osg::Matrix& m, const osg::Vec3& halfExtents, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makePlainBox( halfExtents, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}




static bool
buildWireBoxData( const osg::Vec3& halfExtents, osg::Geometry* geom )
{
    osg::Vec4Array* color( new osg::Vec4Array );
    color->push_back( osg::Vec4( 1., 1., 1., 1. ) );
    geom->setColorArray( color );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    osg::Vec3Array* verts( new osg::Vec3Array );
    geom->setVertexArray( verts );

    verts->push_back( osg::Vec3( -halfExtents[ 0 ], -halfExtents[ 1 ], -halfExtents[ 2 ] ) );
    verts->push_back( osg::Vec3( halfExtents[ 0 ], -halfExtents[ 1 ], -halfExtents[ 2 ] ) );
    verts->push_back( osg::Vec3( halfExtents[ 0 ], halfExtents[ 1 ], -halfExtents[ 2 ] ) );
    verts->push_back( osg::Vec3( -halfExtents[ 0 ], halfExtents[ 1 ], -halfExtents[ 2 ] ) );
    verts->push_back( osg::Vec3( -halfExtents[ 0 ], -halfExtents[ 1 ], halfExtents[ 2 ] ) );
    verts->push_back( osg::Vec3( halfExtents[ 0 ], -halfExtents[ 1 ], halfExtents[ 2 ] ) );
    verts->push_back( osg::Vec3( halfExtents[ 0 ], halfExtents[ 1 ], halfExtents[ 2 ] ) );
    verts->push_back( osg::Vec3( -halfExtents[ 0 ], halfExtents[ 1 ], halfExtents[ 2 ] ) );

    osg::ref_ptr< osg::DrawElementsUInt > deui(
        new osg::DrawElementsUInt( GL_LINE_LOOP ) );
    deui->push_back( 0 );
    deui->push_back( 1 );
    deui->push_back( 2 );
    deui->push_back( 3 );
    geom->addPrimitiveSet( deui.get() );

    deui = new osg::DrawElementsUInt( GL_LINE_LOOP );
    deui->push_back( 4 );
    deui->push_back( 5 );
    deui->push_back( 6 );
    deui->push_back( 7 );
    geom->addPrimitiveSet( deui.get() );

    deui = new osg::DrawElementsUInt( GL_LINES );
    deui->push_back( 0 );
    deui->push_back( 4 );
    deui->push_back( 1 );
    deui->push_back( 5 );
    deui->push_back( 2 );
    deui->push_back( 6 );
    deui->push_back( 3 );
    deui->push_back( 7 );
    geom->addPrimitiveSet( deui.get() );

    return( true );
}

osg::Geometry*
osgwTools::makeWireBox( const osg::Vec3& halfExtents, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildWireBoxData( halfExtents, geom.get() );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeWireBox: Error during box build." << std::endl;
        return( NULL );
    }
    else
    {
        // Disable lighting and texture mapping for wire primitives.
        osg::StateSet* stateSet = geom->getOrCreateStateSet();
        stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
        stateSet->setTextureMode( 0, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

        return( geom.release() );
    }
}

osg::Geometry*
osgwTools::makeWireBox( const osg::Matrix& m, const osg::Vec3& halfExtents, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeWireBox( halfExtents, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}




static bool
buildArrowData( osg::Geometry* geom )
{
    // Create an arrow pointing in the +z direction.
    const float sD( .05 ); // shaft diameter
    const float hD( .075 ); // head diameter
    const float len( 1. ); // length
    const float sh( .65 ); // length from base to start of head

    osg::Vec3Array* verts( new osg::Vec3Array );
    verts->resize( 22 );
    geom->setVertexArray( verts );

    osg::Vec3Array* norms( new osg::Vec3Array );
    norms->resize( 22 );
    geom->setNormalArray( norms );
    geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    {
        osg::Vec4Array* osgC = new osg::Vec4Array;
        osgC->push_back( osg::Vec4( 1., 1., 1., 1. ) );
        geom->setColorArray( osgC );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );
    }

    // Shaft
    (*verts)[ 0 ] = osg::Vec3( sD, 0., 0. );
    (*verts)[ 1 ] = osg::Vec3( sD, 0., sh );
    (*verts)[ 2 ] = osg::Vec3( 0., -sD, 0. );
    (*verts)[ 3 ] = osg::Vec3( 0., -sD, sh );
    (*verts)[ 4 ] = osg::Vec3( -sD, 0., 0. );
    (*verts)[ 5 ] = osg::Vec3( -sD, 0., sh );
    (*verts)[ 6 ] = osg::Vec3( 0., sD, 0. );
    (*verts)[ 7 ] = osg::Vec3( 0., sD, sh );
    (*verts)[ 8 ] = osg::Vec3( sD, 0., 0. );
    (*verts)[ 9 ] = osg::Vec3( sD, 0., sh );

    (*norms)[ 0 ] = osg::Vec3( 1., 0., 0. );
    (*norms)[ 1 ] = osg::Vec3( 1., 0., 0. );
    (*norms)[ 2 ] = osg::Vec3( 0., -1., 0. );
    (*norms)[ 3 ] = osg::Vec3( 0., -1., 0. );
    (*norms)[ 4 ] = osg::Vec3( -1., 0., 0. );
    (*norms)[ 5 ] = osg::Vec3( -1., 0., 0. );
    (*norms)[ 6 ] = osg::Vec3( 0., 1., 0. );
    (*norms)[ 7 ] = osg::Vec3( 0., 1., 0. );
    (*norms)[ 8 ] = osg::Vec3( 1., 0., 0. );
    (*norms)[ 9 ] = osg::Vec3( 1., 0., 0. );

    geom->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP, 0, 10 ) );


    // Head
    (*verts)[ 10 ] = osg::Vec3( hD, -hD, sh );
    (*verts)[ 11 ] = osg::Vec3( hD, hD, sh );
    (*verts)[ 12 ] = osg::Vec3( 0., 0., len );
    osg::Vec3 norm = ((*verts)[ 11 ] - (*verts)[ 10 ]) ^ ((*verts)[ 12 ] - (*verts)[ 10 ]);
    norm.normalize();
    (*norms)[ 10 ] = norm;
    (*norms)[ 11 ] = norm;
    (*norms)[ 12 ] = norm;

    (*verts)[ 13 ] = osg::Vec3( hD, hD, sh );
    (*verts)[ 14 ] = osg::Vec3( -hD, hD, sh );
    (*verts)[ 15 ] = osg::Vec3( 0., 0., len );
    norm = ((*verts)[ 14 ] - (*verts)[ 13 ]) ^ ((*verts)[ 15 ] - (*verts)[ 13 ]);
    norm.normalize();
    (*norms)[ 13 ] = norm;
    (*norms)[ 14 ] = norm;
    (*norms)[ 15 ] = norm;

    (*verts)[ 16 ] = osg::Vec3( -hD, hD, sh );
    (*verts)[ 17 ] = osg::Vec3( -hD, -hD, sh );
    (*verts)[ 18 ] = osg::Vec3( 0., 0., len );
    norm = ((*verts)[ 17 ] - (*verts)[ 16 ]) ^ ((*verts)[ 18 ] - (*verts)[ 16 ]);
    norm.normalize();
    (*norms)[ 16 ] = norm;
    (*norms)[ 17 ] = norm;
    (*norms)[ 18 ] = norm;

    (*verts)[ 19 ] = osg::Vec3( -hD, -hD, sh );
    (*verts)[ 20 ] = osg::Vec3( hD, -hD, sh );
    (*verts)[ 21 ] = osg::Vec3( 0., 0., len );
    norm = ((*verts)[ 20 ] - (*verts)[ 19 ]) ^ ((*verts)[ 21 ] - (*verts)[ 19 ]);
    norm.normalize();
    (*norms)[ 19 ] = norm;
    (*norms)[ 20 ] = norm;
    (*norms)[ 21 ] = norm;

    geom->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLES, 10, 12 ) );

    return( true );
}

osg::Geometry*
osgwTools::makeArrow( osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildArrowData( geom.get() );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeArrow: Error during arrow build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}

osg::Geometry*
osgwTools::makeArrow( const osg::Matrix& m, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeArrow( geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}




static bool
buildCylinderData( const double length, const double radius0, const double radius1, const osg::Vec2s& subdivisions, osg::Geometry* geometry, const bool wire )
{
    int subCylinders = subdivisions[ 0 ];
    if( subCylinders < 1 )
        subCylinders = 1;
    const double radiusDelta = ( radius1 - radius0 ) / subCylinders;

    osg::Vec3Array* vertices;
    if( geometry->getVertexArray() != NULL )
    {
        vertices = dynamic_cast< osg::Vec3Array* >( geometry->getVertexArray() );
        if( vertices == NULL )
            return( false );
    }
    else
    {
        vertices = new osg::Vec3Array;
        geometry->setVertexArray( vertices );
    }

    osg::Vec3Array* normals;
    osg::Vec2Array* texCoords;
    if( !wire )
    {
        if( geometry->getNormalArray() != NULL )
        {
            normals = dynamic_cast< osg::Vec3Array* >( geometry->getNormalArray() );
            if( normals == NULL )
                return( false );
        }
        else
        {
            normals = new osg::Vec3Array;
            geometry->setNormalArray( normals );
        }
        geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

        if( geometry->getTexCoordArray( 0 ) != NULL )
        {
            texCoords = dynamic_cast< osg::Vec2Array* >( geometry->getTexCoordArray( 0 ) );
            if( texCoords == NULL )
                return( false );
        }
        else
        {
            texCoords = new osg::Vec2Array;
            geometry->setTexCoordArray( 0, texCoords );
        }
    }

    osg::Vec4Array* osgC = new osg::Vec4Array;
    osgC->push_back( osg::Vec4( 1., 1., 1., 1. ) );
    geometry->setColorArray( osgC );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );


    // Generate a set of normals that we'll map to each cylinder hoop.
    osg::Vec4 plane( 0., 0., 1., 0. );
    osg::ref_ptr< osg::Vec3Array > cNorms;
    if( !wire )
        cNorms = generateCircleVertices( subdivisions[ 1 ], 1., plane, true );

    int idx;
    for( idx=0; idx <= subCylinders; idx++ )
    {
        const double percent( (double)idx / (double)subCylinders );
        plane[ 3 ] = length * percent;

        const double radius = radius0 + ( idx * radiusDelta );
        osg::ref_ptr< osg::Vec3Array > cVerts = generateCircleVertices( subdivisions[ 1 ], radius, plane, !wire );
        vertices->insert( vertices->end(), cVerts->begin(), cVerts->end() );

        if( !wire )
        {
            normals->insert( normals->end(), cNorms->begin(), cNorms->end() );

            const double tVal( percent );
            texCoords->reserve( vertices->size() );
            unsigned int tcIdx;
            for( tcIdx = 0; tcIdx < cVerts->size(); tcIdx++ )
            {
                const double sVal( (double)tcIdx / (double)( cVerts->size() - 1 ) );
                texCoords->push_back( osg::Vec2( sVal, tVal ) );
            }
        }
    }


    // Add PrimitiveSets

    if( !wire )
    {
        const unsigned int vertCount = vertices->size() / ( subCylinders + 1 );
        for( idx=0; idx < subCylinders; idx++ )
        {
            osg::DrawElementsUShort* deus = new osg::DrawElementsUShort( GL_TRIANGLE_STRIP );
            unsigned int vIdx = vertCount * ( idx + 1 );
            unsigned int innerIdx;
            for( innerIdx = 0; innerIdx < vertCount; innerIdx++ )
            {
                deus->push_back( vIdx );
                deus->push_back( vIdx - vertCount );
                vIdx++;
            }
            geometry->addPrimitiveSet( deus );
        }
    }
    else
    {
        const unsigned int vertCount = vertices->size() / ( subCylinders + 1 );
        unsigned int vIdx = 0;
        for( idx=0; idx <= subCylinders; idx++ )
        {
            osg::DrawElementsUShort* deus = new osg::DrawElementsUShort( GL_LINE_LOOP );
            deus->reserve( vertCount );
            unsigned int innerIdx;
            for( innerIdx = 0; innerIdx < vertCount; innerIdx++ )
            {
                deus->push_back( vIdx );
                vIdx++;
            }
            geometry->addPrimitiveSet( deus );
        }

        const unsigned int lineIndex = vertices->size() - vertCount;
        osg::DrawElementsUShort* deusl = new osg::DrawElementsUShort( GL_LINES );
        deusl->reserve( vertCount * 2 );
        for( vIdx=0; vIdx<vertCount; vIdx++ )
        {
            deusl->push_back( vIdx );
            deusl->push_back( vIdx + lineIndex );
        }
        geometry->addPrimitiveSet( deusl );
    }

    return( true );
}

osg::Geometry*
osgwTools::makeOpenCylinder( const double length, const double radius0, const double radius1, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildCylinderData( length, radius0, radius1, subdivisions, geom.get(), false );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeOpenCylinder: Error during cylinder build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}

osg::Geometry*
osgwTools::makeOpenCylinder( const osg::Matrix& m, const double length, const double radius0, const double radius1, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeOpenCylinder( length, radius0, radius1, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}

osg::Geometry*
osgwTools::makeClosedCylinder( const double length, const double radius0, const double radius1, const bool cap0, const bool cap1, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeOpenCylinder( length, radius0, radius1, subdivisions, geometry );
    if( geom == NULL )
    {
        osg::notify( osg::WARN ) << "makeClosedCylinder: Error during cylinder build." << std::endl;
        return( NULL );
    }

    // Calling makeCircle does result in repeating vertices.
    // However, this is required when rendering a solid cylinder
    // because the normals and tex coords for those shared
    // vertices are different.

    osg::Vec4 plane( 0., 0., -1., 0. );
    if( cap0 )
    {
        makeCircle( plane, radius0, subdivisions[ 1 ], geom );
    }

    if( cap1 )
    {
        plane[2] = 1.;
        plane[3] = length;
        makeCircle( plane, radius1, subdivisions[ 1 ], geom );
    }

    return( geom );
}

osg::Geometry*
osgwTools::makeClosedCylinder( const osg::Matrix& m, const double length, const double radius0, const double radius1, const bool cap0, const bool cap1, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeClosedCylinder( length, radius0, radius1, cap0, cap1, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}

osg::Geometry*
osgwTools::makeWireCylinder( const double length, const double radius0, const double radius1, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildCylinderData( length, radius0, radius1, subdivisions, geom.get(), true );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeWireCylinder: Error during cylinder build." << std::endl;
        return( NULL );
    }
    else
    {
        // Disable lighting and texture mapping for wire primitives.
        osg::StateSet* stateSet = geom->getOrCreateStateSet();
        stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
        stateSet->setTextureMode( 0, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

        return( geom.release() );
    }
}

osg::Geometry*
osgwTools::makeWireCylinder( const osg::Matrix& m, const double length, const double radius0, const double radius1, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeWireCylinder( length, radius0, radius1, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}


osg::Geometry*
osgwTools::makeCone( const double length, const double radius, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    return( makeClosedCylinder( length, radius, 0., true, false, subdivisions, geometry ) );
}

osg::Geometry*
osgwTools::makeCone( const osg::Matrix& m, const double length, const double radius, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    return( makeClosedCylinder( m, length, radius, 0., true, false, subdivisions, geometry ) );
}


static bool
buildCapsuleData( const double length, const double radius, const osg::Vec2s& subdivisions, osg::Geometry* geometry, const bool wire )
{
    // Note: For simplicity, subdivisions around the capsule must be at least 3.
    unsigned short subs = osg::maximum< unsigned short >( 3, subdivisions[ 1 ] );

    const double cylinderLength = osg::maximum< double >( 0., length - radius * 2. );
    buildCylinderData( cylinderLength, radius, radius, osg::Vec2s( subdivisions[ 0 ], subs ), geometry, wire );
    osgwTools::transform( osg::Matrix::translate( 0., 0., radius ), geometry );

    osg::ref_ptr< osg::Vec3Array > vertices = dynamic_cast< osg::Vec3Array* >( geometry->getVertexArray() );
    if( vertices == NULL )
        return( false );

    osg::ref_ptr< osg::Vec3Array > normals;
    osg::ref_ptr< osg::Vec2Array > texCoords;
    if( !wire )
    {
        normals = dynamic_cast< osg::Vec3Array* >( geometry->getNormalArray() );
        if( normals == NULL )
            return( false );
        texCoords = dynamic_cast< osg::Vec2Array* >( geometry->getTexCoordArray( 0 ) );
        if( texCoords == NULL )
            return( false );
    }


    const unsigned int capStartIdx( vertices->size() );

    int numHoops = osg::maximum< int >( ( subs >> 2 ) - 1, 1 );
    double latDelta = osg::PI_2 / ( numHoops + 1 );
    int idx;
    // Capsule bottom
    double lat( osg::PI_2 );
    vertices->insert( vertices->end(), osg::Vec3( 0., 0., 0. ) );
    if( !wire )
    {
        normals->push_back( osg::Vec3( 0., 0., -1. ) );
        texCoords->push_back( osg::Vec2( 0., 0. ) );
    }
    for( idx=0; idx<numHoops; idx++ )
    {
        lat -= latDelta;
        const double height = sin( lat ) * radius;
        const double tempRad = cos( lat ) * radius;
        const osg::Vec4 plane( 0., 0., 1., radius-height );
        osg::ref_ptr< osg::Vec3Array > v = generateCircleVertices( subs, tempRad, plane, !wire );

        vertices->insert( vertices->end(), v->begin(), v->end() );

        if( !wire )
        {
            for( unsigned int vIdx=0; vIdx < v->size(); vIdx++ )
            {
                osg::Vec3 normal = (*v)[ vIdx ];
                normal[2] -= radius;
                normal.normalize();
                normals->push_back( normal );
                texCoords->push_back( osg::Vec2( normal[ 0 ], normal[ 1 ] ) );
            }
        }
    }
    // Capsule top
    lat = 0.;
    for( idx=0; idx<numHoops; idx++ )
    {
        lat += latDelta;
        const double height = sin( lat ) * radius;
        const double tempRad = cos( lat ) * radius;
        const osg::Vec4 plane( 0., 0., 1., length - radius + height );
        osg::ref_ptr< osg::Vec3Array > v = generateCircleVertices( subs, tempRad, plane, !wire );

        vertices->insert( vertices->end(), v->begin(), v->end() );

        if( !wire )
        {
            // TBD add normals and tex coords
            const osg::Vec3 center( 0., 0., length - radius );
            for( unsigned int vIdx=0; vIdx < v->size(); vIdx++ )
            {
                osg::Vec3 normal = (*v)[ vIdx ] - center;
                normal.normalize();
                normals->push_back( normal );
                texCoords->push_back( osg::Vec2( normal[ 0 ], normal[ 1 ] ) );
            }
        }
    }
    vertices->insert( vertices->end(), osg::Vec3( 0., 0., length ) );
    if( !wire )
    {
        normals->push_back( osg::Vec3( 0., 0., 1. ) );
        texCoords->push_back( osg::Vec2( 0., 0. ) );
    }


    // PrimitiveSets
    if( !wire )
    {
        // Solid capsule
        int vIdx;

        osg::DrawElementsUShort* deus = new osg::DrawElementsUShort( GL_TRIANGLE_FAN );
        deus->reserve( subs + 2 );
        deus->push_back( capStartIdx );
        for( vIdx=subs; vIdx >= 0; vIdx-- )
            deus->push_back( capStartIdx + vIdx + 1 );
        geometry->addPrimitiveSet( deus );

        unsigned int cylIdx = 0;
        vIdx = 0;
        int hIdx;
        for( hIdx=0; hIdx < numHoops; hIdx++ )
        {
            deus = new osg::DrawElementsUShort( GL_TRIANGLE_STRIP );
            deus->reserve( ( subs + 1 ) * 2 );
            for( idx=0; idx <= subs; idx++ )
            {
                if( hIdx == numHoops-1 )
                    deus->push_back( cylIdx++ );
                else
                    deus->push_back( capStartIdx + vIdx + subs + 2 );
                deus->push_back( capStartIdx + vIdx + 1 );
                vIdx++;
            }
            geometry->addPrimitiveSet( deus );
        }

        cylIdx = capStartIdx - subs - 1;
        for( hIdx=0; hIdx < numHoops; hIdx++ )
        {
            deus = new osg::DrawElementsUShort( GL_TRIANGLE_STRIP );
            deus->reserve( ( subs + 1 ) * 2 );
            for( idx=0; idx <= subs; idx++ )
            {
                deus->push_back( capStartIdx + vIdx + 1 );
                if( hIdx == 0 )
                    deus->push_back( cylIdx++ );
                else
                    deus->push_back( capStartIdx + vIdx - subs );
                vIdx++;
            }
            geometry->addPrimitiveSet( deus );
        }

        vIdx -= ( subs + 1 );
        deus = new osg::DrawElementsUShort( GL_TRIANGLE_FAN );
        deus->reserve( subs + 2 );
        deus->push_back( vertices->size() - 1 );
        for( ; capStartIdx + vIdx + 1 < vertices->size() - 1; vIdx++ )
            deus->push_back( capStartIdx + vIdx + 1 );
        geometry->addPrimitiveSet( deus );
    }
    else
    {
        // Wire capsule
        unsigned short vIdx( (unsigned short)capStartIdx + 1 );
        for( idx=0; idx < numHoops * 2; idx++ )
        {
            osg::DrawElementsUShort* deus = new osg::DrawElementsUShort( GL_LINE_LOOP );
            deus->reserve( subs );
            for( int count=0; count < subs; count++ )
                deus->push_back( vIdx++ );
            geometry->addPrimitiveSet( deus );
        }

        unsigned int topCapStart = capStartIdx + (subs * numHoops) + 1;
        for( idx=0; idx < subs; idx++ )
        {
            osg::DrawElementsUShort* deus = new osg::DrawElementsUShort( GL_LINE_STRIP );
            deus->reserve( numHoops + 2 );
            deus->push_back( capStartIdx );
            for( int jdx=0; jdx<numHoops; jdx++ )
                deus->push_back( capStartIdx + ( jdx * subs ) + 1 + idx );
            deus->push_back( idx );
            geometry->addPrimitiveSet( deus );

            deus = new osg::DrawElementsUShort( GL_LINE_STRIP );
            deus->reserve( numHoops + 2 );
            deus->push_back( capStartIdx - subs + idx );
            for( int jdx=0; jdx<numHoops; jdx++ )
                deus->push_back( topCapStart + ( jdx * subs ) + idx );
            deus->push_back( vertices->size() - 1 );
            geometry->addPrimitiveSet( deus );
        }
    }

    return( true );
}

osg::Geometry*
osgwTools::makeCapsule( const double length, const double radius, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildCapsuleData( length, radius, subdivisions, geom.get(), false );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeCapsule: Error during capsule build." << std::endl;
        return( NULL );
    }
    else
        return( geom.release() );
}

osg::Geometry*
osgwTools::makeCapsule( const osg::Matrix& m, const double length, const double radius, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeCapsule( length, radius, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}

osg::Geometry*
osgwTools::makeWireCapsule( const double length, const double radius, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::ref_ptr< osg::Geometry > geom( geometry );
    if( geom == NULL )
        geom = new osg::Geometry;

    bool result = buildCapsuleData( length, radius, subdivisions, geom.get(), true );
    if( !result )
    {
        osg::notify( osg::WARN ) << "makeWireCapsule: Error during capsule build." << std::endl;
        return( NULL );
    }
    else
    {
        // Disable lighting and texture mapping for wire primitives.
        osg::StateSet* stateSet = geom->getOrCreateStateSet();
        stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
        stateSet->setTextureMode( 0, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

        return( geom.release() );
    }
}

osg::Geometry*
osgwTools::makeWireCapsule( const osg::Matrix& m, const double length, const double radius, const osg::Vec2s& subdivisions, osg::Geometry* geometry )
{
    osg::Geometry* geom = osgwTools::makeWireCapsule( length, radius, subdivisions, geometry );
    if( geom != NULL )
        transform( m, geom );
    return( geom );
}
