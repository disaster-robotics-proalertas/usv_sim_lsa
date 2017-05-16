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

#include <osgwTools/ReducerOp.h>
#include <osgwTools/PrimitiveSetConversion.h>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/TriangleFunctor>
#include <osg/CopyOp>
#include <osg/io_utils>
#include <osg/Math>
#include <vector>
#include <map>
#include <math.h>



namespace osgwTools {


ReducerOp::ReducerOp()
  : _removeDegenerateAndRedundantTriangles( true )
{
    setGroupThreshold( 10. );
    setMaxEdgeError( 10.f );
}
ReducerOp::ReducerOp( const ReducerOp& rhs, const osg::CopyOp& copyOp )
  : _removeDegenerateAndRedundantTriangles( rhs._removeDegenerateAndRedundantTriangles )
{
    setGroupThreshold( rhs._groupThreshold );
    setMaxEdgeError( rhs._maxEdgeError );
}
ReducerOp::~ReducerOp()
{
}

void
ReducerOp::setGroupThreshold( float groupThreshold )
{
    _groupThreshold = groupThreshold;
    _groupThresholdRad = osg::DegreesToRadians( groupThreshold );
}
void
ReducerOp::setMaxEdgeError( float maxEdgeError )
{
    _maxEdgeError = maxEdgeError;
    _maxEdgeErrorRad = osg::DegreesToRadians( maxEdgeError );
}


bool ReducerOp::convertToDEUITriangles( osg::Geometry* geom )
{
    const osg::Geometry::PrimitiveSetList& pslIn = geom->getPrimitiveSetList();
    osg::Geometry::PrimitiveSetList pslIntermed, pslIntermed2, pslOut;

    // Convert everything to a DEUI
    osg::Geometry::PrimitiveSetList::const_iterator it;
    for( it=pslIn.begin(); it != pslIn.end(); it++ )
    {
        const osg::ref_ptr< osg::PrimitiveSet > primSet = *it;

        if( primSet->getType() == osg::PrimitiveSet::DrawArraysPrimitiveType )
            pslIntermed.push_back( convertToDEUI( static_cast< const osg::DrawArrays* >( primSet.get() ) ) );
        else if( primSet->getType() == osg::PrimitiveSet::DrawArrayLengthsPrimitiveType )
        {
            osg::Geometry::PrimitiveSetList newPsl = convertToDEUI( static_cast< const osg::DrawArrayLengths* >( primSet.get() ) );
            pslIntermed.insert( pslIntermed.end(), newPsl.begin(), newPsl.end() );
        }
        else if( primSet->getType() == osg::PrimitiveSet::DrawElementsUBytePrimitiveType )
            pslIntermed.push_back( convertToDEUI( static_cast< const osg::DrawElementsUByte* >( primSet.get() ) ) );
        else if( primSet->getType() == osg::PrimitiveSet::DrawElementsUShortPrimitiveType )
            pslIntermed.push_back( convertToDEUI( static_cast< const osg::DrawElementsUShort* >( primSet.get() ) ) );
        else if( primSet->getType() == osg::PrimitiveSet::DrawElementsUIntPrimitiveType )
            pslIntermed.push_back( primSet );
    }

    // Convert all filled DEUIs to triangles DEUIs.
    for( it=pslIntermed.begin(); it != pslIntermed.end(); it++ )
    {
        const osg::DrawElementsUInt* deui = static_cast< const osg::DrawElementsUInt* >( (*it).get() );
        pslIntermed2.push_back( convertAllFilledToTriangles( deui ) );
    }

    // Batch DEUI triangles into a minimum number of PrimitiveSets
    osg::ref_ptr< osg::DrawElementsUInt > newDeui( new osg::DrawElementsUInt( GL_TRIANGLES ) );
    pslOut.push_back( newDeui );
    unsigned int remainder = 0xffffffff;
    for( it=pslIntermed2.begin(); it != pslIntermed2.end(); it++ )
    {
        const osg::DrawElementsUInt* deui = static_cast< const osg::DrawElementsUInt* >( (*it).get() );
        if( deui->getMode() == GL_TRIANGLES )
        {
            if( remainder < deui->size() )
            {
                // We filled the newDeui.
                newDeui = new osg::DrawElementsUInt( GL_TRIANGLES );
                pslOut.push_back( newDeui );
                remainder = 0xffffffff;
            }

            newDeui->insert( newDeui->end(), deui->begin(), deui->end() );
            remainder -= deui->size();
        }
    }

    geom->setPrimitiveSetList( pslOut );

    return( true );
}

bool
ReducerOp::makeMap( VertToTriMap& v2t, const osg::Geometry& geom )
{
    // TBD This needs to be more general-purpose. Right now it only works for
    // DrawElements* and DrawArrays with mode TRIANGLES and Vec3Array
    // vertices (suitable for PolyTrans output).

    const osg::Vec3Array* verts = dynamic_cast< const osg::Vec3Array* >( geom.getVertexArray() );
    if( verts == NULL )
    {
        osg::notify( osg::WARN ) << "ReducerOp: Unsupported VertexArray." << std::endl;
        return( false );
    }

    std::string errorStr;
    unsigned int psIdx;
    for( psIdx=0; psIdx<geom.getNumPrimitiveSets(); psIdx++ )
    {
        const osg::PrimitiveSet* ps( geom.getPrimitiveSet( psIdx ) );
        if( ( ps->getType() == osg::PrimitiveSet::DrawArrayLengthsPrimitiveType ) ||
            ( ps->getMode() != GL_TRIANGLES ) )
        {
            errorStr = "ReducerOp: Unsupported PrimitiveSet type or mode.";
            continue;
        }

        bool allocated( false );
        unsigned int jdx( 0 );
        const unsigned int idxCount( ps->getNumIndices() );
        unsigned int* indexes;
        if( ps->getType() == osg::PrimitiveSet::DrawArraysPrimitiveType )
        {
            indexes = new unsigned int[ idxCount ];
            allocated = true;
            const osg::DrawArrays* da = dynamic_cast< const osg::DrawArrays* >( ps );
            unsigned int dai( da->getFirst() );
            while( jdx < idxCount )
                indexes[ jdx++ ] = dai++;
        }
        else if( ps->getType() == osg::PrimitiveSet::DrawElementsUBytePrimitiveType )
        {
            indexes = new unsigned int[ idxCount ];
            allocated = true;
            const unsigned char* ubi = static_cast< const unsigned char* >( ps->getDataPointer() );
            while( jdx < idxCount )
            {
                indexes[ jdx ] = ubi[ jdx ];
                jdx++;
            }
        }
        else if( ps->getType() == osg::PrimitiveSet::DrawElementsUShortPrimitiveType )
        {
            indexes = new unsigned int[ idxCount ];
            allocated = true;
            const unsigned short* usi = static_cast< const unsigned short* >( ps->getDataPointer() );
            while( jdx < idxCount )
            {
                indexes[ jdx ] = usi[ jdx ];
                jdx++;
            }
        }
        else
            // Must be uint
            indexes = const_cast< unsigned int* >( static_cast< const unsigned int* >( ps->getDataPointer() ) );

        for( jdx=0; (jdx+2)<idxCount; jdx+=3 )
        {
            unsigned int v0( indexes[ jdx ] );
            unsigned int v1( indexes[ jdx+1 ] );
            unsigned int v2( indexes[ jdx+2 ] );
            if( _removeDegenerateAndRedundantTriangles &&
                ( (v0==v1) || (v0==v2) || (v1 == v2) ) )
                continue;
            Tri tri( v0, v1, v2, verts );
            v2t[ v0 ].push_back( tri );
            v2t[ v1 ].push_back( tri );
            v2t[ v2 ].push_back( tri );
        }

        if( allocated )
            delete[] indexes;
    }

    if( errorStr != "" )
        osg::notify( osg::WARN ) << errorStr << std::endl;

#if 0
    {
        // DEBUG dump out the map.
        VertToTriMap::const_iterator itr;
        for( itr=v2t.begin(); itr != v2t.end(); itr++ )
        {
            osg::notify( osg::ALWAYS ) << "Index " << itr->first << std::endl;
            const TriList& trilist( itr->second );
            TriList::const_iterator tlitr;
            for( tlitr=trilist.begin(); tlitr != trilist.end(); tlitr++ )
                osg::notify( osg::ALWAYS ) << "  " << tlitr->_v0 << " " << tlitr->_v1 << " " << tlitr->_v2 << std::endl;
        }
    }
#endif

    return( true );
}

void
ReducerOp::makeGroups( TriListList& tll, const TriList& tl )
{
    if( tl.size() < 2)
    {
        tll.push_back( tl );
        osg::notify( osg::INFO ) << "ReducerOp: makeGroup input list has size " << tl.size() << std::endl;
        return;
    }


    // Init the group list with the first triangle.
    const Tri firstTri( tl.front() );
    tll.resize( 1 );
    tll[ 0 ].push_back( firstTri );

    // Compare each remaining tri to each group in the group list.
    // Find the group in which all the normals differ by less thatn the group threshold.
    // If we can't find such a group, start a new group.
    TriList::const_iterator tlit;
    for( tlit = tl.begin()+1; tlit != tl.end(); tlit++ )
    {
        const Tri curTri( *tlit );

        bool pass( true );
        TriListList::iterator tllit;
        for( tllit=tll.begin(); tllit != tll.end(); tllit++ )
        {
            pass = true;
            TriList& grp( *tllit );
            TriList::const_iterator grpIt;
            for( grpIt=grp.begin(); grpIt != grp.end(); grpIt++ )
            {
                const float d( curTri._norm * grpIt->_norm );
                pass = ( acosf( d ) < _groupThresholdRad );
            }
            // if pass, add tri to this grp and stop looping.
            if( pass )
            {
                grp.push_back( curTri );
                break;
            }
        }
        // if not pass, resize tll to size+1 and add tri to new group.
        if( !pass )
        {
            unsigned int grpIdx( tll.size() );
            tll.resize( grpIdx+1 );
            tll[ grpIdx ].push_back( curTri );
        }
    }


    // Check for and remove redundant triangles
    if( _removeDegenerateAndRedundantTriangles )
    {
        TriListList::iterator tllit;
        for( tllit = tll.begin(); tllit != tll.end(); tllit++ )
        {
            TriList& origGrp = *tllit;
            TriList newGrp;
            newGrp.push_back( origGrp[ 0 ] );
            unsigned int origIdx;
            for( origIdx = 1; origIdx < origGrp.size(); origIdx++ )
            {
                bool pass( true );
                TriList::const_iterator newIdx;
                for( newIdx = newGrp.begin(); newIdx != newGrp.end(); newIdx++ )
                {
                    if( origGrp[ origIdx ] == *newIdx )
                    {
                        pass = false;
                        //osg::notify( osg::ALWAYS ) << "Found redundant triangle." << std::endl;
                        break;
                    }
                }
                if( pass )
                    newGrp.push_back( origGrp[ origIdx ] );
            }
            origGrp = newGrp;
        }
    }
}

ReducerOp::EdgeList
ReducerOp::findBoundaryEdges( const TriList& tl, unsigned int vertIdx )
{
    osg::notify( osg::INFO ) << " ** findBoundaryEdges: Enter." << std::endl;
    osg::notify( osg::INFO) << "      TL size " << tl.size() << std::endl;

    EdgeToTriMap e2t;
    TriList::const_iterator tri;
    for( tri = tl.begin(); tri != tl.end(); tri++ )
    {
        const Tri& t( *tri );
        if( t._v0 == vertIdx )
        {
            Edge e0( t._v2, t._v0 );
            Edge e1( t._v0, t._v1 );
            e2t[ e0 ].push_back( t );
            e2t[ e1 ].push_back( t );
        }
        else if( t._v1 == vertIdx )
        {
            Edge e0( t._v0, t._v1 );
            Edge e1( t._v1, t._v2 );
            e2t[ e0 ].push_back( t );
            e2t[ e1 ].push_back( t );
        }
        else if( t._v2 == vertIdx )
        {
            Edge e0( t._v1, t._v2 );
            Edge e1( t._v2, t._v0 );
            e2t[ e0 ].push_back( t );
            e2t[ e1 ].push_back( t );
        }
        else
        {
            osg::notify( osg::INFO ) << "findBoundaryEdges: Triangle doesn't reference current vertex." << std::endl;
        }
    }
    osg::notify( osg::INFO ) << "      EdgeToTriMap size " << e2t.size() << std::endl;

    EdgeList el;
    EdgeToTriMap::const_iterator e2tItr;
    for( e2tItr = e2t.begin(); e2tItr != e2t.end(); e2tItr++ )
    {
        if( e2tItr->second.size() == 1)
        {
            osg::notify( osg::INFO ) << "      Found edge." << std::endl;
            el.push_back( e2tItr->first );
        }
    }

#if 0
    // DEBUG error check.
    if( el.size() == 2 )
        osg::notify( osg::ALWAYS ) << "Boundary vert" << std::endl;
    else if( el.size() == 0 )
        osg::notify( osg::ALWAYS ) << "Contained vert" << std::endl;
    else
        osg::notify( osg::ALWAYS ) << "Error: Number of boundary edges " << el.size() << ", should be 0 or 2." << std::endl;
#endif

    osg::notify( osg::INFO ) << " ** findBoundaryEdges: Exit." << std::endl;
    return( el );
}

bool
ReducerOp::removeableEdge( const EdgeList& el, const osg::Vec3Array* verts )
{
    //osg::notify( osg::ALWAYS ) << "    removeableEdge EL size " << el.size() << std::endl;
    if( el.size() == 0 )
        // An empty boundary edge list means it's a contained vertex. Just remove it.
        return( true );

    if( el.size() != 2 )
    {
        // This can be caused by degenerate triangles. Hands off, don't reduce.
        return( false );
    }

    Edge a( el[ 0 ] );
    Edge b( el[ 1 ] );
    osg::Vec3 v0( (*verts)[ a._a ] - (*verts)[ a._b ] );
    v0.normalize();
    osg::Vec3 v1( (*verts)[ b._a ] - (*verts)[ b._b ] );
    v1.normalize();
    float d( osg::absolute( v0 * v1 ) );
    //osg::notify( osg::ALWAYS ) << "    Angle " << acosf(d) << ", error " << _maxEdgeErrorRad << std::endl;

    return( acosf( d ) < _maxEdgeErrorRad );
}

void
ReducerOp::orderVerts( unsigned int removeIdx, const TriList& tl, IndexList& idxList )
{
#if 0
    {
        osg::notify( osg::ALWAYS ) << "Remove: " << removeIdx << "  Tri dump:" << std::endl;
        TriList::const_iterator tri;
        for( tri = tl.begin(); tri != tl.end(); tri++ )
        {
            const Tri& t( *tri );
            osg::notify( osg::ALWAYS ) << "  " << t._v0 << " " << t._v1 << "  " << t._v2 << std::endl;
        }
    }
#endif

    // Make an ordered list of all the edges in the TriList that _don't_ contain the index being removed.
    EdgeList el;
    TriList::const_iterator tri;
    for( tri = tl.begin(); tri != tl.end(); tri++ )
    {
        const Tri& t( *tri );
        if( t._v0 == removeIdx )
        {
            Edge e( t._v1, t._v2, false );
            el.push_back( e );
        }
        else if( t._v1 == removeIdx )
        {
            Edge e( t._v2, t._v0, false );
            el.push_back( e );
        }
        else if( t._v2 == removeIdx )
        {
            Edge e( t._v0, t._v1, false );
            el.push_back( e );
        }
        else
        {
            osg::notify( osg::INFO ) << "orderVerts: Triangle doesn't reference removeIdx." << std::endl;
        }
    }

    // Find first edge. If the removed index was a boundary index, there will be
    // an edge whose A index is not on any other edge; this is the starting edge.
    // If we can't find an edge like this, it means the removed index was contained,
    // and any edge is suitable for starting, so we just use the first one.
    unsigned int eIdx;
    for( eIdx=0; eIdx<el.size(); eIdx++ )
    {
        unsigned int tIdx( 0 );
        Edge e0( el[ eIdx ] );
        Edge e1( el[ tIdx ] );
        while( ( e0._a != e1._b ) && ( tIdx+1<el.size() ) )
        {
            tIdx++;
            e1 = el[ tIdx ];
        }
        if( e0._a != e1._b )
        {
            if( eIdx != 0 )
            {
                Edge swap = el[ 0 ];
                el[ 0 ] = el[ eIdx ];
                el[ eIdx ] = swap;
            }
            break;
        }
    }

    // Sort the edge list so that they are in the order that preserves the normals.
    // They should be connected: edge B index should be equal to edge+1 A vertex.
    for( eIdx=0; eIdx+1<el.size(); eIdx++ )
    {
        unsigned int tIdx( eIdx );
        Edge e0( el[ eIdx ] );
        Edge e1( el[ tIdx ] );
        while( ( e0._b != e1._a ) && ( ++tIdx<el.size() ) )
            e1 = el[ tIdx ];
        if( ( e0._b != e1._a ) )
        {
            osg::notify( osg::WARN ) << "orderVerts, could not find next edge. Should never get here." << std::endl;
            osg::notify( osg::WARN ) << "     Edge list dump follows." << std::endl;
            for( eIdx=0; eIdx<el.size(); eIdx++ )
                osg::notify( osg::WARN ) << "  " << el[eIdx]._a << " " << el[eIdx]._b;
            osg::notify( osg::WARN ) << std::endl;
            idxList.clear();
            return;
        }
        if( eIdx+1 != tIdx ) 
        {
            Edge swap = el[ eIdx+1 ];
            el[ eIdx+1 ] = el[ tIdx ];
            el[ tIdx ] = swap;
        }
    }

    // Create an index list from the edge list. This is a polygon
    // created from all the indices in the group minus the index being removed.
    if( el[ el.size()-1 ]._b != el[ 0 ]._a )
        idxList.push_back( el[ 0 ]._a );
    for( eIdx=0; eIdx<el.size(); eIdx++ )
        idxList.push_back( el[ eIdx ]._b );
}

void
ReducerOp::removeTri( const Tri& tri, TriList& tl )
{
    // Find the tri on the TriList and remove it.
    TriList::iterator tlit( tl.begin() );
    while( tlit != tl.end() )
    {
        if( *tlit == tri )
        {
            tlit = tl.erase( tlit );
            // Don't break! Go though the whole list in case of redundant triangles.
            continue;
        }
        tlit++;
    }
}

// Checks to see if the vertex is removeable. Does some redundant work
// with remove Vertex.
bool
ReducerOp::removeableVertex( unsigned int removeIdx, const TriList& tl, osg::Vec3Array* verts )
{
    IndexList idxList;
    orderVerts( removeIdx, tl, idxList );
    if( idxList.size() == 0 )
        return( false );

    // Make new list of triangles to replace current (tl) list.
    TriList newTris;
    unsigned int idx;
    for( idx=0; idx+2<idxList.size(); idx++ )
    {
        Tri tri( idxList[ 0 ], idxList[ idx+1 ], idxList[ idx+2 ], verts );
        if( tri._norm.length2() == 0.f)
            // Don't create degenerate triangles.
            continue;
        newTris.push_back( tri );
    }
    if( newTris.size() == 0 )
        return( false );

    // Do not allow creation of backfacing triangles
    TriList::const_iterator tlit( newTris.begin() );
    const osg::Vec3& norm( tlit->_norm );
    for( tlit=newTris.begin()+1; tlit != newTris.end(); tlit++ )
    {
        if( (norm * tlit->_norm) < 0. )
            return( false );
    }

    return( true );
}

void
ReducerOp::deleteVertex( unsigned int removeIdx, const TriList& tl, VertToTriMap& v2t, osg::Vec3Array* verts )
{
    IndexList idxList;
    orderVerts( removeIdx, tl, idxList );

#if 0
    // debug only.
    IndexList::const_iterator iii;
    for( iii=idxList.begin(); iii != idxList.end(); iii++ )
        osg::notify( osg::ALWAYS ) << " " << *iii;
    osg::notify( osg::ALWAYS ) << std::endl;
#endif

    // Make new list of triangles to replace current (tl) list.
    TriList newTris;
    unsigned int idx;
    for( idx=0; idx+2<idxList.size(); idx++ )
    {
        Tri tri( idxList[ 0 ], idxList[ idx+1 ], idxList[ idx+2 ], verts );
        if( tri._norm.length2() == 0.f)
            // Don't create degenerate triangles.
            continue;
        newTris.push_back( tri );
    }

    // For each triangle in tl, and each vertex in the tri, look up the vertex and remove the tri from the v2t map.
    TriList::const_iterator tlit( newTris.begin() );
    for( tlit=tl.begin(); tlit != tl.end(); tlit++ )
    {
        const Tri& tri = *tlit;
        VertToTriMap::iterator v2tmit;

        v2tmit = v2t.find( tri._v0 );
        if( v2tmit == v2t.end() )
            osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
        removeTri( tri, v2tmit->second );

        v2tmit = v2t.find( tri._v1 );
        if( v2tmit == v2t.end() )
            osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
        removeTri( tri, v2tmit->second );

        v2tmit = v2t.find( tri._v2 );
        if( v2tmit == v2t.end() )
            osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
        removeTri( tri, v2tmit->second );
    }

    // For each tri in newTris, find each vertex in the tri and add the triangle to the v2t map.
    for( tlit=newTris.begin(); tlit != newTris.end(); tlit++ )
    {
        const Tri& tri = *tlit;
        VertToTriMap::iterator v2tmit;

        v2tmit = v2t.find( tri._v0 );
        if( v2tmit == v2t.end() )
            osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
        v2tmit->second.push_back( tri );

        v2tmit = v2t.find( tri._v1 );
        if( v2tmit == v2t.end() )
            osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
        v2tmit->second.push_back( tri );

        v2tmit = v2t.find( tri._v2 );
        if( v2tmit == v2t.end() )
            osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
        v2tmit->second.push_back( tri );
    }
}

void
ReducerOp::reduce( osg::Geometry& geom )
{
    //
    // Step 1: Create a map of vertices to triangles. This is a 1 to many map.
    // Each triangle contains its three indices and a normalized facet normal.
    osg::Array* vArray = geom.getVertexArray();
    osg::ref_ptr< osg::Vec3Array > verts = dynamic_cast< osg::Vec3Array* >( vArray );
    if( verts == NULL )
        return;

    VertToTriMap v2t;
    bool success = makeMap( v2t, geom );
    if( !success )
    {
        osg::notify( osg::WARN ) << "ReducerOp: makeMap failed." << std::endl;
        return;
    }

    VertToTriMap::iterator currentVert;
    for( currentVert=v2t.begin(); currentVert != v2t.end(); currentVert++ )
    {
        // Step 2: For each vertex in the map, create a list of groups of triangles that
        // share that vertex. All triangles within a group will have normals that differ
        // by no more than _groupThreshold degrees. Example: If all triangles are planar,
        // there will be a single group for the shared vertex. Groups come back to us as
        // a TriListList, a vector of vector of triangles.
        const TriList& tl( currentVert->second );
        TriListList tll;
        makeGroups( tll, tl );

        // If a vertex can't be removed from all groups, then don't remove it
        // from any group.
        // See the dectest32.osg test case.
        bool remove( true );
        TriListList::const_iterator gitr;
        for( gitr=tll.begin(); gitr!= tll.end(); gitr++ )
        {
            if( gitr->size() < 2 )
            {
                remove = false;
                break;
            }

            EdgeList el = findBoundaryEdges( *gitr, currentVert->first );
            if( !removeableEdge( el, verts.get() ) ||
                !removeableVertex( currentVert->first, *gitr, verts.get() ) )
            {
                remove = false;
                break;
            }
        }
        if( !remove )
            continue;
        // We probably computed some stuff above that we will recompute in the
        // following code block. This is a potential future optimization.

        for( gitr=tll.begin(); gitr!= tll.end(); gitr++ )
        {
            if( gitr->size() < 2 )
                // Can't reduce this group.
                continue;

#if 0
// Shouldn't need to find and check the boundary edge again.
// Should just be able to delete the vertex.

            // Step 3: For each group of triangles within the group threshold, generate a list
            // of "boundary edges": triangle edges that share the current vertex and are used
            // by only one triangle.
            //  * If the list is empty, the vertex is completely contained.
            //  * If not empty, there must be only two edges.
            EdgeList el = findBoundaryEdges( *gitr, currentVert->first );

            // removeableEdge returns true of the EdgeList contains zero edges (the vertex is
            // contained), or it contains 2 edges who's angle differs by less than max edge error.
            if( removeableEdge( el, verts.get() ) )
#endif
                deleteVertex( currentVert->first, *gitr, v2t, verts.get() );
        }
    }


    // Count the number of indices we'll need so that we can do a fast resize
    // on the destination memory block. Simultaneously check for and remove
    // redundant triangles.
    unsigned int numIndices( 0 );
    VertToTriMap::const_iterator v2tmit;
    for( v2tmit=v2t.begin(); v2tmit != v2t.end(); v2tmit++ )
    {
        const unsigned int currentIndex( v2tmit->first );

        // Each triangle appears in the VertsToTriMap 3 times.
        // Find and remove the other two.
        const TriList& tl = v2tmit->second;
        TriList::const_iterator tlit;
        for( tlit=tl.begin(); tlit != tl.end(); tlit++ )
        {
            const Tri& tri( *tlit );
            VertToTriMap::iterator target;

            if( tri._v0 != currentIndex )
            {
                target = v2t.find( tri._v0 );
                if( target == v2t.end() )
                    osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
                removeTri( tri, target->second );
            }

            if( tri._v1 != currentIndex )
            {
                target = v2t.find( tri._v1 );
                if( target == v2t.end() )
                    osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
                removeTri( tri, target->second );
            }

            if( tri._v2 != currentIndex )
            {
                target = v2t.find( tri._v2 );
                if( target == v2t.end() )
                    osg::notify( osg::ALWAYS ) << "Can't find vertex in v2t." << std::endl;
                removeTri( tri, target->second );
            }
        }

        numIndices += ( tl.size() * 3 );
    }

    osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );
    deui->resize( numIndices );

    unsigned int idx( 0 );
    for( v2tmit=v2t.begin(); v2tmit != v2t.end(); v2tmit++ )
    {
        const TriList& tl = v2tmit->second;
        TriList::const_iterator tlit;
        for( tlit=tl.begin(); tlit != tl.end(); tlit++ )
        {
            const Tri& tri( *tlit );
            (*deui)[ idx++ ] = tri._v0;
            (*deui)[ idx++ ] = tri._v1;
            (*deui)[ idx++ ] = tri._v2;
        }
    }

    geom.removePrimitiveSet( 0, geom.getNumPrimitiveSets() );
    geom.addPrimitiveSet( deui );
}

osg::Geometry*
ReducerOp::operator()( osg::Geometry& geom )
{
    // ReducerOp works only with DrawElementsUInt GL_TRIANGLES.
    // Convert all PrimitiveSets to this format.
    if( !( convertToDEUITriangles( &geom ) ) )
    {
        osg::notify( osg::WARN ) << "ReducerOp: Unable to convert to DrawElementsUInt TRIANGLES." << std::endl;
        return( &geom );
    }


    // Run possibly multiple passes to ensure complete reduction.
    unsigned int preIndices, postIndices( 0 );

    // Reduction will be demonstrated in reduced number of indices.
    // Get the initial index count.
    unsigned int idx;
    for( idx=0; idx < geom.getNumPrimitiveSets(); idx++ )
        postIndices += geom.getPrimitiveSet( idx )->getNumIndices();

    int reducerPass( 0 );
    do {
        preIndices = postIndices;

        // Do a reduction pass.
        reduce( geom );

        // Get the result index count.
        postIndices = 0;
        for( idx=0; idx < geom.getNumPrimitiveSets(); idx++ )
            postIndices += geom.getPrimitiveSet( idx )->getNumIndices();

        //osg::notify( osg::ALWAYS ) << "ReducerOp pass " << reducerPass++ << ": start indices " << preIndices << ", end indices " << postIndices << std::endl;

    // Continue if the result is smaller that the start number of indices.
    // However, don't repeat more than (arbitrary) 10 times.
    } while( (postIndices < preIndices) && (reducerPass < 10) );

    return( &geom );
}

}
