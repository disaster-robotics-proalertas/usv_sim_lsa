/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2012 by Kenneth Mark Bryden
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

#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/ComputeShapeVisitor.h>
#include <osgbCollision/ComputeTriMeshVisitor.h>
#include <osgbCollision/ComputeCylinderVisitor.h>
#include <osgbCollision/CollectVerticesVisitor.h>
#include <osgwTools/ForceFlattenTransforms.h>
#include <osgbCollision/Utils.h>
#include <osgwTools/Shapes.h>

#include <osg/ComputeBoundsVisitor>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/MatrixTransform>

#include <osg/Timer>
#include <osg/io_utils>
#include <iostream>


namespace osgbCollision
{


btSphereShape* btSphereCollisionShapeFromOSG( osg::Node* node )
{
    osg::ComputeBoundsVisitor cbv;
    node->accept( cbv );
    const osg::BoundingBox bb( cbv.getBoundingBox() );
    osg::Vec3 ext( bb._max - bb._min );
    ext = ext * 0.5;
    float radius = 0.;
    for( size_t i = 0; i < 3; ++i )
    {
        radius = std::max( radius, ext[ i ] );
    }
    btSphereShape* shape = new btSphereShape( radius );

    return( shape );
}

btBoxShape* btBoxCollisionShapeFromOSG( osg::Node* node, const osg::BoundingBox* bb )
{
    osg::BoundingBox bbox;
    if (bb)
        bbox = *bb;
    else
    {
        osg::ComputeBoundsVisitor visitor;
        node->accept( visitor );
        bbox = visitor.getBoundingBox();
    }

    btBoxShape* shape = new btBoxShape( btVector3( ( bbox.xMax() - bbox.xMin() ) * 0.5,
        ( bbox.yMax() - bbox.yMin() ) * 0.5, ( bbox.zMax() - bbox.zMin() ) * 0.5 ) );
    return( shape );
}

btCylinderShape* btCylinderCollisionShapeFromOSG( osg::Node* node, AXIS axis )
{
    ComputeCylinderVisitor visitor;
    switch( axis )
    {
    case X:
        visitor.setAxis( osg::X_AXIS );
        break;
    case Y:
        visitor.setAxis( osg::Y_AXIS );
        break;
    case Z:
        visitor.setAxis( osg::Z_AXIS );
        break;
    }
    node->accept( visitor );

    BoundingCylinder cyl = visitor.getBoundingCylinder();
    if( cyl.getRadius() <= 0. )
    {
        osg::notify( osg::FATAL ) << "NULL bounding cylinder." << std::endl;
        return( NULL );
    }

    btCylinderShape* shape = 0;
    switch( axis )
    {
    case X:
        shape = new btCylinderShapeX( btVector3( cyl.getLength(), cyl.getRadius(), cyl.getRadius() ) );
        break;
    case Y:
        shape = new btCylinderShape( btVector3( cyl.getRadius(), cyl.getLength(), cyl.getRadius() ) );
        break;
    case Z:
        shape = new btCylinderShapeZ( btVector3( cyl.getRadius(), cyl.getRadius(), cyl.getLength() ) );
        break;
    }
    return( shape );
}

btTriangleMeshShape* btTriMeshCollisionShapeFromOSG( osg::Node* node )
{
    ComputeTriMeshVisitor visitor;
    node->accept( visitor );

    osg::Vec3Array* vertices = visitor.getTriMesh();
    if( vertices->size() < 3 )
    {
        osg::notify( osg::WARN ) << "osgbCollision::btTriMeshCollisionShapeFromOSG, no triangles found" << std::endl;
        return( NULL );
    }

    btTriangleMesh* mesh = new btTriangleMesh;
    for( size_t i = 0; i + 2 < vertices->size(); i += 3 )
    {
        osg::Vec3& p1 = ( *vertices )[ i ];
        osg::Vec3& p2 = ( *vertices )[ i + 1 ];
        osg::Vec3& p3 = ( *vertices )[ i + 2 ];
        mesh->addTriangle( osgbCollision::asBtVector3( p1 ),
            osgbCollision::asBtVector3( p2 ), osgbCollision::asBtVector3( p3 ) );
    }

    btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape( mesh, true );
    return( meshShape );
}

btConvexTriangleMeshShape* btConvexTriMeshCollisionShapeFromOSG( osg::Node* node )
{
    ComputeTriMeshVisitor visitor;
    node->accept( visitor );

    osg::Vec3Array* vertices = visitor.getTriMesh();

    btTriangleMesh* mesh = new btTriangleMesh;
    osg::Vec3 p1, p2, p3;
    for( size_t i = 0; i + 2 < vertices->size(); i += 3 )
    {
        p1 = vertices->at( i );
        p2 = vertices->at( i + 1 );
        p3 = vertices->at( i + 2 );
        mesh->addTriangle( osgbCollision::asBtVector3( p1 ),
            osgbCollision::asBtVector3( p2 ), osgbCollision::asBtVector3( p3 ) );
    }

    btConvexTriangleMeshShape* meshShape = new btConvexTriangleMeshShape( mesh );
    return( meshShape );
}

btConvexHullShape* btConvexHullCollisionShapeFromOSG( osg::Node* node )
{
    CollectVerticesVisitor cvv;
    node->accept( cvv );
    osg::Vec3Array* v = cvv.getVertices();
    osg::notify( osg::INFO ) << "CollectVerticesVisitor: " << v->size() << std::endl;

    // Convert verts to array of Bullet scalars.
    btScalar* btverts = new btScalar[ v->size() * 3 ];
    if( btverts == NULL )
    {
        osg::notify( osg::FATAL ) << "NULL btverts" << std::endl;
        return( NULL );
    }
    btScalar* btvp = btverts;

    osg::Vec3Array::const_iterator itr;
    for( itr = v->begin(); itr != v->end(); ++itr )
    {
        const osg::Vec3& s( *itr );
        *btvp++ = (btScalar)( s[ 0 ] );
        *btvp++ = (btScalar)( s[ 1 ] );
        *btvp++ = (btScalar)( s[ 2 ] );
    }
    btConvexHullShape* chs = new btConvexHullShape( btverts,
        (int)( v->size() ), (int)( sizeof( btScalar ) * 3 ) );
    delete[] btverts;

    return( chs );
}

btCompoundShape* btCompoundShapeFromOSGGeodes( osg::Node* node,
    const BroadphaseNativeTypes shapeType, const osgbCollision::AXIS axis,
    const unsigned int reductionLevel )
{
    ComputeShapeVisitor csv( shapeType, axis, reductionLevel );
    node->accept( csv );

    btCompoundShape* cs = static_cast< btCompoundShape* >( csv.getShape() );
    return( cs );
}
btCompoundShape* btCompoundShapeFromOSGGeometry( osg::Node* node )
{
    osg::notify( osg::WARN ) << "btCompoundShapeFromOSGGeometry: This function is not currently implemented." << std::endl;
    throw( std::string( "btCompoundShapeFromOSGGeometry not implemented" ) );
    return( NULL );
}

btCompoundShape* btCompoundShapeFromBounds( osg::Node* node,
    const BroadphaseNativeTypes shapeType, const osgbCollision::AXIS axis )
{
    btCollisionShape* shape( NULL );
    switch( shapeType )
    {
    case BOX_SHAPE_PROXYTYPE:
        shape = btBoxCollisionShapeFromOSG( node );
        break;
    case SPHERE_SHAPE_PROXYTYPE:
        shape = btSphereCollisionShapeFromOSG( node );
        break;
    case CYLINDER_SHAPE_PROXYTYPE:
        shape = btCylinderCollisionShapeFromOSG( node, axis );
        break;
    default:
        osg::notify( osg::WARN ) << "btCompoundShapeFromBounds: Unsupported shapeType: " << (int)shapeType << std::endl;
        break;
    }

    osg::ComputeBoundsVisitor cbv;
    node->accept( cbv );
    btVector3 center( osgbCollision::asBtVector3( cbv.getBoundingBox().center() ) );

    btTransform wt; wt.setIdentity();
    wt.setOrigin( center );

    btCompoundShape* xformShape = new btCompoundShape;
    xformShape->addChildShape( wt, shape );
    return( xformShape );
}




osg::Node* osgNodeFromBtCollisionShape( const btCollisionShape* btShape, const btTransform& trans )
{
    if( btShape->getShapeType() == BOX_SHAPE_PROXYTYPE )
    {
        const btBoxShape* btBox = static_cast< const btBoxShape* >( btShape );
        return( osgNodeFromBtCollisionShape( btBox, trans ) );
    }
    else if( btShape->getShapeType() == SPHERE_SHAPE_PROXYTYPE )
    {
        const btSphereShape* btSphere = static_cast< const btSphereShape* >( btShape );
        return( osgNodeFromBtCollisionShape( btSphere, trans ) );
    }
    else if( btShape->getShapeType() == CYLINDER_SHAPE_PROXYTYPE )
    {
        const btCylinderShape* btCylinder = static_cast< const btCylinderShape* >( btShape );
        return( osgNodeFromBtCollisionShape( btCylinder, trans ) );
    }
    else if( btShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE )
    {
        const btBvhTriangleMeshShape* btTriMesh = static_cast< const btBvhTriangleMeshShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( osgNodeFromBtCollisionShape( btTriMesh ) );
    }
    else if( btShape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE )
    {
        const btConvexTriangleMeshShape* btConvexTriMesh = static_cast< const btConvexTriangleMeshShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( osgNodeFromBtCollisionShape( btConvexTriMesh ) );
    }
    else if( btShape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE )
    {
        const btConvexHullShape* convexHull = static_cast< const btConvexHullShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( osgNodeFromBtCollisionShape( convexHull ) );
    }
    else if( btShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE )
    {
        const btCompoundShape* masterShape = static_cast< const btCompoundShape* >( btShape );
        osg::Group* grp = new osg::Group;
        int idx;
        for (idx=0; idx< masterShape->getNumChildShapes(); idx++)
        {
            const btCollisionShape* s = masterShape->getChildShape( idx );
            const btTransform t = masterShape->getChildTransform( idx );
            const btTransform accumTrans = trans * t;
            grp->addChild( osgNodeFromBtCollisionShape( s, accumTrans ) );
        }
        return( grp );
    }
    else
    {
        osg::notify( osg::WARN ) << "osgNodeFromBtCollisionShape: Unsupported shape type: " <<
        btShape->getShapeType() << std::endl;
        return( NULL );
    }
}

osg::Node* osgNodeFromBtCollisionShape( const btBoxShape* btBox, const btTransform& trans )
{
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( osgGeometryFromBtCollisionShape( btBox ) );

    osg::Matrix m = asOsgMatrix( trans );
    if (m.isIdentity())
        return( geode );
    else
    {
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix( m );
        mt->addChild( geode );
        return mt;
    }
}

osg::Node* osgNodeFromBtCollisionShape( const btSphereShape* btSphere, const btTransform& trans )
{
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( osgGeometryFromBtCollisionShape( btSphere ) );

    osg::Matrix m = asOsgMatrix( trans );
    if (m.isIdentity())
        return( geode );
    else
    {
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix( m );
        mt->addChild( geode );
        return mt;
    }
}

osg::Node* osgNodeFromBtCollisionShape( const btCylinderShape * btCylinder, const btTransform& trans )
{
    osg::Cylinder* cylinder = new osg::Cylinder();
    cylinder->setRadius( btCylinder->getRadius() );

    switch( btCylinder->getUpAxis() )
    {
        case X:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getX() );
            cylinder->setRotation( osg::Quat( osg::PI_2, osg::Vec3( 0, 1, 0 ) ) );
            break;
        case Y:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getY() );
            cylinder->setRotation( osg::Quat( osg::PI_2, osg::Vec3( 1, 0, 0 ) ) );
            break;
        case Z:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getZ() );
    }

    osg::TessellationHints* hints = new osg::TessellationHints();
    hints->setDetailRatio( .2f );

    osg::ShapeDrawable* shape = new osg::ShapeDrawable( cylinder, hints );
    shape->setColor( osg::Vec4( 1., 1., 1., 1. ) );
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( shape );

    osg::Matrix m = asOsgMatrix( trans );
    if (m.isIdentity())
        return( geode );
    else
    {
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix( m );
        mt->addChild( geode );
        return mt;
    }
}

osg::Node* osgNodeFromBtCollisionShape( const btTriangleMeshShape* btTriMesh, const btTransform& trans )
{
    const btTriangleMesh* mesh = dynamic_cast< const btTriangleMesh* >( btTriMesh->getMeshInterface() );
    if( !mesh )
    {
        osg::notify( osg::FATAL ) << "osgNodeFromBtCollisionShape: No triangle mesh." << std::endl;
        return( NULL );
    }

    btVector3* verts;
    int* indices;
    int numVerts;
    int numFaces;
    PHY_ScalarType vt, ft;
    int vs, fs;

    mesh->getLockedReadOnlyVertexIndexBase( ( const unsigned char** )&verts, numVerts, vt, vs, ( const unsigned char** )&indices, fs, numFaces, ft );

    osg::Vec3Array* vec = new osg::Vec3Array();
    vec->resize( numVerts );
    int idx;
    for( idx = 0; idx < numVerts; idx++ )
    {
        const btVector3& bulletVert = verts[ idx ];
        ( *vec )[ idx ].set( bulletVert.getX(), bulletVert.getY(), bulletVert.getZ() );
    }

    osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );
    for( idx = 0; idx < numFaces * 3; idx++ )
        deui->push_back( indices[ idx ] );

    osg::Vec4Array* color = new osg::Vec4Array();
    color->push_back( osg::Vec4( 1., 1., 1., 1. ) );

    osg::Geometry* geom = new osg::Geometry;
    geom->setVertexArray( vec );
    geom->setColorArray( color );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    geom->addPrimitiveSet( deui );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geom );

    osg::Matrix m = asOsgMatrix( trans );
    if (m.isIdentity())
        return( geode );
    else
    {
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix( m );
        mt->addChild( geode );
        return mt;
    }
}

osg::Node* osgNodeFromBtCollisionShape( const btConvexTriangleMeshShape* btTriMesh, const btTransform& trans )
{
    const btTriangleMesh* mesh = dynamic_cast< const btTriangleMesh* >( btTriMesh->getMeshInterface() );
    if( !mesh )
    {
        osg::notify( osg::FATAL ) << "osgNodeFromBtCollisionShape: No triangle mesh." << std::endl;
        return( NULL );
    }

    btVector3* verts;
    int* indices;
    int numVerts;
    int numFaces;
    PHY_ScalarType vt, ft;
    int vs, fs;

    mesh->getLockedReadOnlyVertexIndexBase( ( const unsigned char** )&verts, numVerts, vt, vs, ( const unsigned char** )&indices, fs, numFaces, ft );

    osg::Vec3Array* vec = new osg::Vec3Array();
    vec->resize( numVerts );
    int idx;
    for( idx = 0; idx < numVerts; idx++ )
    {
        const btVector3& bulletVert = verts[ idx ];
        ( *vec )[ idx ].set( bulletVert.getX(), bulletVert.getY(), bulletVert.getZ() );
    }

    osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );
    for( idx = 0; idx < numFaces * 3; idx++ )
        deui->push_back( indices[ idx ] );

    osg::Vec4Array* color = new osg::Vec4Array();
    color->push_back( osg::Vec4( 1., 1., 1., 1. ) );

    osg::Geometry* geom = new osg::Geometry;
    geom->setVertexArray( vec );
    geom->setColorArray( color );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    geom->addPrimitiveSet( deui );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geom );

    osg::Matrix m = asOsgMatrix( trans );
    if (m.isIdentity())
        return( geode );
    else
    {
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix( m );
        mt->addChild( geode );
        return mt;
    }
}

osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans )
{
    btShapeHull sh( hull );
    sh.buildHull( btScalar( 0. ) );
	int nVerts( sh.numVertices () );
	int nIdx( sh.numIndices() );
    if( (nVerts <= 0) || (nIdx <= 0) )
        return( NULL );

    const btVector3* bVerts( sh.getVertexPointer() );
    const unsigned int* bIdx( sh.getIndexPointer() );

    osg::Vec3Array* v = new osg::Vec3Array();
    v->resize( nVerts );
    unsigned int idx;
    for( idx = 0; idx < (unsigned int)nVerts; idx++ )
        ( *v )[ idx ] = asOsgVec3( bVerts[ idx ] );

    osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );
    for( idx = 0; idx < (unsigned int)nIdx; idx++ )
        deui->push_back( bIdx[ idx ] );

    osg::Vec4Array* color = new osg::Vec4Array();
    color->push_back( osg::Vec4( 1., 1., 1., 1. ) );

    osg::Geometry* geom = new osg::Geometry;
    geom->setVertexArray( v );
    geom->setColorArray( color );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    geom->addPrimitiveSet( deui );

    osg::ref_ptr< osg::Geode > geode = new osg::Geode();
    geode->addDrawable( geom );

    osg::Matrix m = asOsgMatrix( trans );
    if (m.isIdentity())
        return( geode.release() );
    else
    {
        osg::ref_ptr< osg::MatrixTransform > mt = new osg::MatrixTransform;
        mt->setMatrix( m );
        mt->addChild( geode.get() );
        return mt.release();
    }
}


osg::Geometry* osgGeometryFromBtCollisionShape( const btBoxShape* btBox )
{
    return( osgwTools::makeBox( osgbCollision::asOsgVec3( btBox->getHalfExtentsWithMargin() ) ) );
}

osg::Geometry* osgGeometryFromBtCollisionShape( const btSphereShape* btSphere )
{
    return( osgwTools::makeAltAzSphere( btSphere->getRadius() ) );
}

osg::Geometry* osgGeometryFromBtCollisionShape( const btCylinderShape* btCylinder )
{
    const osg::Vec3 defaultOrientation( 0., 0., 1. );
    osg::Matrix m;
    double length;
    const btVector3 halfExtents( btCylinder->getHalfExtentsWithMargin() );
    switch( btCylinder->getUpAxis() )
    {
        case X:
            m = osg::Matrix::rotate( defaultOrientation, osg::Vec3( 1., 0., 0. ) );
            length = halfExtents.getX();
            break;
        case Y:
            m = osg::Matrix::rotate( defaultOrientation, osg::Vec3( 0., 1., 0. ) );
            length = halfExtents.getY();
            break;
        case Z:
            // Leave m set to the identity matrix.
            length = halfExtents.getZ();
            break;
    }
    const double radius( btCylinder->getRadius() );

    return( osgwTools::makeOpenCylinder( m, length, radius, radius ) );
}


// osgbCollision
}
