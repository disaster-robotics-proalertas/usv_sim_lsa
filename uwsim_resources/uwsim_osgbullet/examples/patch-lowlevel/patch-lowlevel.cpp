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

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osg/Geode>
#include <osg/LightModel>
#include <osg/Texture2D>
#include <osgUtil/SmoothingVisitor>

#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/LaunchHandler.h>
#include <osgbInteraction/SaveRestoreHandler.h>

#include <osgwTools/Shapes.h>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBody.h>

#include <osg/io_utils>
#include <string>


btSoftBodyWorldInfo	worldInfo;


/** \cond */
struct MeshUpdater : public osg::Drawable::UpdateCallback
{
    MeshUpdater( const btSoftBody* softBody, const unsigned int size )
      : _softBody( softBody ),
        _size( size )
    {}
    virtual ~MeshUpdater()
    {}

    virtual void update( osg::NodeVisitor*, osg::Drawable* draw )
    {
        osg::Geometry* geom( draw->asGeometry() );
        osg::Vec3Array* verts( dynamic_cast< osg::Vec3Array* >( geom->getVertexArray() ) );

        // Update the vertex array from the soft body node array.
        const btSoftBody::tNodeArray& nodes = _softBody->m_nodes;
        osg::Vec3Array::iterator it( verts->begin() );
        unsigned int idx;
        for( idx=0; idx<_size; idx++)
        {
            *it++ = osgbCollision::asOsgVec3( nodes[ idx ].m_x );
        }
        verts->dirty();
        draw->dirtyBound();

        // Generate new normals.
        osgUtil::SmoothingVisitor smooth;
        smooth.smooth( *geom );
        geom->getNormalArray()->dirty();
    }

    const btSoftBody* _softBody;
    const unsigned int _size;
};
/** \endcond */

osg::Node* makeFlag( btSoftRigidDynamicsWorld* bw )
{
    const short resX( 12 ), resY( 9 );

    osg::ref_ptr< osg::Geode > geode( new osg::Geode );

    const osg::Vec3 llCorner( -2., 0., 5. );
    const osg::Vec3 uVec( 4., 0., 0. );
    const osg::Vec3 vVec( 0., 0.1, 3. ); // Must be at a slight angle for wind to catch it.
    osg::Geometry* geom = osgwTools::makePlane( llCorner,
        uVec, vVec, osg::Vec2s( resX-1, resY-1 ) );
    geode->addDrawable( geom );

    // Set up for dynamic data buffer objects
    geom->setDataVariance( osg::Object::DYNAMIC );
    geom->setUseDisplayList( false );
    geom->setUseVertexBufferObjects( true );
    geom->getOrCreateVertexBufferObject()->setUsage( GL_DYNAMIC_DRAW );

    // Flag state: 2-sided lighting and a texture map.
    {
        osg::StateSet* stateSet( geom->getOrCreateStateSet() );

        osg::LightModel* lm( new osg::LightModel() );
        lm->setTwoSided( true );
        stateSet->setAttributeAndModes( lm );

        const std::string texName( "fort_mchenry_flag.jpg" );
        osg::Texture2D* tex( new osg::Texture2D(
            osgDB::readImageFile( texName ) ) );
        if( ( tex == NULL ) || ( tex->getImage() == NULL ) )
            osg::notify( osg::WARN ) << "Unable to read texture: \"" << texName << "\"." << std::endl;
        else
        {
            tex->setResizeNonPowerOfTwoHint( false );
            stateSet->setTextureAttributeAndModes( 0, tex );
        }
    }


    // Create the soft body using a Bullet helper function. Note that
    // our update callback will update vertex data from the soft body
    // node data, so it's important that the corners and resolution
    // parameters produce node data that matches the vertex data.
    btSoftBody* softBody( btSoftBodyHelpers::CreatePatch( worldInfo,
        osgbCollision::asBtVector3( llCorner ),
        osgbCollision::asBtVector3( llCorner + uVec ),
        osgbCollision::asBtVector3( llCorner + vVec ),
        osgbCollision::asBtVector3( llCorner + uVec + vVec ),
        resX, resY, 1+4, true ) );


    // Configure the soft body for interaction with the wind.
    softBody->getCollisionShape()->setMargin( 0.1 );
    softBody->m_materials[ 0 ]->m_kLST = 0.3;
    softBody->generateBendingConstraints( 2, softBody->m_materials[ 0 ] );
    softBody->m_cfg.kLF = 0.05;
    softBody->m_cfg.kDG = 0.01;
    softBody->m_cfg.piterations = 2;
#if( BT_BULLET_VERSION >= 279 )
    softBody->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
#else
    // Hm. Not sure how to make the wind blow on older Bullet.
    // This doesn't seem to work.
    softBody->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSided;
#endif
    softBody->setWindVelocity( btVector3( 50., 0., 0. ) );
    softBody->setTotalMass( 1. );

    bw->addSoftBody( softBody );
    geom->setUpdateCallback( new MeshUpdater( softBody, resX*resY ) );

    return( geode.release() );
}

btSoftRigidDynamicsWorld* initPhysics()
{
    btSoftBodyRigidBodyCollisionConfiguration* collision = new btSoftBodyRigidBodyCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collision );
    worldInfo.m_dispatcher = dispatcher;
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* broadphase = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );
    worldInfo.m_broadphase = broadphase;

    btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld( dispatcher, broadphase, solver, collision );

    btVector3 gravity( 0, 0, -32.17 );
    dynamicsWorld->setGravity( gravity );
    worldInfo.m_gravity = gravity;

    worldInfo.air_density = btScalar( 1.2 );
    worldInfo.water_density = 0;
    worldInfo.water_offset = 0;
    worldInfo.water_normal = btVector3( 0, 0, 0 );
    worldInfo.m_sparsesdf.Initialize();

    return( dynamicsWorld );
}


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    btSoftRigidDynamicsWorld* bw = initPhysics();
    osg::Group* root = new osg::Group;

    osg::Group* launchHandlerAttachPoint = new osg::Group;
    root->addChild( launchHandlerAttachPoint );


    osg::ref_ptr< osg::Node > rootModel( makeFlag( bw ) );
    if( !rootModel.valid() )
    {
        osg::notify( osg::FATAL ) << "mesh: Can't create flag mesh." << std::endl;
        return( 1 );
    }

    root->addChild( rootModel.get() );


    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new
        osgbInteraction::SaveRestoreHandler;

    // Add ground
    const osg::Vec4 plane( 0., 0., 1., 0. );
    root->addChild( osgbDynamics::generateGroundPlane( plane, bw ) );



    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bw->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.addEventHandler( new osgViewer::StatsHandler() );
    //viewer.setUpViewInWindow( 30, 30, 768, 480 );
    viewer.setSceneData( root );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -16., 6. ), osg::Vec3( 0., 0., 5. ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );
    viewer.realize();

    // Create the launch handler.
    osgbInteraction::LaunchHandler* lh = new osgbInteraction::LaunchHandler(
        bw, launchHandlerAttachPoint, viewer.getCamera() );
    {
        // Use a custom launch model: Sphere with radius 0.2 (instead of default 1.0).
        osg::Geode* geode = new osg::Geode;
        const double radius( .2 );
        geode->addDrawable( osgwTools::makeGeodesicSphere( radius ) );
        lh->setLaunchModel( geode, new btSphereShape( radius ) );
        lh->setInitialVelocity( 40. );

        viewer.addEventHandler( lh );
    }

    srh->setLaunchHandler( lh );
    srh->capture();
    viewer.addEventHandler( srh.get() );
    viewer.addEventHandler( new osgbInteraction::DragHandler(
        bw, viewer.getCamera() ) );


    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        if( dbgDraw != NULL )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bw->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        if( dbgDraw != NULL )
        {
            bw->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        worldInfo.m_sparsesdf.GarbageCollect();

        viewer.frame();
    }

    return( 0 );
}
