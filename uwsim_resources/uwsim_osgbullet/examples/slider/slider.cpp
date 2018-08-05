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
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgUtil/Optimizer>
#include <osg/ComputeBoundsVisitor>

#include <osg/Light>
#include <osg/LightSource>
#include <osg/Material>
#include <osg/LightModel>
#include <osgShadow/ShadowedScene>
#include <osgShadow/StandardShadowMap>

#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/LaunchHandler.h>
#include <osgbInteraction/SaveRestoreHandler.h>

#include <osgwTools/InsertRemove.h>
#include <osgwTools/FindNamedNode.h>
#include <osgwTools/Version.h>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <map>



/** \cond */

// Derive a class from StandardShadowMap so we have better control over
// the direction of the depth map generation Camera.
class ControlledShadowMap : public osgShadow::StandardShadowMap
{
public:
    ControlledShadowMap()
      : osgShadow::StandardShadowMap()
    {
        // Improve on the defaults.
        _textureSize = osg::Vec2s( 2048, 2048 ),
        // StandardShadowMap doesn't appear to provide accessors for these values...
        _polygonOffsetFactor = 2.f;
        _polygonOffsetUnits = 2.f;
    }

protected:
    struct ViewData : public osgShadow::StandardShadowMap::ViewData
    {
        virtual void aimShadowCastingCamera( 
            const osg::BoundingSphere& bounds,
            const osg::Light* light,
            const osg::Vec4& worldLightPos,
            const osg::Vec3& worldLightDir,                                       
            const osg::Vec3& worldLightUp = osg::Vec3( 0, 1,0 ) )
        {
            // For out case, we have a point light source, but we always want
            // the center of the depth map at the center of the scene. We don't
            // care of objects away from the center don't cast a shadow. So we
            // override aimShadowCastingCamera() to compute the depth map generation
            // Camera the way we want it.

            osg::Matrixd& view = _camera->getViewMatrix();
            osg::Matrixd& projection = _camera->getProjectionMatrix();

            osg::Vec3 up = worldLightUp;
            if( up.length2() <= 0 )
                up.set( 0, 1, 0 );

            osg::Vec3 position( worldLightPos.x(), worldLightPos.y(), worldLightPos.z() );
            view.makeLookAt( position, osg::Vec3( 0., 0., 0. ), up );

            const double distance( ( bounds.center() - position ).length() );
            const double zFar = distance + bounds.radius();
            const double zNear = osg::maximum< double >( (zFar * 0.001), (distance-bounds.radius()) );
            const float fovy = 120.f;
            projection.makePerspective( fovy, 1.0, zNear, zFar );
            _camera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
        }
    };

    friend struct ViewData;

    META_ViewDependentShadowTechniqueData( ControlledShadowMap, ControlledShadowMap::ViewData )
};

/** \endcond */


// Filter out collisions between the drawer and nightstand.
//
// Bullet collision filtering tutorial:
//   http://www.bulletphysics.com/mediawiki-1.5.8/index.php?title=Collision_Filtering
//
// Define filter groups
enum CollisionTypes {
    COL_DRAWER = 0x1 << 0,
    COL_STAND = 0x1 << 1,
    COL_DEFAULT = 0x1 << 2,
};
// Define filter masks
unsigned int drawerCollidesWith( COL_DEFAULT );
unsigned int standCollidesWith( COL_DEFAULT );
unsigned int defaultCollidesWith( COL_DRAWER | COL_STAND | COL_DEFAULT );



btRigidBody* standBody;
void makeStaticObject( btDiscreteDynamicsWorld* bw, osg::Node* node, const osg::Matrix& m )
{
    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = node;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = 0.f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody( rb, COL_STAND, standCollidesWith );

    // Save RB in global
    standBody = rb;
}

btRigidBody* drawerBody;
osg::Transform* makeDrawer( btDiscreteDynamicsWorld* bw, osgbInteraction::SaveRestoreHandler* srh, osg::Node* node, const osg::Matrix& m )
{
    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    osgwTools::insertAbove( node, amt );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->setCenterOfMass( node->getBound().center() );
    cr->_parentTransform = m;
    cr->_mass = .75f;
    cr->_restitution = .5f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody( rb, COL_DRAWER, drawerCollidesWith );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Save RB in global, as AMT UserData (for DragHandler), and in SaveRestoreHandler.
    drawerBody = rb;
    amt->setUserData( new osgbCollision::RefRigidBody( rb ) );
    srh->add( "gate", rb );

    return( amt );
}


btDiscreteDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.81 ) );

    return( dynamicsWorld );
}


osg::Node* findNamedNode( osg::Node* model, const std::string& name, osg::Matrix& xform )
{
    osgwTools::FindNamedNode fnn( name );
    model->accept( fnn );
    if( fnn._napl.empty() )
    {
        osg::notify( osg::FATAL ) << "hinge: Can't find node names \"" << name << "\"." << std::endl;
        return( NULL );
    }
    xform = osg::computeLocalToWorld( fnn._napl[ 0 ].second );
    return( fnn._napl[ 0 ].first );
}

void simpleLighting( osg::Group* root )
{
    osg::StateSet* rootState = root->getOrCreateStateSet();
    rootState->setMode( GL_LIGHT0, osg::StateAttribute::ON );

    osg::LightSource* ls = new osg::LightSource();
    ls->setReferenceFrame( osg::LightSource::RELATIVE_RF );
    root->addChild( ls );

    osg::Light* light = new osg::Light;
    light->setLightNum( 0 );
    light->setAmbient( osg::Vec4( 1., 1., 1., 1. ) );
    light->setDiffuse( osg::Vec4( 1., 1., 1., 1. ) );
    light->setSpecular( osg::Vec4( 1., 1., 1., 1. ) );

    osg::Vec3 pos( -.5, -.4, 2. );
    light->setPosition( osg::Vec4( pos, 1. ) );
    ls->setLight( light );

    osg::LightModel* lm = new osg::LightModel;
    lm->setAmbientIntensity( osg::Vec4( 0., 0., 0., 1. ) );
    lm->setColorControl( osg::LightModel::SEPARATE_SPECULAR_COLOR );
    rootState->setAttribute( lm, osg::StateAttribute::ON );
}

void nightstandMaterial( osg::Node* root )
{
    osg::StateSet* rootState = root->getOrCreateStateSet();

    osg::Material* mat = new osg::Material;
    mat->setAmbient( osg::Material::FRONT, osg::Vec4( .1, .1, .1, 1. ) );
    mat->setDiffuse( osg::Material::FRONT, osg::Vec4( 1., 1., 1., 1. ) );
    mat->setSpecular( osg::Material::FRONT, osg::Vec4( 0.6, 0.6, 0.5, 1. ) );
    mat->setShininess( osg::Material::FRONT, 16.f );
    rootState->setAttribute( mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
}

void groundMaterial( osg::Node* root )
{
    osg::StateSet* rootState = root->getOrCreateStateSet();

    osg::Material* mat = new osg::Material;
    mat->setAmbient( osg::Material::FRONT, osg::Vec4( .1, .1, .1, 1. ) );
    mat->setDiffuse( osg::Material::FRONT, osg::Vec4( .75, .75, .75, 1. ) );
    mat->setSpecular( osg::Material::FRONT, osg::Vec4( 0., 0., 0., 1. ) );
    rootState->setAttribute( mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
}

void launchMaterial( osg::Node* root )
{
    osg::StateSet* rootState = root->getOrCreateStateSet();

    osg::Material* mat = new osg::Material;
    mat->setAmbient( osg::Material::FRONT, osg::Vec4( .2, .2, .3, 1. ) );
    mat->setDiffuse( osg::Material::FRONT, osg::Vec4( .4, .4, .5, 1. ) );
    mat->setSpecular( osg::Material::FRONT, osg::Vec4( .4, .4, .4, 1. ) );
    mat->setShininess( osg::Material::FRONT, 30.f );
    rootState->setAttribute( mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
}

#define SHADOW_CASTER 0x1
#define SHADOW_RECEIVER 0x2
#define SHADOW_BOTH (SHADOW_CASTER|SHADOW_RECEIVER)

int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::Group* root = new osg::Group;

    simpleLighting( root );

    osg::Group* launchHandlerAttachPoint = new osg::Group;
    launchHandlerAttachPoint->setNodeMask( SHADOW_BOTH );
    root->addChild( launchHandlerAttachPoint );
    launchMaterial( launchHandlerAttachPoint );


    osg::ref_ptr< osg::Node > rootModel = osgDB::readNodeFile( "NightStand.flt" );
    if( !rootModel.valid() )
    {
        osg::notify( osg::FATAL ) << "hinge: Can't load data file \"NightStand.flt\"." << std::endl;
        return( 1 );
    }
    rootModel->setNodeMask( SHADOW_BOTH );
    nightstandMaterial( rootModel.get() );

    root->addChild( rootModel.get() );

    // Get Node pointers and parent transforms for the night stand and drawer.
    // (Node names are taken from the osgWorks osgwnames utility.)
    osg::Matrix standXform, drawerXform;
    osg::Node* standNode = findNamedNode( rootModel.get(), "NightStand_Body", standXform );
    osg::Node* drawerNode = findNamedNode( rootModel.get(), "DOF_Drawer", drawerXform );
    if( ( standNode == NULL ) || ( drawerNode == NULL ) )
        return( 1 );

    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new
        osgbInteraction::SaveRestoreHandler;

    // Make Bullet rigid bodies and collision shapes for the drawer...
    makeDrawer( bulletWorld, srh.get(), drawerNode, drawerXform );
    // ...and the stand.
    makeStaticObject( bulletWorld, standNode, standXform );


    // Add ground
    const osg::Vec4 plane( 0., 0., 1., 0. );
    osg::Node* groundRoot = osgbDynamics::generateGroundPlane( plane,
        bulletWorld, NULL, COL_DEFAULT, defaultCollidesWith );
    groundRoot->setNodeMask( SHADOW_RECEIVER );
    groundMaterial( groundRoot );
    root->addChild( groundRoot );
    

    // create slider constraint between drawer and stand, and add it to world.
    // Note: Bullet slider is always along x axis. Alter this behavior with reference frames.
    btSliderConstraint* slider;
    float drawerMinLimit;
    btVector3 startPos;
    {
        // Model-specific constants.
        // TBD Should obtain these from model metadata or user input:
        const osg::Vec3 drawerAxis( 0., 1., 0. );
        const float drawerMaxLimit( 0.f );

        osg::ComputeBoundsVisitor cbv;
        drawerNode->accept( cbv );
        const osg::BoundingBox& bb = cbv.getBoundingBox();
        drawerMinLimit = -( bb.yMax() - bb.yMin() );

        // Compute a matrix that transforms the stand's collision shape origin and x axis
        // to the drawer's origin and drawerAxis.
        //   1. Matrix to align the (slider constraint) x axis with the drawer axis.
        const osg::Vec3 bulletSliderAxis( 1., 0., 0. );
        const osg::Matrix axisRotate( osg::Matrix::rotate( bulletSliderAxis, drawerAxis ) );
        //
        //   2. Inverse stand center of mass offset.
        osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( standBody->getMotionState() );
        const osg::Matrix invStandCOM( osg::Matrix::translate( -( motion->getCenterOfMass() ) ) );
        //
        //   3. Transform from the stand's origin to the drawer's origin.
        const osg::Matrix standToDrawer( osg::Matrix::inverse( standXform ) * drawerXform );
        //
        //   4. The final stand frame matrix.
        btTransform standFrame = osgbCollision::asBtTransform(
            axisRotate * invStandCOM * standToDrawer );


        // Compute a matrix that transforms the drawer's collision shape origin and x axis
        // to the drawer's origin and drawerAxis.
        //   1. Drawer center of mass offset.
        motion = dynamic_cast< osgbDynamics::MotionState* >( drawerBody->getMotionState() );
        const osg::Matrix invDrawerCOM( osg::Matrix::translate( -( motion->getCenterOfMass() ) ) );
        //
        //   2. The final drawer frame matrix.
        btTransform drawerFrame = osgbCollision::asBtTransform(
            axisRotate * invDrawerCOM );


        slider = new btSliderConstraint( *drawerBody, *standBody, drawerFrame, standFrame, false );
        slider->setLowerLinLimit( drawerMinLimit );
	    slider->setUpperLinLimit( drawerMaxLimit );
        bulletWorld->addConstraint( slider, true );

        btTransform m = drawerBody->getWorldTransform();
        startPos = m.getOrigin();
    }


    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        osg::Node* dbgRoot = dbgDraw->getSceneGraph();
        dbgRoot->setNodeMask( ~SHADOW_BOTH );
        root->addChild( dbgRoot );
    }


    osgShadow::ShadowedScene* sceneRoot = new osgShadow::ShadowedScene;
    sceneRoot->setCastsShadowTraversalMask( SHADOW_CASTER );
    sceneRoot->setReceivesShadowTraversalMask( SHADOW_RECEIVER );
    sceneRoot->addChild( root );
    {
        ControlledShadowMap* sTex = new ControlledShadowMap;

        // Workaround the fact that OSG StandardShadowMap fragment shader
        // doesn't add in the specular color.
#if( OSGWORKS_OSG_VERSION < 20912 )
        // Prior to Feb 23 2011 (2.9.12 release), StandardShadowMap used vertex
        // shaders that employed a varying to convey ambient color.
        std::string shaderName( "ShadowMap-Main.fs" );
#else
        // From 2.9.12 onward, StandardShadowMap uses only a fragment shader,
        // so ambient color is conveyed in the FFP built-in varying.
        std::string shaderName( "ShadowMap-Main-3x.fs" );
#endif
        osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT );
        shader->setName( shaderName );
        shader->loadShaderSourceFromFile( osgDB::findDataFile( shaderName ) );
        if( shader->getShaderSource().empty() )
            osg::notify( osg::WARN ) << "Warning: Unable to load shader file: \"" << shaderName << "\"." << std::endl;
        else
            sTex->setMainFragmentShader( shader );

        sceneRoot->setShadowTechnique( sTex );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 30, 30, 800, 450 );
    viewer.setSceneData( sceneRoot );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( .8, -5., 1.6 ), osg::Vec3( 0., 0., .5 ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    // Create the launch handler.
    osgbInteraction::LaunchHandler* lh = new osgbInteraction::LaunchHandler(
        bulletWorld, launchHandlerAttachPoint, viewer.getCamera() );
    {
        // Use a custom launch model: A scaled-down teapot.
        osg::ref_ptr< osg::MatrixTransform > mt = new osg::MatrixTransform(
            osg::Matrix::scale( 0.2, 0.2, 0.2 ) );
        mt->addChild( osgDB::readNodeFile( "teapot.osg" ) );
        osgUtil::Optimizer opt;
        opt.optimize( mt.get(), osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS );
        mt->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );

        lh->setLaunchModel( mt.get() );
        lh->setInitialVelocity( 10. );

        // Also add the proper collision masks
        lh->setCollisionFilters( COL_DEFAULT, defaultCollidesWith );

        viewer.addEventHandler( lh );
    }

    srh->setLaunchHandler( lh );
    srh->capture();
    viewer.addEventHandler( srh.get() );
    viewer.addEventHandler( new osgbInteraction::DragHandler(
        bulletWorld, viewer.getCamera() ) );

    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        if( dbgDraw != NULL )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        if( dbgDraw != NULL )
        {
            bulletWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();

        if( slider != NULL )
        {
            btTransform m = drawerBody->getWorldTransform();
            btVector3 v = m.getOrigin();
            if( ( v[ 1 ] - startPos[ 1 ] ) < ( drawerMinLimit * 1.02 ) )
            {
                bulletWorld->removeConstraint( slider );
                delete slider;
                slider = NULL;
                // Advice from Bullet forum is that the correct way to change the collision filters
                // is to remove the body and add it back:
                // http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=7538
                bulletWorld->removeRigidBody( drawerBody );
                bulletWorld->addRigidBody( drawerBody, COL_DEFAULT, defaultCollidesWith );
            }
        }
    }

    return( 0 );
}


/** \page sliderlowlevel Simple Slider Constraint

Demonstrates coding directly to the Bullet API to create a slider constraint.

Use the --debug command line option to enable debug collision object display.

\section slidercontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.
\li shift-leftmouse: Launches a sphere into the scene.

*/
