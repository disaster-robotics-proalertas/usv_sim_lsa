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

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgwTools/AbsoluteModelTransform.h>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <btBulletDynamicsCommon.h>

#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Version.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/PhysicsState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/Utils.h>

#include <osgbInteraction/SaveRestoreHandler.h>
#include <osgbInteraction/DragHandler.h>
#include <osgGA/TrackballManipulator>

#include <osg/io_utils>
#include <iostream>
#include <sstream>



btDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ));

    return( dynamicsWorld );
}

osg::Transform* createOSGBox( osg::Vec3 size )
{
    osg::Box* box = new osg::Box();
    box->setHalfLengths( size );

    osg::ShapeDrawable* shape = new osg::ShapeDrawable( box );
    shape->setColor( osg::Vec4( 1., 1., 1., 1. ) );
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( shape );

    osg::MatrixTransform* mt = new osg::MatrixTransform();
    mt->addChild( geode );

    return( mt );
}

osg::Node*
createGround( float w, float h, const osg::Vec3& center, btDynamicsWorld* dw )
{
    osg::Transform* ground = createOSGBox( osg::Vec3( w, h, .05 ) );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = ground;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = 0.f;
    btRigidBody* body = osgbDynamics::createRigidBody( cr.get(),
        osgbCollision::btBoxCollisionShapeFromOSG( ground ) );

    // Transform the box explicitly.
    osgbDynamics::MotionState* motion = static_cast< osgbDynamics::MotionState* >( body->getMotionState() );
    osg::Matrix m( osg::Matrix::translate( center ) );
    motion->setParentTransform( m );
    body->setWorldTransform( osgbCollision::asBtTransform( m ) );

    dw->addRigidBody( body );

    return( ground );
}

int main( int argc, char* argv[] )
{
    osg::ArgumentParser arguments( &argc, argv );

    const std::string appName = osgDB::getSimpleFileName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setApplicationName( appName );
    arguments.getApplicationUsage()->setDescription( appName + " creates a rigid body from model files and tests it in a physics simulation." );
    arguments.getApplicationUsage()->setCommandLineUsage( appName + " [options] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "--com <x>,<y>,<z>", "Specifies the center of mass. If not specified, osgbpp uses the center of the OSG bounding sphere." );
    arguments.getApplicationUsage()->addCommandLineOption( "--box", "This is the default. Creates a box collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--sphere", "Creates a sphere collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--cylinder", "Creates a cylinder collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--axis <a>", "Use this option to specify the cylinder axis X, Y, or Z. Default is Z. This argument is ignored if --cylinder is not specified." );
    arguments.getApplicationUsage()->addCommandLineOption( "--triMesh", "It creates a tri mesh collision shape (suitable for static objects)." );
    arguments.getApplicationUsage()->addCommandLineOption( "--convexTM", "Creates a convex tri mesh collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--convexHull", "Creates a convex hull collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--mass <n>", "Specifies the desired rigid body mass value. The default is 1.0." );
    arguments.getApplicationUsage()->addCommandLineOption( "--debug", "Use the GLDebugDrawer class to display collision shapes." );

    arguments.getApplicationUsage()->addCommandLineOption( "--overall", "Creates a single collision shape for the entire input scene graph, rather than a collision shape per Geode, which is the default." );

    arguments.getApplicationUsage()->addCommandLineOption( "-h or --help", "Displays help text and command line documentation." );
    arguments.getApplicationUsage()->addCommandLineOption( "-v or --version", "Display the osgBullet version string." );


    if( arguments.read( "-v" ) || arguments.read( "--version" ) )
    {
        osg::notify( osg::ALWAYS ) << osgbCollision::getVersionString() << std::endl;
        osg::notify( osg::ALWAYS ) << "  (Bullet version " << BT_BULLET_VERSION;
#ifdef BT_USE_DOUBLE_PRECISION
        osg::notify( osg::ALWAYS ) << " double precision";
#endif
        osg::notify( osg::ALWAYS ) << ")" << std::endl << std::endl;
    }

    const bool briefHelp = arguments.read( "-h" );
    const bool fullHelp = arguments.read( "--help" );
    if( briefHelp || fullHelp )
        osg::notify( osg::ALWAYS ) << arguments.getApplicationUsage()->getDescription() << std::endl;
    if( briefHelp )
        osg::notify( osg::ALWAYS ) << "Usage: " << arguments.getApplicationUsage()->getCommandLineUsage() << std::endl;
    else if( fullHelp )
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
    if( briefHelp || fullHelp )
        osg::notify( osg::ALWAYS ) << "Use the Delete key to reset the physics simultation." << std::endl;

    if( arguments.argc() <= 1 )
    {
        if( !briefHelp && !fullHelp )
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    // Get all arguments.
    BroadphaseNativeTypes shapeType( BOX_SHAPE_PROXYTYPE );
    if( arguments.read( "--box" ) ) shapeType = BOX_SHAPE_PROXYTYPE;
    if( arguments.read( "--sphere" ) ) shapeType = SPHERE_SHAPE_PROXYTYPE;
    if( arguments.read( "--cylinder" ) ) shapeType = CYLINDER_SHAPE_PROXYTYPE;
    if( arguments.read( "--triMesh" ) ) shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
    if( arguments.read( "--convexTM" ) ) shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
    if( arguments.read( "--convexHull" ) ) shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;

    switch( shapeType )
    {
    case BOX_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: Box" << std::endl;
        break;
    case SPHERE_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: Sphere" << std::endl;
        break;
    case CYLINDER_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: Cylinder" << std::endl;
        break;
    case TRIANGLE_MESH_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: TriMesh" << std::endl;
        break;
    case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: ConvexTriMesh" << std::endl;
        break;
    case CONVEX_HULL_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: ConvexHull" << std::endl;
        break;
    default:
        osg::notify( osg::FATAL ) << "osgbpp: Error, unknown shape type, using tri mesh." << std::endl;
        shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
        break;
    }

    std::string str;
    osgbCollision::AXIS axis( osgbCollision::Z );
    if ( arguments.read( "--axis", str ) )
    {
        if( (str.find( "X" ) != str.npos) || (str.find( "x" ) != str.npos) )
            axis = osgbCollision::X;
        else if( (str.find( "Y" ) != str.npos) || (str.find( "y" ) != str.npos) )
            axis = osgbCollision::Y;
        else if( (str.find( "Z" ) != str.npos) || (str.find( "z" ) != str.npos) )
            axis = osgbCollision::Z;
        else
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    switch( axis )
    {
    case osgbCollision::X:
        osg::notify( osg::INFO ) << "osgbpp: Axis: X" << std::endl;
        break;
    case osgbCollision::Y:
        osg::notify( osg::INFO ) << "osgbpp: Axis: Y" << std::endl;
        break;
    case osgbCollision::Z:
        osg::notify( osg::INFO ) << "osgbpp: Axis: Z" << std::endl;
        break;
    }


    float decimatorPercent( 1. );
    float decimatorMaxError( FLT_MAX );
    if ( arguments.read( "--decPercent", str ) )
    {
        if( sscanf( str.c_str(), "%f", &decimatorPercent ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
        if ( arguments.read( "--decMaxError", str ) )
        {
            if( sscanf( str.c_str(), "%f", &decimatorMaxError ) != 1 )
            {
                arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
                return 1;
            }
        }
    }
    if (decimatorPercent != 1.f )
        osg::notify( osg::INFO ) << "osgbpp: DecimatorOp: " << decimatorPercent << ", " << decimatorMaxError << std::endl;

    float simplifyPercent = 1.f;
    if ( arguments.read( "--simplify", str ) )
    {
        if( sscanf( str.c_str(), "%f", &simplifyPercent ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    if (simplifyPercent != 1.f )
        osg::notify( osg::INFO ) << "osgbpp: Simplify: " << simplifyPercent << std::endl;

    unsigned int vertexAggMaxVerts( 0 );
    osg::Vec3 vertexAggMinCellSize( 0., 0., 0. );
    if ( arguments.read( "--aggMaxVerts", str ) )
    {
        if( sscanf( str.c_str(), "%u", &vertexAggMaxVerts ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
        if ( arguments.read( "--aggMinCellSize", str ) )
        {
            char comma;
            std::istringstream oStr( str );
            oStr >> vertexAggMinCellSize[ 0 ] >> comma >>
                vertexAggMinCellSize[ 1 ] >> comma >>
                vertexAggMinCellSize[ 2 ];
        }
    }
    if (vertexAggMaxVerts > 0 )
        osg::notify( osg::INFO ) << "osgbpp: VertexAggOp: " << vertexAggMaxVerts << ", " << vertexAggMinCellSize << std::endl;

    float mass( 1.f );
    if ( arguments.read( "--mass", str ) )
    {
        if( sscanf( str.c_str(), "%f", &mass ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    if (mass != 1.f )
        osg::notify( osg::INFO ) << "osgbpp: Mass: " << mass << std::endl;

    bool comSpecified;
    std::string comStr;
    osg::Vec3 com;
    if( comSpecified = arguments.read( "--com", comStr ) )
    {
        char comma;
        std::istringstream oStr( comStr );
        oStr >> com[ 0 ] >> comma >> com[ 1 ] >> comma >> com[ 2 ];
        osg::notify( osg::INFO ) << "osgbpp: Using center of mass: " << com << std::endl;
    }


    const bool overall( arguments.read( "--overall" ) != 0 );
    if (overall)
        osg::notify( osg::INFO ) << "osgbpp: Overall" << std::endl;


    osgbCollision::GLDebugDrawer dbgDraw;
    const bool debug( arguments.read( "--debug" ) );
    if( debug )
    {
        osg::notify( osg::INFO ) << "osgbpp: Debug" << std::endl;
        dbgDraw.setDebugMode( ~btIDebugDraw::DBG_DrawText );
    }

    osgViewer::Viewer viewer( arguments );

    arguments.reportRemainingOptionsAsUnrecognized();
    arguments.writeErrorMessages( osg::notify( osg::WARN ) );


    osg::ref_ptr< osg::Node > model = osgDB::readNodeFiles( arguments );
    if( !model )
    {
        osg::notify( osg::FATAL ) << "Can't load input file(s)." << std::endl;
        osg::notify( osg::ALWAYS ) << "Usage: " << arguments.getApplicationUsage()->getCommandLineUsage() << std::endl;
        return 1;
    }
    osg::notify( osg::INFO ) << "osgbpp: Loaded model(s)." << std::endl;
    osg::BoundingSphere bs = model->getBound();


    //
    // Create the rigid body.
    // 1. Create an AnsoluteModelTransform to parent the model.
    osg::ref_ptr< osgwTools::AbsoluteModelTransform > amtRoot( new osgwTools::AbsoluteModelTransform );
    amtRoot->addChild( model.get() );

    // 2. Specify rigid body parameters in a CreationRecord.
    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amtRoot.get();
    if( comSpecified )
        cr->setCenterOfMass( com );
    cr->_shapeType = shapeType;
    cr->_restitution = .5f;
    cr->_mass = mass;
    cr->_axis = axis;
    cr->_reductionLevel = osgbDynamics::CreationRecord::MINIMAL;

    // 3. Create the body.
    btRigidBody* rb( osgbDynamics::createRigidBody( cr.get() ) );
    if( rb == NULL )
    {
        osg::notify( osg::FATAL ) << "osgbpp: NULL rigid body." << std::endl;
        return( 1 );
    }
    rb->setActivationState( DISABLE_DEACTIVATION );

    // This is how DragHandler knows that the user has selected a rigid body.
    amtRoot->setUserData( new osgbCollision::RefRigidBody( rb ) );


    // Create a root for the scene graph, and add the AbsoluteModelTransform as a child.
    osg::ref_ptr<osg::Group> root = new osg::Group();
    viewer.setSceneData( root.get() );
    root->addChild( amtRoot.get() );


    // Create the dynamics world and add the rigid body.
    btDynamicsWorld* dynamicsWorld = initPhysics();
    dynamicsWorld->addRigidBody( rb );

    if( debug )
    {
        // Enable debug drawing.
        root->addChild( dbgDraw.getSceneGraph() );
        dynamicsWorld->setDebugDrawer( &dbgDraw );
    }

    // Compute a reasonable ground plane size based on the bounding sphere radius.
    float dim = bs._radius * 1.5;
    osg::Vec3 cen = bs._center;
    cen[ 2 ] -= dim;
    osg::ref_ptr< osg::Node > ground = createGround( dim, dim, cen, dynamicsWorld );
    root->addChild( ground.get() );


    // Viewer set up.
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator();
    viewer.setCameraManipulator( tb );

    // Add an event handler to let us reset the scene.
    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new osgbInteraction::SaveRestoreHandler;
    srh->add( dynamicsWorld );
    srh->capture();
    viewer.addEventHandler( srh.get() );

    osgViewer::Viewer::Cameras cams;
    viewer.getCameras( cams );
    viewer.addEventHandler( new osgbInteraction::DragHandler(
        dynamicsWorld, cams[ 0 ] ) );

    // Do this now, before getting the start time, so that it doesn't
    // negatively impact the first frame of the physics sim.
    viewer.realize();


    double prevSimTime = viewer.getFrameStamp()->getSimulationTime();
    while( !viewer.done())
    {
        if( debug )
            dbgDraw.BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        dynamicsWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        if( debug )
        {
            dynamicsWorld->debugDrawWorld();
            dbgDraw.EndDraw();
        }

        viewer.frame();
    }

    return( 0 );
}



/** \page osgbpp The osgBullet Physics Preview Application
osgbpp creates rigid bodies from OSG data and drops them on a platform. Use
this application to test rigid body creation with a variety of control
parameters.

\section su Simple Usage
Use osgbpp to create a rigid body from \c dice.osg, one of the osgBullet data files:

\code
C:\Projects>osgbpp dice.osg
\endcode

To see the model fall again, hit the Delete key. To drag the model, hold
down the control key and use the left mouse button.

By default, osgbpp creates a Bullet box collision shape. You can visualize the
collision shape with the --debug command line option:

\code
C:\Projects>osgbpp dice.osg --debug
\endcode

osgbpp supports command line patameters to control the collision shape type. For example:

\code
C:\Projects>osgbpp dice.osg --debug --cylinder
C:\Projects>osgbpp dice.osg --debug --convexHull
\endcode

\section au Advanced Usage
osgbpp lets you specify a non-origin center of mass with the --com command line option.
For example, you can simulate a loaded die with this command:

\code
C:\Projects>osgbpp dice.osg --com 1.1,0,0
\endcode

osgBullet can create a compound shape from Geodes in your scene graph, or can create
a single non-compound shape encompassing your entire scene graph. Use the \c --overall
option to create a single non-compound shape.

For example, compare the results of these two commands:

\code
C:\Projects>osgbpp dice.osg block.osg.(2,0,2.5).trans
C:\Projects>osgbpp dice.osg block.osg.(2,0,2.5).trans --overall
\endcode

In the first command, osgBullet creates two box shapes, one for each model, and assebled them
into a single btCompoundShape. In the second example, osgBullet creates a single box shape
around both models.

\section osgbppcontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.

\section osgbppclp Command Line Parameters
<table border="0">
  <tr>
    <td><b>--axis &lt;a&gt;</b></td>
    <td>Use this option to specify the cylinder axis X, Y, or Z. Default is Z. This argument is ignored if \c --cylinder is not specified.</td>
  </tr>
  <tr>
    <td><b>--box</b></td>
    <td>This is the default. Creates a box collision shape.</td>
  </tr>
  <tr>
    <td width=150><b>--com&nbsp;&lt;x&gt;,&lt;y&gt;,&lt;z&gt;</b></td>
    <td>Specifies the center of mass. If not specified, osgbpp uses the center of the OSG bounding sphere.
    Example: <b>--com 5.5,3,-1.</b> Do not include spaces in the center of mass vector.</td>
  </tr>
  <tr>
    <td><b>--convexHull</b></td>
    <td>Creates a convex hull collision shape.</td>
  </tr>
  <tr>
    <td><b>--convexTM</b></td>
    <td>Creates a convex tri mesh collision shape.</td>
  </tr>
  <tr>
    <td><b>--cylinder</b></td>
    <td>Creates a cylinder collision shape. See the <b>--axis</b> parameter.</td>
  </tr>
  <tr>
    <td><b>--debug</b></td>
    <td>Use the GLDebugDrawer class to display collision shapes.</td>
  </tr>
  <tr>
    <td><b>--mass &lt;n&gt;</b></td>
    <td>Specifies the desired rigid body mass value. The default is 1.0.</td>
  </tr>
  <tr>
    <td><b>--overall</b></td>
    <td>Creates a single collision shape for the entire input scene graph, rather than a collision shape per Geode, which is the default.</td>
  </tr>
  <tr>
    <td><b>--sphere</b></td>
    <td>Creates a sphere collision shape.</td>
  </tr>
  <tr>
    <td><b>--triMesh</b></td>
    <td>It creates a tri mesh collision shape (suitable for static objects).</td>
  </tr>
  <tr>
    <td><b>-v/--version</b></td>
    <td>Display the osgBullet version string.</td>
  </tr>
  <tr>
    <td><b>-h/--help</b></td>
    <td>Displays help text and command line documentation.</td>
  </tr>
</table>

*/
