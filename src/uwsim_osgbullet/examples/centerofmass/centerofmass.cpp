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
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>

#include <osgbDynamics/GroundPlane.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/SaveRestoreHandler.h>
#include <osgbInteraction/DragHandler.h>

#include <btBulletDynamicsCommon.h>
#include <sstream>


btRigidBody* createObject( osg::Group* parent, const osg::Matrix& m,
                          osgbInteraction::SaveRestoreHandler* srh,
                          const osg::Vec3& com=osg::Vec3(0,0,0), bool setCom=false )
{
    osg::Node* node = osgDB::readNodeFile( "com.osg" );
    if( node == NULL )
    {
        osg::notify( osg::FATAL ) << "Can't load file \"com.osg\". Make sure osgBullet data directory is in OSG_FILE_PATH." << std::endl;
        return( NULL );
    }

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    parent->addChild( mt );
    mt->addChild( node );

    // Begin_doxygen example code block
    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    if( setCom )
        cr->setCenterOfMass( com );
    cr->_sceneGraph = mt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_parentTransform = m;
    cr->_restitution = 1.f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );
    // End_doxygen example code block

    rb->setAngularVelocity( btVector3( 0., .2, 0. ) );

    mt->setUserData( new osgbCollision::RefRigidBody( rb ) );
    std::ostringstream id;
    id << std::hex << mt;
    srh->add( id.str(), rb );

    return( rb );
}


btDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ) );

    return( dynamicsWorld );
}


int main( int argc, char** argv )
{
    btDynamicsWorld* bw = initPhysics();
    osg::Group* root = new osg::Group;

    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new
        osgbInteraction::SaveRestoreHandler;

    osg::Matrix m;

    // Left object:
    // Create the object on the left, which has center of mass at the model origin.
    // In this case, that's the lower-left front of the model. This is almost certainly
    // NOT what you want, but is what you would get with a naive conversion of OSG data
    // into a collision shape.
    m = osg::Matrix::rotate( .4, 0., 0., 1. ) * osg::Matrix::translate( -24., 0., 10. );
    bw->addRigidBody( createObject( root, m, srh.get(), osg::Vec3( 0., 0., 0. ), true ) );

    // Center object:
    // Specify the actual center of the mass of the model, in the model's own coordinate
    // space. In the case of this model, the COM is approx ( 2.15, 3., 2. ). This results
    // in much more realistic dynamics.
    m = osg::Matrix::rotate( .4, 0., 0., 1. ) * osg::Matrix::translate( -4., 0., 10. );
    bw->addRigidBody( createObject( root, m, srh.get(), osg::Vec3( 2.15, 3., 2. ), true ) );

    // Right object:
    // If you don't specify the center of mass, osgBullet tries to do you a favor, and
    // uses the bounding volume center as the COM. This works well in a lot of cases, but
    // for a model such as com.osg, you really should specify the COM expliticly.
    m = osg::Matrix::rotate( .4, 0., 0., 1. ) * osg::Matrix::translate( 16., 0., 10. );
    bw->addRigidBody( createObject( root, m, srh.get() ) );

    root->addChild( osgbDynamics::generateGroundPlane( osg::Vec4( 0.f, 0.f, 1.f, 0.f ), bw ) );


    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 30, 30, 768, 480 );
    viewer.setSceneData( root );
    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    viewer.setCameraManipulator( tb );

    viewer.realize();
    srh->capture();
    viewer.addEventHandler( srh.get() );
    viewer.addEventHandler( new osgbInteraction::DragHandler(
        bw, viewer.getCamera() ) );


    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bw->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;
        viewer.frame();
    }

    return( 0 );
}


/** \page examplecom Support For Non-Origin Center Of Mass
Bullet dynamics always assumes the collision shape origin is the center of mass (COM).
However, 3D model origins rarely coincide with the COM. A simple conversion of OSG
geometric data into a collision shape would almost certainly result in incorrect
dynamics due to this issue.

osgBullet supports 3D models with non-origin COM. The support is implemented in the
\ref osgbDynamics::MotionState "MotionState" class and the
\link collisionshapes collision shape creation utilities. \endlink
If your application uses the
\link rigidbody rigid body creation utilities, \endlink
then you merely specify the COM in the
\ref osgbDynamics::CreationRecord "CreationRecord".

\dontinclude centerofmass.cpp
\skipline Begin_doxygen
\until createRigidBody

Note that the call to setCenterOfMass() is conditional; if you
don't call it, osgBullet will use the bounding volume center
as the COM. However, osgBullet never uses the origin as the COM,
unless you specify it directly with a call to setCenterOfMass(),
or it just happens to coincide with the model origin.

If your application doesn't use the
\link rigidbody rigid body creation utilities, \endlink
then you should emulate the RigidBody.cpp source code.

\section examdescrip The centerofmass Example

The \c centerofmass example demonstrates setting COM on
a 3D model. It uses the \c com.osg model file, which has
its origin at the lower-left front corner. To see the
model origin, use the osgWorks utility \c osgwbvv:

\code
osgwbvv com.osg --box --origin
\endcode

Clearly, using the model origin as the COM would be incorrect. You can
see this incorrect behavior when you run the example; look at the model
on the left. If osgBullet (or your application) were to do a naive
comversion of OSG geometric data to a Bullet collision shape, this is
what you'd get.

If you don't specify a COM at all (that is, your application doesn't call
osgbDynamics::CreationRecord::setCenterOfMass() ), osgBullet uses the
subgraph bounding volume center as the center of mass. This works well for
many models, but not for \c com.osg. To see this incorrect behavior, run
the example and look at the model on the right.

The object in the center of the example has a correct COM, which produces
correct rigid body dynamics. This was accomplished using the code above.

\section comcontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.

*/
