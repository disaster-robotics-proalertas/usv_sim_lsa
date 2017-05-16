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

#include <osgUtil/Optimizer>

#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/BoundingSphere>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>

#include <osgwTools/FindNamedNode.h>
#include <osgwTools/InsertRemove.h>
#include <osgwTools/ReadFile.h>
#include <osgwTools/Shapes.h>
#include <osgwTools/Version.h>

#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/Constraints.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/GLDebugDrawer.h>

#include <osgbInteraction/HandNode.h>
#include <osgbInteraction/GestureHandler.h>
#include <osgbInteraction/HandTestEventHandler.h>
#include <osgbCollision/Utils.h>

#ifdef USE_P5
#include <osgbInteraction/p5support.h>
#endif

#include <osgwTools/AbsoluteModelTransform.h>

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btQuickprof.h>

#include <list>
#include <map>
#include <string>
#include <sstream>
#include <osg/io_utils>

#include <iostream>



//#define USE_PARALLEL_DISPATCHER
#ifdef USE_PARALLEL_DISPATCHER

#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"

#ifdef USE_LIBSPE2
#include "BulletMultiThreaded/SpuLibspe2Support.h"
#elif defined (WIN32)
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#elif defined (USE_PTHREADS)

#include "BulletMultiThreaded/PosixThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#else
//other platforms run the parallel code sequentially (until pthread support or other parallel implementation is added)
#include "BulletMultiThreaded/SequentialThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif //USE_LIBSPE2

#endif







/** \cond */
class GestureTest : public osgGA::GUIEventHandler
{
public:
    GestureTest( osgbInteraction::HandNode* handNode ) : _handNode( handNode ) {}

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
    {
        switch( ea.getEventType() )
        {
        case osgGA::GUIEventAdapter::KEYUP:
        {
            if( (ea.getKey() == 'd' ) || (ea.getKey() == 'D' ) )
            {
                osg::notify( osg::ALWAYS ) << "default" << std::endl;

                // Default gesture
                _handNode->sendGestureCode( osgbInteraction::GestureHandler::Default );
                _handNode->setPose( osgbInteraction::HandNode::POSE_DEFAULT );
                return( true );
            }
            else if( (ea.getKey() == 'p' ) || (ea.getKey() == 'P' ) )
            {
                // Point gesture
                _handNode->sendGestureCode( osgbInteraction::GestureHandler::Point );
                _handNode->setPose( osgbInteraction::HandNode::POSE_POINT );
                return( true );
            }
            else if( (ea.getKey() == 'f' ) || (ea.getKey() == 'F' ) )
            {
                osg::notify( osg::ALWAYS ) << "fist" << std::endl;

                // Fist gesture
                _handNode->sendGestureCode( osgbInteraction::GestureHandler::Fist );
                _handNode->setPose( osgbInteraction::HandNode::POSE_FIST );
                return( true );
            }
        }
        }
        return( false );
    }

protected:
    osg::ref_ptr< osgbInteraction::HandNode > _handNode;
};
/** \endcond */


btDiscreteDynamicsWorld* initPhysics( osg::Vec3 gravity = osg::Vec3( 0, 0, -9.8 ) )
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();

#ifdef USE_PARALLEL_DISPATCHER
    btThreadSupportInterface*		threadSupportCollision;
    threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								2));
    btCollisionDispatcher * dispatcher = new SpuGatheringCollisionDispatcher(
        threadSupportCollision, 2, collisionConfiguration );
#else
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );
#endif

    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( osgbCollision::asBtVector3( gravity * 9.8 ) );

    return( dynamicsWorld );
}


/** \cond */
// This class can read itself from a file. Long-term,
// we might want something like this in an osgPlugin form.
class ConfigReaderWriter : public osg::Referenced
{
public:
    ConfigReaderWriter( osg::Group* root, btDiscreteDynamicsWorld* dw, osgbInteraction::HandNode* hn )
      : _up( osg::Vec3( 0, 0, 1 ) ),
        _root( root ),
        _dw( dw ),
        _hn( hn )
    {}

    bool read( const std::string& fileName )
    {
        std::string inFile( osgDB::findDataFile( fileName ) );
        std::ifstream in( inFile.c_str() );
        if( !in.good() )
        {
            osg::notify( osg::FATAL ) << "CondigReaderWriter::read: Can't open " << fileName << std::endl;
            return false;
        }

        osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;

        short filterGroup( 0 );
        short filterWith( 0 );
        osg::Vec4 groundPlane( 0., 0., 1., 0. );
        bool noGroundPlane( false );
        osg::Vec2s groundFilters( 0, 0 );

        std::string nodeName;
        bool matchAllNodes( false );

        float translateUp( 0. );

        const int bufSize( 1024 );
        char bufCh[ bufSize ];
        in.getline( bufCh, bufSize );
        std::string buf( bufCh );
        while( !in.eof() )
        {
            osg::notify( osg::DEBUG_INFO ) << "Data: " << buf << std::endl;
            int spacePos( buf.find_first_of( " " ) );
            std::string key;
            if( spacePos != std::string::npos )
                key = buf.substr( 0, spacePos );
            else
                key = buf;
            osg::notify( osg::DEBUG_INFO ) << spacePos << " KEY " << key << std::endl;

            std::istringstream istr( buf.substr( ( spacePos != std::string::npos ) ? spacePos : 0 ) );
            in.getline( bufCh, bufSize );
            buf = std::string( bufCh );
            if( ( key.empty() ) ||
                ( key == std::string( "#" ) ) )
            {
                osg::notify( osg::INFO ) << "  Found comment." << std::endl;
                continue;
            }

            if( key == std::string( "Up:" ) )
            {
                istr >> _up;
                _dw->setGravity( osgbCollision::asBtVector3( _up * -9.8 ) );
            }
            else if( key == std::string( "TranslateUp:" ) )
            {
                istr >> translateUp;
            }
            else if( key == std::string( "GroundPlane:" ) )
            {
                istr >> groundPlane[ 0 ] >> groundPlane[ 1 ] >> groundPlane[ 2 ] >> groundPlane[ 3 ];
                groundFilters = osg::Vec2s( filterGroup, filterWith );
            }
            else if( key == std::string( "NoGroundPlane:" ) )
            {
                noGroundPlane = true;
            }
            else if( key == std::string( "HandNode:" ) )
            {
                osg::Vec3 pos;
                osg::Quat quat;
                float length;
                bool right;
                istr >> pos >> quat >> length >> right;
                if( !_hn.valid() )
                    osg::notify( osg::WARN ) << "  Warning: No HandNode." << std::endl;
                else
                {
                    _hn->setHandLength( length );
                    _hn->setHandedness( right ? osgbInteraction::HandNode::RIGHT : osgbInteraction::HandNode::LEFT );
                    _hn->setPosition( pos );
                    _hn->setAttitude( quat );
                    _hn->setDebug( true );
                }
            }
            else if( key == std::string( "Model:" ) )
            {
                std::string modelName;
                istr >> modelName;

                osg::ref_ptr< osg::Node > model = osgDB::readNodeFile( modelName );
                if( !model.valid() )
                    osg::notify( osg::WARN ) << "  Warning: Can't find model " << modelName << std::endl;
                else if( translateUp != 0.f )
                {
                    osg::ref_ptr< osg::MatrixTransform > orient = new osg::MatrixTransform(
                        osg::Matrix::translate( _up * translateUp ) );
                    orient->addChild( model.get() );
                    _root->addChild( orient.get() );
                }
                else
                    _root->addChild( model.get() );
            }
            else if( key == std::string( "Node:" ) )
            {
                istr >> nodeName;
            }
            else if( key == std::string( "MatchAllNodes:" ) )
            {
                istr >> matchAllNodes;
            }
            else if( key == std::string( "Mass:" ) )
            {
                istr >> cr->_mass;
            }
            else if( key == std::string( "Shape:" ) )
            {
                int shape;
                istr >> shape;
                cr->_shapeType = (BroadphaseNativeTypes)shape;
            }
            else if( key == std::string( "Overall:" ) )
            {
                istr >> cr->_overall;
            }
            else if( key == std::string( "CollisionFilter:" ) )
            {
                istr >> std::hex >> filterGroup >> filterWith >> std::dec;
            }
            else if( key == std::string( "Body:" ) )
            {
                if( nodeName.empty() )
                {
                    osg::notify( osg::WARN ) << "Warning: Body: No node name specified." << std::endl;
                    continue;
                }

                osgwTools::FindNamedNode fnn( nodeName );
                // Don't include the target node in the returned paths.
                fnn.setPathsIncludeTargetNode( false );
                _root->accept( fnn );
                if( fnn._napl.empty() )
                {
                    osg::notify( osg::WARN ) << "Warning: Body: Could not find node named \"" << nodeName << "\"." << std::endl;
                    continue;
                }

                osgwTools::FindNamedNode::NodeAndPathList::iterator it;
                for( it = fnn._napl.begin(); it != fnn._napl.end(); it++ )
                {
                    btRigidBody* rb = createRigidBody(
                        it->first, it->second, cr.get() );
                    if( rb != NULL )
                    {
                        if( (filterGroup == 0) && (filterWith == 0) )
                            _dw->addRigidBody( rb );
                        else
                            _dw->addRigidBody( rb, filterGroup, filterWith );
                    }
                    if( !matchAllNodes )
                        break;
                }
            }
            else if( key == std::string( "AttachBody:" ) )
            {
                if( matchAllNodes )
                    // TBD
                    osg::notify( osg::WARN ) << "Warning: MmatchAllNodes is not yet implemented for AttachBody." << std::endl;

                std::string loadAndAttachName;
                istr >> loadAndAttachName;

                osg::ref_ptr< osg::Node > loadedModel = osgDB::readNodeFile( loadAndAttachName );
                if( !loadedModel.valid() )
                {
                    osg::notify( osg::WARN ) << "Warning: Could not load AttachBody: " << loadAndAttachName << std::endl;
                    continue;
                }

                // Figure out where to attach it.
                osg::Group* attachParent = _root.get();
                osg::NodePath np;
                if( !nodeName.empty() )
                {
                    osgwTools::FindNamedNode fnn( nodeName );
                    _root->accept( fnn );
                    if( fnn._napl.empty() )
                    {
                        osg::notify( osg::WARN ) << "Warning: Could not find node named \"" << nodeName << "\" for AttachBody parent." << std::endl;
                        continue;
                    }
                    attachParent = fnn._napl.front().first->asGroup();
                    if( attachParent == NULL )
                    {
                        osg::notify( osg::WARN ) << "Warning: Node named \"" << nodeName << "\" not a Group (in AttachBody)." << std::endl;
                        continue;
                    }
                    np = fnn._napl.front().second;
                }
                else
                    np.push_back( _root.get() );
                attachParent->addChild( loadedModel.get() );

                btRigidBody* rb = createRigidBody( loadedModel.get(), np, cr.get() );
                if( rb != NULL )
                {
                    if( (filterGroup == 0) && (filterWith == 0) )
                        _dw->addRigidBody( rb );
                    else
                        _dw->addRigidBody( rb, filterGroup, filterWith );
                }
            }
            else if( key == std::string( "Hinge:" ) )
            {
                std::string nodeA, nodeB;
                osg::Vec3 pivotA, pivotB;
                osg::Vec3 axisA, axisB;
                istr >> nodeA >> nodeB >> pivotA >> pivotB >> axisA >> axisB;

                osg::Matrix aXform, bXform;
                btRigidBody* rbA = lookupRigidBody( nodeA, aXform );
                btRigidBody* rbB = lookupRigidBody( nodeB, bXform );
                if( ( rbA == NULL ) || ( rbB == NULL ) )
                    continue;

                const btVector3 btPivotA( osgbCollision::asBtVector3( pivotA ) );
                const btVector3 btPivotB( osgbCollision::asBtVector3( pivotB ) );
                btVector3 btAxisA( osgbCollision::asBtVector3( axisA ) );
                btVector3 btAxisB( osgbCollision::asBtVector3( axisB ) );
                btHingeConstraint* hinge = new btHingeConstraint( *rbA, *rbB,
                    btPivotA, btPivotB, btAxisA, btAxisB );

                hinge->setLimit( -3.f, -.3f );
                //hinge->setAngularOnly( true );
                _dw->addConstraint( hinge, true );
                //spHingeDynAB->setDbgDrawSize(btScalar(5.f));
            }
            else if( key == std::string( "Slider:" ) )
            {
                std::string nodeA, nodeB;
                osg::Vec3 axis;
                float low, high;
                bool useA;
                istr >> nodeA >> nodeB >> axis >> low >> high >> useA;

                osg::Matrix axisMatrix;
                if( axis != osg::Vec3( 1., 0., 0. ) )
                    axisMatrix = osg::Matrix::rotate( osg::Vec3( 1., 0., 0. ), axis );

                // Find Node A (usually a static object, like the cabinet framework.
                osgwTools::FindNamedNode fnnA( nodeA );
                _root->accept( fnnA );

                // We get back a list of nodes and paths, but only use the first one.
                osgwTools::FindNamedNode::NodeAndPath& napA( fnnA._napl[ 0 ] );
                osgwTools::AbsoluteModelTransform* amtA = dynamic_cast< osgwTools::AbsoluteModelTransform* >( napA.first->getParent( 0 ) );
                if( amtA == NULL )
                {
                    osg::notify( osg::FATAL ) << "Slider node (" << nodeA << ") parent is not AMT." << std::endl;
                    continue;
                }
                osgbCollision::RefRigidBody* rbA = dynamic_cast< osgbCollision::RefRigidBody* >( amtA->getUserData() );
                if( rbA == NULL )
                {
                    osg::notify( osg::FATAL ) << "AMT for " << nodeA << " has invalid user data." << std::endl;
                    continue;
                }

                osg::ref_ptr< osg::Node > axes = osgDB::readNodeFile( "axes.osg" );
                osg::BoundingSphere bs;
                {
                    // Debug -- Visualize the reference frame with the axis model at the center of mass.
                    bs = amtA->getChild( 0 )->getBound();
                    osg::Vec3 centerA( bs.center() );
                    osg::MatrixTransform* mtA = new osg::MatrixTransform( osg::Matrix::translate( centerA ) );
                    mtA->addChild( axes.get() );
                    amtA->addChild( mtA );
                }

                // Get the world coordinate center to assist in computing the reference frame of the constraint.
                // Body B will be constrained to (centerB - centerA).
                bs = amtA->getBound();
                osg::Vec3 centerA = bs.center();

                // Get the 3x3 basis of body A's reference frame.
                osg::Matrix m;
                amtA->computeLocalToWorldMatrix( m, NULL );
                m = m * axisMatrix;
                btMatrix3x3 m3A( osgbCollision::asBtMatrix3x3( m ) );


                // Find all possible body B objects.
                // Usually something dynamic, like a drawer.
                // We will constraint each body B to the first body A (which we found and processed above)
                osgwTools::FindNamedNode fnnB( nodeB );
                _root->accept( fnnB );

                osgwTools::FindNamedNode::NodeAndPath& napB( fnnB._napl[ 0 ] );
                unsigned int idx;
                for( idx=0; idx<fnnB._napl.size(); idx++ )
                {
                    osgwTools::FindNamedNode::NodeAndPath& nap( fnnB._napl[ idx ] );
                    osgwTools::AbsoluteModelTransform* amtB = dynamic_cast< osgwTools::AbsoluteModelTransform* >( nap.first->getParent( 0 ) );
                    if( amtB == NULL )
                    {
                        osg::notify( osg::FATAL ) << "Slider node (" << nodeB << ") parent is not AMT." << std::endl;
                        continue;
                    }
                    osgbCollision::RefRigidBody* rbB = dynamic_cast< osgbCollision::RefRigidBody* >( amtB->getUserData() );
                    if( rbB == NULL )
                    {
                        osg::notify( osg::FATAL ) << "AMT for " << nodeB << " has invalid user data." << std::endl;
                        continue;
                    }

                    {
                        // Debug -- Visualize the reference frame with the axis model at the center of mass.
                        bs = amtB->getChild( 0 )->getBound();
                        osg::Vec3 centerB( bs.center() );
                        osg::MatrixTransform* mtB = new osg::MatrixTransform( osg::Matrix::translate( centerB ) );
                        mtB->addChild( axes.get() );
                        amtB->addChild( mtB );
                    }

                    // Get the world coordinate center to assist in computing the reference frame of the constraint.
                    // Body B will be constrained to offsetA = (centerB - centerA).
                    bs = amtB->getBound();
                    osg::Vec3 centerB = bs.center();
                    btVector3 offsetA( osgbCollision::asBtVector3( centerB - centerA ) );

                    // Get the 3x3 basis of body B's reference frame.
                    amtB->computeLocalToWorldMatrix( m, NULL );
                    m = m * axisMatrix;
                    btMatrix3x3 m3B( osgbCollision::asBtMatrix3x3( m ) );

                    // Finally, set up the constraint.
                    // Set the Bullet Transforms. The constraint will align these two coordinate systems.
                    // If _useA is true, frameB is placed in frameA between _low and _high on the specified _axis (axisMatrix).
                    const btTransform frameInA( m3A, offsetA );
                    const btTransform frameInB( m3B );
                    btSliderConstraint* slider = new btSliderConstraint( *( rbA->get() ), *( rbB->get() ),
                        frameInA, frameInB, useA );
                    slider->setLowerLinLimit( low );
                    slider->setUpperLinLimit( high );
                    _dw->addConstraint( slider, true );
                }
            }
            else if( key == std::string( "Constraint:" ) )
            {
                std::string fileName, nodeA, nodeB;
                istr >> fileName >> nodeA;
                istr >> nodeB;
                if( istr.fail() )
                    nodeB.clear();
                osg::notify( osg::INFO ) << fileName << "  nodeA " << nodeA << " nodeB " << nodeB << std::endl;

                osg::ref_ptr< osgbDynamics::Constraint > cons = static_cast< osgbDynamics::Constraint* >(
                    osgDB::readObjectFile( fileName ) );
                if( cons == NULL )
                {
                    osg::notify( osg::WARN ) << "Unable to load constraint file \"" << fileName << "\"." << std::endl;
                    continue;
                }

                osg::Matrix aXform;
                btRigidBody* rbA = lookupRigidBody( nodeA, aXform );
                if( rbA == NULL )
                    continue;
                cons->setAXform( aXform );

                btRigidBody* rbB( NULL );
                if( !( nodeB.empty() ) )
                {
                    osg::Matrix bXform;
                    rbB = lookupRigidBody( nodeB, bXform );
                    if( rbB == NULL )
                        continue;
                    cons->setBXform( bXform );
                }
                cons->setRigidBodies( rbA, rbB );
                _dw->addConstraint( cons->getConstraint() );
            }
            else if( key == std::string( "WirePlane:" ) )
            {
                osg::Vec3 c, u, v;
                osg::Vec2s subdiv;
                istr >> c >> u >> v >> subdiv;
                if( istr.fail() )
                {
                    osg::notify( osg::WARN ) << "WirePlane: error." << std::endl;
                    continue;
                }
                osg::notify( osg::INFO ) << "WirePlane: " << c << " " << u << " " << v << " " << subdiv << std::endl;

                osg::Geode* geode = new osg::Geode;
                geode->addDrawable( osgwTools::makeWirePlane( c, u, v, subdiv ) );
                _root->addChild( geode );
            }
            else
                osg::notify( osg::WARN ) << "ConfigReaderWriter: Unknown key: " << key << std::endl;
        }

        if( !noGroundPlane )
        {
            osg::Node* groundNode;
            if( groundFilters == osg::Vec2s( 0, 0 ) )
                groundNode = osgbDynamics::generateGroundPlane( groundPlane, _dw );
            else
                groundNode = osgbDynamics::generateGroundPlane( groundPlane, _dw, NULL, filterGroup, filterWith );
            _root->addChild( groundNode );
        }

        return true;
    }
    bool write( const std::string& fileName )
    {
        // Disabling this unused code.
#if 0
        std::ofstream out( fileName.c_str() );
        if( !out.good() )
        {
            osg::notify( osg::FATAL ) << "CondigReaderWriter::write: Can't open " << fileName << std::endl;
            return false;
        }
        out << "HandNode: " << _hn->getPosition() << " " <<
            _hn->getAttitude() << " " <<
            _hn->getHandLength() << " " <<
            (_hn->getHandedness() == osgbInteraction::HandNode::RIGHT) << std::endl;
        out << "Model: " << _model << std::endl;
        NodeDaeMap::const_iterator itr;
        for( itr = _nodeDaeMap.begin(); itr != _nodeDaeMap.end(); itr++ )
        {
            out << "Node: " << itr->first << std::endl;
            out << "DAE: " << itr->second << std::endl;
        }
        return true;
#endif
    }

    osg::Vec3 _up;

protected:
    ~ConfigReaderWriter() {}

    btRigidBody* createRigidBody( osg::Node* subgraph, const osg::NodePath& np, osgbDynamics::CreationRecord* cr )
    {
        osg::ref_ptr< osgwTools::AbsoluteModelTransform > amt = new osgwTools::AbsoluteModelTransform;
        amt->setDataVariance( osg::Object::DYNAMIC );
        osgwTools::insertAbove( subgraph, amt.get() );
        cr->_sceneGraph = amt.get();

        osg::Matrix m = osg::computeLocalToWorld( np );
        cr->_scale = m.getScale();
        m = osg::Matrix::scale( 1. / cr->_scale[0], 1. / cr->_scale[1], 1. / cr->_scale[2] ) * m;
        cr->_parentTransform = m;
        cr->setCenterOfMass( subgraph->getBound().center() );

        btRigidBody* rb = osgbDynamics::createRigidBody( cr );
        if( rb == NULL )
        {
            osg::notify( osg::WARN ) << "Warning: createRigidBody: NULL rigid body." << std::endl;
            return( NULL );
        }
        rb->setActivationState( DISABLE_DEACTIVATION );
        amt->setUserData( new osgbCollision::RefRigidBody( rb ) );

        btTransform wt;
        rb->getMotionState()->getWorldTransform( wt );
        rb->setWorldTransform( wt );

        return( rb );
    }

    btRigidBody* lookupRigidBody( const std::string& nodeName, osg::Matrix& xform )
    {
        osgwTools::FindNamedNode fnnA( nodeName );
        _root->accept( fnnA );

        // We get back a list of nodes and paths, but only use the first one.
        osgwTools::FindNamedNode::NodeAndPath& napA( fnnA._napl[ 0 ] );
        osgwTools::AbsoluteModelTransform* amtA = dynamic_cast< osgwTools::AbsoluteModelTransform* >( napA.first->getParent( 0 ) );
        if( amtA == NULL )
        {
            osg::notify( osg::FATAL ) << "Node \"" << nodeName << "\" parent is not AMT." << std::endl;
            return( NULL );
        }
        osgbCollision::RefRigidBody* rbA = dynamic_cast< osgbCollision::RefRigidBody* >( amtA->getUserData() );
        if( rbA == NULL )
        {
            osg::notify( osg::FATAL ) << "AMT for \"" << nodeName << "\" has invalid user data." << std::endl;
            return( NULL );
        }

        osg::NodePath np = napA.second;
    
        np.resize( np.size() - 2 );
        xform = osg::computeLocalToWorld( np );

        return( rbA->get() );
    }


    osg::ref_ptr< osg::Group > _root;
    btDiscreteDynamicsWorld* _dw;
    osg::ref_ptr< osgbInteraction::HandNode > _hn;
};
/** \endcond */


int main( int argc, char** argv )
{
    if( argc < 2 )
    {
        osg::notify( osg::FATAL ) << "Specify a config file (such as \"concave.txt\") on the command line." << std::endl;
        return 1;
    }
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::ref_ptr< osg::Group > root = new osg::Group;

    osg::ref_ptr< osgbInteraction::HandNode > hn = new
        osgbInteraction::HandNode( bulletWorld );
    root->addChild( hn.get() );

    // Look for config and model files on the command line.
    std::string configFile( "" );
    std::string otherFiles( "" );
    int idx;
    for( idx=1; idx<arguments.argc(); idx++ )
    {
        if( arguments.isOption( arguments[ idx ] ) )
            continue;

        const std::string thisArg( arguments[ idx ] );
        const std::string fileExt = osgDB::convertToLowerCase( osgDB::getFileExtension( thisArg ) );
        if( fileExt == "txt" )
            configFile = thisArg;
        else
            otherFiles += ( thisArg + std::string( " " ) );
    }
    if( !( configFile.empty() ) )
    {
        ConfigReaderWriter* crw = new ConfigReaderWriter( root.get(), bulletWorld, hn.get() );
        //osg::setNotifyLevel( osg::DEBUG_FP );
        crw->read( configFile );
        //osg::setNotifyLevel( osg::NOTICE );
    }
    if( !( otherFiles.empty() ) )
    {
        // Treat non-options and non-text files as other files to be loaded and rendered,
        // but not part of the physics sim.
        root->addChild( osgwTools::readNodeFiles( otherFiles ) );
    }


    //This is the default value for the island deactivation time. The smaller this
    //value is the sooner the island will be put to sleep. I believe this number is
    //in seconds. It can be located in btRigidBody.cpp.
    //btScalar	gDeactivationTime = btScalar(2.);
    gDeactivationTime = btScalar(1.);


    // Displays the hand origin's desired position.
    //hn->setDebug( true );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 30, 30, 768, 480 );
    viewer.setSceneData( root.get() );
    viewer.addEventHandler( new osgbInteraction::VirtualHandTestEventHandler( hn.get() ) );

    // Support for gestures.
    viewer.addEventHandler( new GestureTest( hn.get() ) );
    hn->getGestureHandlerVector().push_back( new osgbInteraction::GripRelease );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    viewer.setCameraManipulator( tb );

    // Add osgviewer-style event handlers.
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    viewer.addEventHandler(new osgViewer::ThreadingHandler);
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
    viewer.addEventHandler(new osgViewer::StatsHandler);
    //viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);
    viewer.addEventHandler(new osgViewer::LODScaleHandler);
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );
    }

    osg::ref_ptr< osg::Node > foo( viewer.getSceneData() );
    osgUtil::Optimizer opt;
    opt.optimize( foo.get(),
        osgUtil::Optimizer::DEFAULT_OPTIMIZATIONS &
        ~osgUtil::Optimizer::REMOVE_REDUNDANT_NODES &
        ~osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS );

#ifdef USE_P5
    osgbInteraction::P5Glove* gloveZero = new osgbInteraction::P5Glove( true );
	if( !gloveZero->getReady() )
    {
		osg::notify( osg::WARN ) << "P5 Glove failed to initialize." << std::endl;
        delete gloveZero;
        gloveZero = NULL;
    }
    if( gloveZero != NULL )
	    gloveZero->setMovementScale( osg::Vec3( 0.6, 0.6, 0.6 ) );
#endif

    viewer.realize();
    double currSimTime;
    double prevSimTime = viewer.getFrameStamp()->getSimulationTime();

    while( !viewer.done() )
    {
#ifdef USE_P5
        if( gloveZero != NULL )
            gloveZero->updateHandState(hn.get()); // update from glove
#endif

        if( dbgDraw != NULL )
            dbgDraw->BeginDraw();

        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime, 8, 1./120. );
        float dt = currSimTime - prevSimTime;
        prevSimTime = currSimTime;

        if( dbgDraw != NULL )
        {
            bulletWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }

    return( 0 );
}


/** \page handphysicsexample The Hand Example
\c handphysics loads a scene from a configuration file and allows the user
to interact with the scene using the osgbInteraction::HandNode.

You can use the \c HandNode with keyboard controls or with a P5 data glove.
To use the P5 data glove, you must example P5 usage in osgBullet's CMake
system, then rebuild osgBullet with P5 support.

For keyboard controls, see osgbInteraction::VirtualHandTestEventHandler.

An example configuration file, \c concave.txt, is in the \c data directory.
Run the example like this:

\code
> handphysics concave.txt
\endcode

*/
