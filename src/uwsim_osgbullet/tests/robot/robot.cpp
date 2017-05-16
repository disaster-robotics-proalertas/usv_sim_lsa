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

#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>

#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbDynamics/RigidBodyAnimation.h>
#include <osgbCollision/Utils.h>
#include <btBulletDynamicsCommon.h>
#include <osgbCollision/GLDebugDrawer.h>

#include <iostream>
#include <osg/io_utils>




btCompoundShape* _artShape;

/* \cond */
struct Joint
{
    Joint()
      : _angle( 0. ),
        _delta( .15 ),
        _axis( 0., -1., 0. ),
        _btChildIdx( -1 ),
        _idx( -1 ),
        _dependent( NULL )
    {}

    void operator++() { set( _angle + _delta ); }
    void operator--() { set( _angle - _delta ); }
    void set( float angle )
    {
        if (!_mt.valid())
        {
            osg::notify( osg::WARN ) << "Joint has invalid MatrixTransform." << std::endl;
            return;
        }

        _angle = angle;

        osg::Matrix get = osg::Matrix::translate( -_limbOSGOffset );
        osg::Matrix m = osg::Matrix::rotate( _angle, _axis );
        osg::Matrix put = osg::Matrix::translate( _limbOSGOffset );
        _mt->setMatrix( get * m * put );

        osg::Matrix l2w = osg::computeLocalToWorld( _l2wNodePath );
        get = osg::Matrix::translate( _limbBTOffset );
        osg::Matrix btm = get * m * put * l2w;
        if (_btChildIdx >= 0)
            _artShape->updateChildTransform( _btChildIdx, osgbCollision::asBtTransform( btm ) );
            //_artShape->getChildList()[ _btChildIdx ].m_transform
              //  = osgbCollision::asBtTransform( btm );

        if( _dependent != NULL )
            // Update the subordinate joint's world transform, because it
            // depends on out world transform.
            _dependent->set( _dependent->_angle );

        _artShape->recalculateLocalAabb();
    }

    void setMatrixTransform( osg::MatrixTransform* mt )
    {
        _mt = mt;
    }

    void setBtChildIdx( int idx )
    {
        _btChildIdx = idx;
    }

    void setDebugIdx( unsigned int idx )
    {
        _idx = (int)( idx );
    }


    float _angle;
    float _delta;
    osg::Vec3 _axis;
    osg::NodePath _l2wNodePath;
    osg::ref_ptr< osg::MatrixTransform > _mt;
    int _btChildIdx;

    osg::Vec3 _limbOSGOffset;
    osg::Vec3 _limbBTOffset;

    int _idx;

    Joint* _dependent;
};

Joint _joint0, _joint1;


class ArmManipulator : public osgGA::GUIEventHandler
{
public:
    ArmManipulator( btDynamicsWorld* dynamicsWorld ) : _dynamicsWorld( dynamicsWorld ) {}

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
    {
        const unsigned int mod = ea.getModKeyMask();
        const bool j1 = ( (mod&osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) ||
            (mod&osgGA::GUIEventAdapter::MODKEY_RIGHT_CTRL) );
        const bool j0 = ( !j1 || ( (mod&osgGA::GUIEventAdapter::MODKEY_LEFT_ALT) ||
            (mod&osgGA::GUIEventAdapter::MODKEY_RIGHT_ALT) ) );

        switch( ea.getEventType() )
        {
            case osgGA::GUIEventAdapter::KEYUP:
            {
                if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Left)
                {
                    if (j0)
                        ++_joint0;
                    if (j1)
                        ++_joint1;
                    return true;
                }
                else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Right)
                {
                    if (j0)
                        --_joint0;
                    if (j1)
                        --_joint1;
                    return true;
                }
                else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Home)
                {
                    _joint0.set( 0. );
                    _joint1.set( 0. );
                    return true;
                }
                else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_End)
                {
                    _joint0.set( .9 );
                    _joint1.set( .9 );
                    return true;
                }
                return false;
            }

            default:
            break;
        }
        return false;
    }

protected:
    btDynamicsWorld* _dynamicsWorld;
};



osg::MatrixTransform* createOSGBox( osg::Vec3 size )
{
    osg::Box * box = new osg::Box();

    box->setHalfLengths( size );

    osg::ShapeDrawable * shape = new osg::ShapeDrawable( box );

    osg::Geode * geode = new osg::Geode();
    geode->addDrawable( shape );

    osg::MatrixTransform * transform = new osg::MatrixTransform();
    transform->addChild( geode );

    return( transform );
}

btRigidBody * createBTBox( osg::MatrixTransform * box,
                          osg::Vec3 center )
{
    btCollisionShape* collision = osgbCollision::btBoxCollisionShapeFromOSG( box );

    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
    motion->setTransform( box );
    motion->setParentTransform( osg::Matrix::translate( center ) );

    btScalar mass( 0.0 );
    btVector3 inertia( 0, 0, 0 );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rb );

    return( body );
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

    dynamicsWorld->setGravity( btVector3( 0, 0, -10 ) );

    return( dynamicsWorld );
}


// Creates a Bullet collision shape from an osg::Shape.
class BulletShapeVisitor : public osg::ConstShapeVisitor
{
public:
    BulletShapeVisitor() : _shape( NULL ) {}
    virtual ~BulletShapeVisitor() {}

    virtual void apply( const osg::Shape& s )
    {
        osg::notify( osg::INFO ) << "Unknown shape." << std::endl;
    }
    virtual void apply( const osg::Sphere& s )
    {
        osg::notify( osg::INFO ) << "Found Sphere." << std::endl;

        osg::Vec3 c = s.getCenter();
        float radius = s.getRadius();

        btSphereShape* collision = new btSphereShape( radius );
        btTransform xform;
        xform.setIdentity();
        xform.setOrigin( osgbCollision::asBtVector3( c ) );

        _shape = new btCompoundShape;
        _shape->addChildShape( xform, collision );
    }
    virtual void apply( const osg::Box& s )
    {
        osg::notify( osg::INFO ) << "Found Box." << std::endl;

        osg::Vec3 c = s.getCenter();
        osg::Vec3 sizes = s.getHalfLengths();

        btBoxShape* collision = new btBoxShape( btVector3( sizes.x(), sizes.y(), sizes.z() ) );
        btTransform xform;
        xform.setIdentity();
        xform.setOrigin( osgbCollision::asBtVector3( c ) );

        _shape = new btCompoundShape;
        _shape->addChildShape( xform, collision );
    }
    virtual void apply( const osg::Cylinder& s )
    {
        osg::notify( osg::INFO ) << "Found Cylinder." << std::endl;

        osg::Vec3 c = s.getCenter();
        float radius = s.getRadius();
        float height = s.getHeight();

        btCylinderShape* collision = new btCylinderShapeZ( btVector3( radius, 0., height * .5 ) );
        btTransform xform;
        xform.setIdentity();
        //xform.setOrigin( osgbCollision::asBtVector3( c ) );

        _shape = new btCompoundShape;
        _shape->addChildShape( xform, collision );
    }

    btCompoundShape* _shape;
};

// Creates an osg::NodePath. The child-most node id in
// element zero, and the root node is at the end of the vector.
class CreateNodePath : public osg::NodeVisitor
{
public:
    CreateNodePath( osg::Node* root )
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_PARENTS ),
        _root( root )
    {}

    void apply( osg::Node& node )
    {
        osg::notify( osg::INFO ) << " CNP: " << node.getName() << std::endl;
        _p.push_back( &node );
        if (&node != _root.get())
            traverse( node );
    }

    osg::NodePath getNodePath() const
    {
        osg::notify( osg::INFO ) << " CNP: retrieving" << std::endl;
        return _p;
    }

    void clear()
    {
        _p.clear();
    }

protected:
    osg::ref_ptr< osg::Node > _root;
    osg::NodePath _p;
};

class ArticulationVisitor : public osg::NodeVisitor
{
public:
    ArticulationVisitor( btDynamicsWorld* dynamicsWorld )
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _first( true ),
        _dynamicsWorld( dynamicsWorld ),
        _shape( NULL )
    {}

    void apply( osg::Group& node )
    {
        osg::notify( osg::INFO ) << "Found Group." << std::endl;

        _cnp = new CreateNodePath( &node );
        _shape = new btCompoundShape;

        traverse( node );

        btCollisionShape* cs = accumulateChildCollisionShapes( node );

        osg::BoundingBox bb = accumulateChildBB( node );
        btTransform xform =
            osgbCollision::asBtTransform( osg::Matrix::translate( bb.center() ) );
        _shape->addChildShape( xform, cs );
        int idx = _shape->getNumChildShapes();
        osg::notify( osg::INFO ) << "  Total children " << idx << std::endl;
    }

    void apply( osg::MatrixTransform& node )
    {
        osg::notify( osg::INFO ) << "Found MatrixTransform." << std::endl;

        Joint* j;
        if (_first)
        {
            _first = false;
            j = &_joint0;
            j->_dependent = &_joint1;
            osg::notify( osg::INFO ) << " using j0 " << std::endl;
        }
        else
        {
            j = &_joint1;
            osg::notify( osg::INFO ) << " using j1 " << std::endl;
        }
        j->setMatrixTransform( &node );
        osg::notify( osg::INFO ) << std::hex << _joint0._mt.get() << " " << _joint1._mt.get() << std::dec << std::endl;

        node.accept( *_cnp );
        j->_l2wNodePath = _cnp->getNodePath();
        j->_l2wNodePath.erase( j->_l2wNodePath.begin() );
        _cnp->clear();

        traverse( node );

        // Get the BB of non-Transform children
        osg::BoundingBox bb = accumulateChildBB( node );

        // Compute the offset to the origin of both the OSG Geometry as well as the Bullet collision shape.
        j->_limbOSGOffset = osg::Vec3( bb.center()[ 0 ], bb.center()[ 1 ], bb._min[ 2 ] );
        j->_limbBTOffset = osg::Vec3( 0., 0., bb.center()[ 2 ] - bb._min[ 2 ] );

        // Create a collision shape for non-Transform children.
        btCollisionShape* cs = accumulateChildCollisionShapes( node );

        btTransform xform;
        xform.setIdentity();

        _shape->addChildShape( xform, cs );
        int idx = _shape->getNumChildShapes() - 1;
        j->setBtChildIdx( idx );
    }

    void apply( osg::Geode& node )
    {
        osg::notify( osg::INFO ) << "Found Geode." << std::endl;
        unsigned int idx;
        for ( idx=0; idx < node.getNumDrawables(); idx++ )
        {
            osg::Drawable* draw = node.getDrawable( idx );
            osg::Shape* shape = draw->getShape();
            if (shape)
            {
                BulletShapeVisitor bsv;
                shape->accept( bsv );
                if (bsv._shape)
                {
                    osgbCollision::RefBulletObject< btCollisionShape >* collision =
                        new osgbCollision::RefBulletObject< btCollisionShape >( bsv._shape );
                    node.setUserData( collision );
                }
            }
        }
        traverse( node );
    }

    btCompoundShape* getArticulationShape()
    {
        btTransform t; t.setIdentity();
        localCreateRigidBody( 0., t, _shape );

        return _shape;
    }

protected:
    bool _first;
    btDynamicsWorld* _dynamicsWorld;
    osg::ref_ptr< CreateNodePath > _cnp;

    btCompoundShape* _shape;

    btCollisionShape* accumulateChildCollisionShapes( osg::Group& node )
    {
        btCompoundShape* cs = new btCompoundShape;

        unsigned int idx;
        for ( idx=0; idx < node.getNumChildren(); idx++ )
        {
            osg::Node* child = node.getChild( idx );
            if ( dynamic_cast< osg::MatrixTransform* >( child ) )
                continue;
            osgbCollision::RefBulletObject< btCollisionShape >* bcs = dynamic_cast<
                osgbCollision::RefBulletObject< btCollisionShape >* >( child->getUserData() );
            if (!bcs)
                continue;

            btTransform xform;
            xform.setIdentity();
            cs->addChildShape( xform, bcs->get() );
        }

        return cs;
    }

    osg::BoundingBox accumulateChildBB( osg::Group& node )
    {
        osg::BoundingBox bb;

        unsigned int idx;
        for ( idx=0; idx < node.getNumChildren(); idx++ )
        {
            osg::Node* child = node.getChild( idx );
            if ( dynamic_cast< osg::MatrixTransform* >( child ) )
                continue;
            bb.expandBy( child->getBound() );
        }

        return bb;
    }

	btRigidBody* localCreateRigidBody( btScalar mass, const btTransform& startTransform,
        btCollisionShape* shape )
	{
		btVector3 localInertia( 0, 0, 0 );
		const bool isDynamic = (mass != 0.f);
		if (isDynamic)
			shape->calculateLocalInertia( mass, localInertia );

        btDefaultMotionState* myMotionState = new btDefaultMotionState( startTransform );
		btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
		btRigidBody* body = new btRigidBody( rbInfo );
        if (!isDynamic)
            body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
        body->setActivationState( DISABLE_DEACTIVATION );

		_dynamicsWorld->addRigidBody(body);

		return body;
	}
};
/* \endcond */

osg::Group*
createArm()
{
    osg::ref_ptr< osg::Group > grp = new osg::Group;
    grp->setName( "Arm root Group node" );

    osg::Box* box = new osg::Box( osg::Vec3( 0., 0., 0. ), 1., 1., 2. );
    osg::ShapeDrawable* shape = new osg::ShapeDrawable( box );
    osg::Geode* geode = new osg::Geode();
    geode->setName( "Box Geode" );
    geode->addDrawable( shape );
    grp->addChild( geode );

    osg::Cylinder* cyl = new osg::Cylinder( osg::Vec3( 0., 0., 2. ), .35, 2. );
    shape = new osg::ShapeDrawable( cyl );
    geode = new osg::Geode();
    geode->setName( "First segment Geode" );
    geode->addDrawable( shape );
    osg::MatrixTransform* mt0 = new osg::MatrixTransform;
    mt0->setName( "First segment MatrixTransform" );
    mt0->setDataVariance( osg::Object::DYNAMIC );
    mt0->addChild( geode );
    grp->addChild( mt0 );

    cyl = new osg::Cylinder( osg::Vec3( 0., 0., 4. ), .25, 2. );
    shape = new osg::ShapeDrawable( cyl );
    geode = new osg::Geode();
    geode->setName( "Second segment Geode" );
    geode->addDrawable( shape );
    osg::MatrixTransform* mt1 = new osg::MatrixTransform;
    mt1->setName( "Second segment MatrixTransform" );
    mt1->setDataVariance( osg::Object::DYNAMIC );
    mt1->addChild( geode );
    mt0->addChild( mt1 );

    return( grp.release() );
}


osg::MatrixTransform*
createBall( btDynamicsWorld* dynamicsWorld )
{
    osg::Sphere* sp = new osg::Sphere( osg::Vec3( 0., 0., 0. ), 1. );
    osg::ShapeDrawable* shape = new osg::ShapeDrawable( sp );
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( shape );
    osg::ref_ptr< osg::MatrixTransform > mt = new osg::MatrixTransform;
    mt->addChild( geode );

    BulletShapeVisitor bsv;
    sp->accept( bsv );
    btCollisionShape* collision = bsv._shape;

    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState;
    motion->setTransform( mt.get() );

    // Debug OSG rep of bullet shape.
    osg::Node* debugNode = osgbCollision::osgNodeFromBtCollisionShape( collision );
    mt->addChild( debugNode );

    btTransform bodyTransform;
    bodyTransform.setIdentity();
    bodyTransform.setOrigin( btVector3( -10, 0, 10 ) );
    motion->setWorldTransform( bodyTransform );

    btScalar mass( 1. );
    btVector3 inertia( 0., 0., 0. );
    collision->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rbinfo );
    body->setLinearVelocity( btVector3( 2, 0.1, 0 ) );
    body->setActivationState( DISABLE_DEACTIVATION );
    dynamicsWorld->addRigidBody( body );

    return( mt.release() );
}


int
main( int argc,
      char * argv[] )
{
    osg::ArgumentParser arguments( &argc, argv );

    btDynamicsWorld* dynamicsWorld = initPhysics();

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osg::ref_ptr< osg::Group > arm = createArm();
    osgDB::writeNodeFile( *arm, "arm.osg" );
    root->addChild( arm.get() );
    root->addChild( createBall( dynamicsWorld ) );

    ArticulationVisitor av( dynamicsWorld );
    arm->accept( av );
    _artShape = av.getArticulationShape();
    osg::notify( osg::INFO ) << "Finished ArticulationVisitor." << std::endl;



    float thin = .01;
    osg::MatrixTransform* ground = createOSGBox( osg::Vec3( 10, 10, thin ) );
    root->addChild( ground );
    btRigidBody* groundBody = createBTBox( ground, osg::Vec3( 0, 0, -1 ) );
    dynamicsWorld->addRigidBody( groundBody );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    osgGA::TrackballManipulator * tb = new osgGA::TrackballManipulator();
    tb->setHomePosition( osg::Vec3( 9, -21, 8 ),
                        osg::Vec3( 0, 0, 0 ),
                        osg::Vec3( 0, 0, 1 ) );
    viewer.setCameraManipulator( tb );
    viewer.addEventHandler( new ArmManipulator( dynamicsWorld ) );
    viewer.setSceneData( root.get() );
    viewer.setThreadingModel( osgViewer::ViewerBase::SingleThreaded );

    osgbCollision::GLDebugDrawer* dbgDraw = new osgbCollision::GLDebugDrawer();
    dynamicsWorld->setDebugDrawer( dbgDraw );
    root->addChild( dbgDraw->getSceneGraph() );

    double currSimTime = viewer.getFrameStamp()->getSimulationTime();
    double prevSimTime = viewer.getFrameStamp()->getSimulationTime();
    viewer.realize();
    while( !viewer.done() )
    {
        dbgDraw->BeginDraw();

        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        dynamicsWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        dynamicsWorld->debugDrawWorld();
        dbgDraw->EndDraw();

        viewer.frame();
    }

    return( 0 );
}

