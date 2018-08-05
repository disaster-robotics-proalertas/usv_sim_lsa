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

#include <osgbInteraction/HandNode.h>
#include <osgbInteraction/ArticulationRecord.h>
#include <osgbInteraction/GestureHandler.h>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/NodeVisitor>
#include <osg/NodeCallback>
#include <osg/MatrixTransform>
#include <osg/Matrix>
#include <osg/Point>
#include <osgwTools/GeometryModifier.h>
#include <osgbCollision/VertexAggOp.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>
#include <osgbDynamics/PhysicsThread.h>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include <osg/ComputeBoundsVisitor>

#include <string>
#include <osg/io_utils>


// Debug utility. Dump a NodePath.
void dumpNP( const osg::NodePath& p )
{
    osg::notify( osg::ALWAYS ) << p.size() << ": ";
    unsigned int idx;
    for( idx=0; idx<p.size(); idx++ )
        osg::notify( osg::ALWAYS ) << p[idx]->getName() << ", ";
    osg::notify( osg::ALWAYS ) << std::endl;
}


namespace osgbInteraction
{



/** \cond */
// ScaleVisitor is designed specifically to scale the hand model.
// The hand model must be scaled into an app-specified size at load time to
//   avoid the complexities of scaling in Bullet. Bullet does not support
//   scaled collision shapes. So, rather than scale with a transform as we
//   would otherwise do in OSG, we apply a scale to the whole model.
// Can't use osgUtil::Optimizer::FlattenStaticTransformVisitor because it
//   would scale the normals.
class ScaleVisitor : public osg::NodeVisitor
{
public:
    ScaleVisitor( float scale )
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _scale( scale )
    {}
    ~ScaleVisitor()
    {}

    void apply( osg::MatrixTransform& node )
    {
        osgbInteraction::ArticulationRecord* ar = dynamic_cast<
            osgbInteraction::ArticulationRecord* >( node.getUserData() );
        if( ar == NULL )
        {
            osg::notify( osg::WARN ) << "HandNode ScaleVisitor: NULL ArticulationRecord." << std::endl;
            traverse( node );
            return;
        }

        // Scale the translation portion of the matrix.
        osg::Matrix m = node.getMatrix();
        m( 3, 0 ) *= _scale;
        m( 3, 1 ) *= _scale;
        m( 3, 2 ) *= _scale;
        node.setMatrix( m );

        // Scale the pivot point.
        ar->_pivotPoint *= _scale;

        traverse( node );
    }

    void apply( osg::Geode& node )
    {
        unsigned int idx;
        for( idx=0; idx<node.getNumDrawables(); idx++ )
        {
            osg::Geometry* geom = dynamic_cast< osg::Geometry* >( node.getDrawable( idx ) );
            if( geom == NULL )
                continue;
            osg::Vec3Array* v = dynamic_cast< osg::Vec3Array* >( geom->getVertexArray() );
            if( v == NULL )
            {
                osg::notify( osg::WARN ) << "HandNode: Unexpected non-Vec3Array while scaling hand." << std::endl;
                continue;
            }
            unsigned int jdx;
            for( jdx=0; jdx<v->getNumElements(); jdx++ )
            {
                (*v)[ jdx ] *= _scale;
            }
        }
        traverse( node );
    }

protected:
    float _scale;
};

// Turn a right hand into a left hand
class LeftVisitor : public osg::NodeVisitor
{
public:
    LeftVisitor()
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
    {}
    ~LeftVisitor()
    {}

    void apply( osg::MatrixTransform& node )
    {
        osgbInteraction::ArticulationRecord* ar = dynamic_cast<
            osgbInteraction::ArticulationRecord* >( node.getUserData() );
        if( ar == NULL )
        {
            osg::notify( osg::WARN ) << "HandNode ScaleVisitor: NULL ArticulationRecord." << std::endl;
            traverse( node );
            return;
        }

        // Negate x in the translation portion of the matrix.
        osg::Matrix m = node.getMatrix();
        m( 3, 0 ) = -( m( 3, 0 ) );
        node.setMatrix( m );

        // Negate x in the pivot point.
        ar->_pivotPoint[ 0 ] = -( ar->_pivotPoint[ 0 ] );

        // Handle the rotation axis:
        // Effectively, negate the x component, then reverse the
        // entive axis. Shortened algebraically, the equivalent
        // operation is to negate y and z components.
        ar->_axis[ 1 ] = -( ar->_axis[ 1 ] );
        ar->_axis[ 2 ] = -( ar->_axis[ 2 ] );

        traverse( node );
    }

    void apply( osg::Geode& node )
    {
        unsigned int idx;
        for( idx=0; idx<node.getNumDrawables(); idx++ )
        {
            osg::Geometry* geom = dynamic_cast< osg::Geometry* >( node.getDrawable( idx ) );
            if( geom == NULL )
                continue;
            osg::Vec3Array* v = dynamic_cast< osg::Vec3Array* >( geom->getVertexArray() );
            if( v == NULL )
            {
                osg::notify( osg::WARN ) << "HandNode: Unexpected non-Vec3Array during r2l." << std::endl;
                continue;
            }
            osg::Vec3Array* n = dynamic_cast< osg::Vec3Array* >( geom->getNormalArray() );
            if( n == NULL )
            {
                osg::notify( osg::WARN ) << "HandNode: Unexpected non-Vec3Array normals during r2l." << std::endl;
                continue;
            }
            if( v->size() != n->size() )
            {
                osg::notify( osg::WARN ) << "HandNode: Different size normal and vector arrays." << std::endl;
                continue;
            }
            unsigned int jdx;
            for( jdx=0; jdx<v->getNumElements(); jdx++ )
            {
                // Negate x components.
                (*v)[ jdx ][ 0 ] = -( (*v)[ jdx ][ 0 ] );
                (*n)[ jdx ][ 0 ] = -( (*n)[ jdx ][ 0 ] );
            }
        }
        traverse( node );
    }
};




// Creates an osg::NodePath. The child-most node is in
// element zero, and the root node is at the end of the vector.
class CreateNodePath : public osg::NodeVisitor
{
public:
    CreateNodePath( osg::Node* firstNode=NULL )
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_PARENTS ),
        _node( firstNode )
    {
        if( _node != NULL )
            _p.push_back( _node );
    }

    void apply( osg::Node& node )
    {
        traverse( node );
        _p.push_back( &node );
    }
    void apply( osg::Camera& node )
    {
        // Shouldn't be here.
        osg::notify( osg::WARN ) << "HandNode: CreateNodePath encountered unexpected Camera node." << std::endl;
        // Don't include in NodePath and stop traversing.
        return;
    }

    osg::NodePath getNodePath()
    {
        return _p;
    }

protected:
    osg::NodePath _p;
    osg::Node* _node;
};

// At init time, the HandNode loads the hand model, then uses this NodeVisitor
//   to traverse the model and identify all the articulatable parts and create
//   Bullet collision shapes for them.
class FindArticulations : public osg::NodeVisitor
{
public:
    FindArticulations( HandNode* hn, HandNode::ArticulationInfoList& ail )
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _hn( hn ),
        _ail( ail )
    {
        _cs = new btCompoundShape;

        // Load the name map. Maps the names of the nodes we're looking for
        //   to the Articulation enum defined in HandNode.h.
        articulations_[ "f0trans" ] = HandNode::FINGER_0_TRANSLATE;
        articulations_[ "f1trans" ] = HandNode::FINGER_1_TRANSLATE;
        articulations_[ "f2trans" ] = HandNode::FINGER_2_TRANSLATE;
        articulations_[ "f3trans" ] = HandNode::FINGER_3_TRANSLATE;
        articulations_[ "f4trans" ] = HandNode::FINGER_4_TRANSLATE;

        articulations_[ "f0k0" ]    = HandNode::FINGER_0_ROTATE_INNER;
        articulations_[ "f1k0" ]    = HandNode::FINGER_1_ROTATE_INNER;
        articulations_[ "f2k0" ]    = HandNode::FINGER_2_ROTATE_INNER;
        articulations_[ "f3k0" ]    = HandNode::FINGER_3_ROTATE_INNER;
        articulations_[ "f4k0" ]    = HandNode::FINGER_4_ROTATE_INNER;

        articulations_[ "f0k1" ]    = HandNode::FINGER_0_ROTATE_MIDDLE;
        articulations_[ "f1k1" ]    = HandNode::FINGER_1_ROTATE_MIDDLE;
        articulations_[ "f2k1" ]    = HandNode::FINGER_2_ROTATE_MIDDLE;
        articulations_[ "f3k1" ]    = HandNode::FINGER_3_ROTATE_MIDDLE;
        articulations_[ "f4k1" ]    = HandNode::FINGER_4_ROTATE_MIDDLE;

        articulations_[ "f0k2" ]    = HandNode::FINGER_0_ROTATE_OUTER;
        articulations_[ "f1k2" ]    = HandNode::FINGER_1_ROTATE_OUTER;
        articulations_[ "f2k2" ]    = HandNode::FINGER_2_ROTATE_OUTER;
        articulations_[ "f3k2" ]    = HandNode::FINGER_3_ROTATE_OUTER;
        articulations_[ "f4k2" ]    = HandNode::FINGER_4_ROTATE_OUTER;
    }
    ~FindArticulations() {}

    void apply( osg::Group& node )
    {
        if( node.getNumParents() != 0 )
            // Should be one parent, the scale transform to set the hand length.
            osg::notify( osg::WARN ) << "HandNode: Group node has " << node.getNumParents() << " parents, should be 0." << std::endl;

        traverse( node );

        // Store info about the palm in the _palm ArticulationInfo.
        // The palm isn't articulatable like the fingers and knuckles,
        // but we need to record info about it somewhere; ArticulationInfo
        // is really a superset if the info we need to store. But it's
        // good enough for now.
        _palm._valid = true;
        _palm._mt = NULL;
        _palm._cs = _cs;
        _palm._dependent = NULL;

        CreateNodePath cnp( /*_hn*/NULL );
        node.accept( cnp );
        _palm._l2wNodePath = cnp.getNodePath();

        btCollisionShape* shape = createChildCollisionShapes( node );
        if( shape != NULL )
        {
            btTransform xform; xform.setIdentity();
            _cs->addChildShape( xform, shape );
            _palm._btChildIdx = _cs->getNumChildShapes() - 1;
        }
    }

    void apply( osg::MatrixTransform& node )
    {
        osgbInteraction::ArticulationRecord* ar = dynamic_cast<
            osgbInteraction::ArticulationRecord* >( node.getUserData() );
        if ( !ar )
        {
            osg::notify( osg::WARN ) << "HandNode: FindArticulations found a MatrixTransform that lacks an ArticulationRecord." << std::endl;
            osg::notify( osg::WARN ) << "  " << node.className() << ", " << node.getName() << std::endl;
            traverse( node );
            return;
        }

        traverse( node );

        // Use the node name to get the Articulation enum, then obtain a reference to
        //   the ArticulationInfo struct for this articulation.
        HandNode::Articulation part( HandNode::LAST_ACTUAL_ARTICULATION );
        part = articulations_[ node.getName() ];
        if (part >= HandNode::LAST_ACTUAL_ARTICULATION)
        {
            osg::notify( osg::WARN ) << "HandNode: Can't find articulation for " << node.getName() << std::endl;
            return;
        }
        HandNode::ArticulationInfo& ai( _ail[ part ] );
        ai._valid = true;

        // Fill in ArticulationInfo fields with all the required information.
        ai._mt = &node;
        ai._axis = ar->_axis;
        ai._pivotPoint = ar->_pivotPoint;
        ai._cs = _cs;

        // Find dependent AI. We can look this up using the name of the child MT.
        // For example, the dependent AI of the inner knuckle is the outer knuckle.
        //   It's "dependent" because the inner knuckle transform affects the
        //   Bullet (absolute) transform of the outer knuckle.
        ai._dependent = NULL;
        osg::MatrixTransform* childMT = findChildMT( node );
        if( childMT != NULL )
        {
            HandNode::Articulation depPart( HandNode::LAST_ACTUAL_ARTICULATION );
            depPart = articulations_[ childMT->getName() ];
            if (part < HandNode::LAST_ACTUAL_ARTICULATION)
                ai._dependent = &( _ail[ depPart ] );
        }

        // Get the NodePath, this allows the ArticulationInfo to accumulate
        //   parent transformations to create the absolute transform matrix.
        CreateNodePath cnp( /*_hn*/NULL );
        node.accept( cnp );
        ai._l2wNodePath = cnp.getNodePath();

        btCollisionShape* shape = createChildCollisionShapes( node );
        if( shape != NULL )
        {
            btTransform xform; xform.setIdentity();
            _cs->addChildShape( xform, shape );
            ai._btChildIdx = _cs->getNumChildShapes() - 1;
        }
    }

    btCompoundShape* getCollisionShape() const
    {
        return _cs;
    }
    HandNode::ArticulationInfo getPalm() const
    {
        return _palm;
    }


protected:
    typedef std::map< std::string, HandNode::Articulation > ArticulationNameMap;
    ArticulationNameMap articulations_;

    HandNode* _hn;
    HandNode::ArticulationInfoList& _ail;
    HandNode::ArticulationInfo _palm;
    btCompoundShape* _cs;

    // Create a single collision shape from the non-Transform children.
    // 'node' is input and const, but not declared const to facilitate
    //   getting its children and running a NodeVisitor on them.
    static btCollisionShape* createChildCollisionShapes( osg::Group& node )
    {
        osg::ref_ptr< osg::Group > tempRoot = new osg::Group;
        unsigned int idx;
        for ( idx=0; idx < node.getNumChildren(); idx++ )
        {
            osg::Node* child = node.getChild( idx );
            if ( dynamic_cast< osg::Transform* >( child ) )
                continue;
            tempRoot->addChild( child );
        }
        if( tempRoot->getNumChildren() == 0 )
            // No non-Transform children. Must be one of the flexure nodes.
            return NULL;

        // Create a convex hull tri mesh for this segment of the hand model.
        // Use the VertexAggOp to create a point cloud from the vertex data,
        // then create the convex hull from that point cloud.

        // Deep copy, so we don't destroy the visual model.
        osg::ref_ptr< osg::Group > aggGrp = new osg::Group( *tempRoot, osg::CopyOp::DEEP_COPY_ALL );

        osgwTools::GeometryModifier gm( new osgbCollision::VertexAggOp );
        aggGrp->accept( gm );
        return( osgbCollision::btConvexHullCollisionShapeFromOSG( aggGrp.get() ) );
    }

    static osg::MatrixTransform* findChildMT( osg::Group& grp )
    {
        unsigned int idx;
        for( idx=0; idx<grp.getNumChildren(); idx++ )
        {
            osg::MatrixTransform* mt = dynamic_cast< osg::MatrixTransform* >( grp.getChild( idx ) );
            if( mt != NULL )
                return( mt );
        }
        return NULL;
    }
};
/** \endcond */



//
//
// HandNode
//
//



// Statics
float HandNode::_defaultLength( 16.2f );


HandNode::HandNode()
  : _calibrate( false ),
    _rightOrLeft( HandNode::RIGHT ),
    _length( _defaultLength ),
    _bulletWorld( NULL ),
    _body( NULL ),
    _shape( NULL ),
    _useGhost( true ),
    _ghost( NULL ),
    _traverseHand( true ),
    _pt( NULL ),
    _debug( false )
{
    setName( "HandNode" );
    init();
}

HandNode::HandNode( btDynamicsWorld* bulletWorld, const HandNode::Handedness rightOrLeft, float handLength )
  : _calibrate( false ),
    _rightOrLeft( rightOrLeft ),
    _length( handLength ),
    _bulletWorld( bulletWorld ),
    _body( NULL ),
    _shape( NULL ),
    _useGhost( true ),
    _ghost( NULL ),
    _traverseHand( true ),
    _pt( NULL ),
    _debug( false )
{
    setName( "HandNode" );
    init();
}
HandNode::HandNode( const HandNode& rhs, const osg::CopyOp& copyop )
  : _calibrate( rhs._calibrate ),
    _requestedPosition( rhs._requestedPosition ),
    _correctedPosition( rhs._correctedPosition ),
    _rightOrLeft( rhs._rightOrLeft ),
    _attitude( rhs._attitude ),
    _length( rhs._length ),
    _bulletWorld( rhs._bulletWorld ),
    _body( NULL ),
    _shape( NULL ),
    _useGhost( rhs._useGhost ),
    _ghost( NULL ),
    _traverseHand( rhs._traverseHand ),
    _pt( rhs._pt ),
    _debug( rhs._debug )
{
    setName( rhs.getName() );
    init();
}

HandNode::~HandNode()
{
    cleanup();
}

void HandNode::cleanup()
{
    if( _hand.valid())
        _hand = NULL;

    _ail.clear();

    if( _body != NULL )
    {
        if( _bulletWorld != NULL )
            _bulletWorld->removeRigidBody( _body );
        delete _body;
        _body = NULL;
    }
    if( _shape != NULL )
    {
        // TBD possible memory leak, do we need to delete child shapes?
        delete _shape;
        _shape = NULL;
    }
    if( _ghost != NULL )
    {
        if( _bulletWorld != NULL )
            _bulletWorld->removeCollisionObject( _ghost );
        delete _ghost;
        _ghost = NULL;
    }
}

// The Transform Node and its derived classes don't define ::traverse(), they
// just get the base class Group::traverse() behavior. The CullVisitor handles
// accumulating transformation matrices.
void HandNode::traverse( osg::NodeVisitor& nv )
{
    // Traverse children, if any
    Transform::traverse( nv );

    // Traverse the hand, if it's valid (it should be valid; see init() ).
    if (_traverseHand && _hand.valid())
        _hand->accept( nv );
}
osg::BoundingSphere HandNode::computeBound() const
{
    // Get the bounding sphere of any children;
    osg::BoundingSphere childBS = Transform::computeBound();

    if( !_traverseHand || !( _hand.valid() ) )
        return childBS;

    // Get the hand bounding sphere.
    osg::BoundingSphere bsphere = _hand->computeBound();

    // Transform the hand bounding sphere by PAT parameters.
    osg::Matrix l2w;
    computeLocalToWorldMatrix(l2w,NULL);

    osg::Vec3 xdash = bsphere._center;
    xdash.x() += bsphere._radius;
    xdash = xdash*l2w;

    osg::Vec3 ydash = bsphere._center;
    ydash.y() += bsphere._radius;
    ydash = ydash*l2w;

    osg::Vec3 zdash = bsphere._center;
    zdash.z() += bsphere._radius;
    zdash = zdash*l2w;


    bsphere._center = bsphere._center*l2w;

    xdash -= bsphere._center;
    float len_xdash = xdash.length();

    ydash -= bsphere._center;
    float len_ydash = ydash.length();

    zdash -= bsphere._center;
    float len_zdash = zdash.length();

    bsphere._radius = len_xdash;
    if (bsphere._radius<len_ydash) bsphere._radius = len_ydash;
    if (bsphere._radius<len_zdash) bsphere._radius = len_zdash;


    if (childBS.valid())
    {
        // Expand BS of children by transformed BS of the hand.
        childBS.expandBy( bsphere );
        return childBS;
    }
    else
    {
        // Just return the reansformed hand BS.
        return bsphere;
    }
}

bool HandNode::computeLocalToWorldMatrix( osg::Matrix& matrix, osg::NodeVisitor*) const
{
    const osg::Matrix l2w(
        osg::Matrix::rotate( _attitude ) *
        osg::Matrix::translate( _correctedPosition ) );

    if( _referenceFrame==RELATIVE_RF )
        matrix.preMult( l2w );
    else // absolute
        matrix = l2w;
    return true;
}

bool HandNode::computeWorldToLocalMatrix( osg::Matrix& matrix, osg::NodeVisitor* ) const
{
    const osg::Matrix w2l(
        osg::Matrix::translate( -_correctedPosition ) *
        osg::Matrix::rotate( _attitude.inverse() ) );

    if (_referenceFrame==RELATIVE_RF)
        matrix.postMult( w2l );
    else // absolute
        matrix = w2l;
    return true;
}

void
HandNode::setAll( const AllParams& params )
{
    //
    // Handle the position change.
    osg::Vec3 deltaPosition( 0., 0., 0. );
    if( _correctedPosition == _requestedPosition )
    {
        // They're in sync, so no special tracking or adjustments required.
        _correctedPosition = _requestedPosition = params._pos;
    }
    else
    {
        // We're not in sync. Pass delta motion to updateTransform
        // and let it adjust the _correctedPosition.
        deltaPosition = params._pos - _requestedPosition;
        _requestedPosition = params._pos;
    }

    //
    // Handle the attitude change.
    _attitude = params._att;


    if( _pt != NULL )
        _pt->pause( true );

    //
    // Set all articulations
    setArticulationInternal( SPREAD_0_1, params._spread01 );
    setArticulationInternal( SPREAD_1_2, params._spread12 );
    setArticulationInternal( SPREAD_2_3, params._spread23 );
    setArticulationInternal( SPREAD_3_4, params._spread34 );
    setArticulationInternal( FINGER_0_ROTATE_MIDDLE, params._finger0[ 0 ] );
    setArticulationInternal( FINGER_0_ROTATE_OUTER, params._finger0[ 1 ] );
    setArticulationInternal( FINGER_1_ROTATE_INNER, params._finger1[ 0 ] );
    setArticulationInternal( FINGER_1_MIDDLE_OUTER, params._finger1[ 1 ] );
    setArticulationInternal( FINGER_2_ROTATE_INNER, params._finger2[ 0 ] );
    setArticulationInternal( FINGER_2_MIDDLE_OUTER, params._finger2[ 1 ] );
    setArticulationInternal( FINGER_3_ROTATE_INNER, params._finger3[ 0 ] );
    setArticulationInternal( FINGER_3_MIDDLE_OUTER, params._finger3[ 1 ] );
    setArticulationInternal( FINGER_4_ROTATE_INNER, params._finger4[ 0 ] );
    setArticulationInternal( FINGER_4_MIDDLE_OUTER, params._finger4[ 1 ] );

    //
    // Update Bullet data
    adjustPositionInternal( deltaPosition );
    updateTransformInternal();

    if( _pt != NULL )
        _pt->pause( false );

    //
    // Dirty the bounding box.
    dirtyBound();
}
void
HandNode::getAll( AllParams& params ) const
{
    params._pos = getPosition();
    params._att = getAttitude();
    params._spread01 = getArticulation( SPREAD_0_1 );
    params._spread12 = getArticulation( SPREAD_1_2 );
    params._spread23 = getArticulation( SPREAD_2_3 );
    params._spread34 = getArticulation( SPREAD_3_4 );
    params._finger0[ 0 ] = getArticulation( FINGER_0_ROTATE_MIDDLE );
    params._finger0[ 1 ] = getArticulation( FINGER_0_ROTATE_OUTER );
    params._finger1[ 0 ] = getArticulation( FINGER_1_ROTATE_INNER );
    params._finger1[ 1 ] = getArticulation( FINGER_1_MIDDLE_OUTER );
    params._finger2[ 0 ] = getArticulation( FINGER_2_ROTATE_INNER );
    params._finger2[ 1 ] = getArticulation( FINGER_2_MIDDLE_OUTER );
    params._finger3[ 0 ] = getArticulation( FINGER_3_ROTATE_INNER );
    params._finger3[ 1 ] = getArticulation( FINGER_3_MIDDLE_OUTER );
    params._finger4[ 0 ] = getArticulation( FINGER_4_ROTATE_INNER );
    params._finger4[ 1 ] = getArticulation( FINGER_4_MIDDLE_OUTER );
}

void
HandNode::setPosition( const osg::Vec3& pos )
{
    osg::notify( osg::INFO ) << "HandNode::setPosition() is deprecated. Use setAll()." << std::endl;

    osg::Vec3 delta( 0., 0., 0. );
    if( _correctedPosition == _requestedPosition )
    {
        // They're in sync, so no special tracking or adjustments required.
        _correctedPosition = _requestedPosition = pos;
    }
    else
    {
        // We're not in sync. Pass delta motion to updateTransform
        // and let it adjust the _correctedPosition.
        delta = pos - _requestedPosition;
        _requestedPosition = pos;
    }

    updateTransformAndAdjustPosition( delta );
    dirtyBound();
}
void
HandNode::setAttitude( const osg::Quat& quat )
{
    osg::notify( osg::INFO ) << "HandNode::setAttitude() is deprecated. Use setAll()." << std::endl;

    _attitude = quat;
    updateTransformAndAdjustPosition();
    dirtyBound();
}



// Use the ghost body to potentially adjust the position of the hand
// to avoid collisions (with static objects).
// If the position gets adjusted, also update the rigid body position.
void
HandNode::adjustPosition()
{
    // Block physics while modifying the collision shape matrices.
    if( _pt != NULL )
        _pt->pause( true );

    if( adjustPositionInternal() )
        updateTransformInternal();

    // Unblock physics
    if( _pt != NULL )
        _pt->pause( false );
}

// Use the ghost body to potentially adjust the position of the hand
// to avoid collisions (with static objects). Also set the rigid body position.
void
HandNode::updateTransformAndAdjustPosition( const osg::Vec3& deltaMotion )
{
    // Block physics while modifying the collision shape matrices.
    if( _pt != NULL )
        _pt->pause( true );

    adjustPositionInternal( deltaMotion);
    updateTransformInternal();

    // Unblock physics
    if( _pt != NULL )
        _pt->pause( false );
}

// Assumes physics thread is paused.
bool
HandNode::adjustPositionInternal( const osg::Vec3& deltaMotion )
{
    if( _ghost == NULL )
        return( false );

    if( _correctedPosition != _requestedPosition )
    {
        // They're not in sync. We want to 'pull' the _correctedPosition
        // towards the _requestedPosition.
        osg::Vec3 pull = _requestedPosition - _correctedPosition;
        // How aggressively do we pull? Larger values -> more aggressive.
        float aggressiveness = 3.f;

        float dot = pull * deltaMotion;
        osg::notify( osg::ALWAYS ) << "adjustPositionInternal " << dot << " " << pull << std::endl;

        if( dot > 0. )
        {
            // Delta motion is towards the _requestedPosition.
            _correctedPosition += ( deltaMotion * aggressiveness );
        }
        else
        {
            if( dot == 0. )
            {
                // Delta motion is zero. Move in the pull direction.
                _correctedPosition += ( pull * .05 );
            }
            else
            {
                // Delta motion is away from _requestedPosition.
                // Mirror the delta vector towards the _requestedPosition.
                _correctedPosition += ( deltaMotion + ( pull * dot * -2. ) * aggressiveness );
            }
        }
    }

    // Compute transform matrix.
    osg::Matrix m;
    computeLocalToWorldMatrix( m, NULL );
    btTransform btm( osgbCollision::asBtTransform( m ) );

    // Update ghost object position.
    _ghost->setWorldTransform( btm );

    // Check for and recover from penetrations.
    int numPenetrationLoops = 0;
    while( recoverFromPenetration() )
    {
        // We moved. Recompute the matrix.
        m.set( osg::Matrix::identity() );
        computeLocalToWorldMatrix( m, NULL );
        btm = osgbCollision::asBtTransform( m );
        // and update the ghost object position again.
        _ghost->setWorldTransform( btm );

        numPenetrationLoops++;
        if( numPenetrationLoops > 4 )
        {
            osg::notify( osg::DEBUG_FP ) << "HandNode could not recover from penetrations:" << numPenetrationLoops << std::endl;
            break;
        }
    }

    if( ( numPenetrationLoops == 0 ) &&
        ( _correctedPosition != _requestedPosition ) )
    {
        // There were no penetrations, but they are different points,
        // so immediately 'warp' to the requested position. NOTE: If there's
        // something in the way, this technique will tunnel through it.
        _correctedPosition = _requestedPosition;
    }

    // Debug
    if( _debugVerts.valid() )
    {
        osg::Matrix w2l;
        computeWorldToLocalMatrix( w2l, NULL );
        (*_debugVerts)[ 0 ] = _requestedPosition * w2l;
    }

    // Return true if we adjusted the position.
    return( numPenetrationLoops != 0 );
}

// Assumes physics thread is paused.
void
HandNode::updateTransformInternal( const osg::Vec3& deltaMotion )
{
    if( _bulletWorld == NULL )
        return;

    // Compute transform matrix.
    osg::Matrix m;
    computeLocalToWorldMatrix( m, NULL );
    btTransform btm( osgbCollision::asBtTransform( m ) );

    // Update the rigid body position.
    _body->setCenterOfMassTransform( btm );

    // Update the Bullet transform for all component collision shapes.
    int idx;
    for( idx=0; idx<LAST_ACTUAL_ARTICULATION; idx++ )
        _ail[ idx ].setBulletTransform();
    _palm.setBulletTransform();
}


// Adapted from btKinematicCharacterController::recoverFromPenetration
bool
HandNode::recoverFromPenetration()
{
    btHashedOverlappingPairCache* opc = _ghost->getOverlappingPairCache();
    _bulletWorld->getDispatcher()->dispatchAllCollisionPairs(
        opc, _bulletWorld->getDispatchInfo(), _bulletWorld->getDispatcher() );

    btVector3 position = osgbCollision::asBtVector3( _correctedPosition );

    bool penetration = false;
    for( int idx=0; idx < opc->getNumOverlappingPairs(); idx++ )
    {
        _manifoldArray.clear();

        btBroadphasePair* collisionPair = &opc->getOverlappingPairArray()[ idx ];
        if( collisionPair->m_algorithm )
            collisionPair->m_algorithm->getAllContactManifolds( _manifoldArray );
        
        for( int jdx=0; jdx < _manifoldArray.size(); jdx++ )
        {
            btPersistentManifold* manifold = _manifoldArray[ jdx ];
            btScalar directionSign = manifold->getBody0() == _ghost ? btScalar( -1.0 ) : btScalar( 1.0 );
            for( int pdx=0; pdx < manifold->getNumContacts(); pdx++ )
            {
                const btManifoldPoint& pt = manifold->getContactPoint( pdx );
                if( pt.getDistance() < 0.0 )
                {
                    position += pt.m_normalWorldOnB * directionSign * pt.getDistance() * btScalar( 0.2 );
                    penetration = true;
                }
            }
        }
    }
    _correctedPosition = osgbCollision::asOsgVec3( position );

    return( penetration );
}

void
HandNode::setUseGhost( bool useGhost )
{
    if( _useGhost != useGhost )
    {
        _useGhost = useGhost;
        if( !_useGhost && ( _ghost != NULL ) )
        {
            if( _bulletWorld != NULL )
                _bulletWorld->removeCollisionObject( _ghost );
            delete _ghost;
            _ghost = NULL;
        }
        else
            init();
    }
}
bool
HandNode::getUseGhost() const
{
    return( _useGhost );
}



void HandNode::init()
{
    // Start from a clean slate.
    cleanup();

    // Load either the right or left hand.
    std::string fileName( "hand.osg" );
    char* envName;
    if( ( envName = getenv( "OSGBINTERACTION_HAND_FILENAME" ) ) != NULL )
    {
        osg::notify( osg::INFO ) << "HandNode: OSGBINTERACTION_HAND_FILENAME overrides default file name." << std::endl;
        fileName = std::string( envName );
    }
    osg::notify( osg::INFO ) << "HandNode: Attempting to load \"" << fileName << "\"..." << std::endl;
    _hand = osgDB::readNodeFile( fileName );
    if( !_hand.valid() )
    {
        osg::notify( osg::FATAL ) << "HandNode: Can't load \"" << fileName << "\". Check osgDB data file search path." << std::endl;
        return;
    }
    osg::notify( osg::INFO ) << "HandNode: \"" << fileName << "\" loaded successfully." << std::endl;

    if (_rightOrLeft == LEFT)
    {
        // Scale by -1 in x.
        // TBD change order of vertices.
        LeftVisitor lv;
        _hand->accept( lv );
    }

    // Scale the hand to the correct length.
    ScaleVisitor sv( _length / _defaultLength );
    _hand->accept( sv );

    // Run the FindArticulations visitor. This loads the ArticulationInfoList
    // and creates child shapes for our "_shape" compund shape.
    osg::notify( osg::INFO ) << "HandNode: Finding articulations." << std::endl;
    _ail.resize( LAST_ACTUAL_ARTICULATION );
    FindArticulations fa( this, _ail );
    _hand->accept( fa );

    // FindArticulations created a compund collision shape for us;
    // save the address.
#ifdef USE_SIMPLE_BOX
    _shape = new btBoxShape( btVector3( 1.5, 1.5, 1.5 ) );
#else
    _shape = fa.getCollisionShape();
#endif
    _palm = fa.getPalm();

    // Block physics while modifying the collision shape matrices.
    if( _pt != NULL )
        _pt->pause( true );

    // Create a rigid body from the compound shape and add it to the
    // Bullet dynamics world.
    if (_bulletWorld != NULL)
    {
        // Create rigid body and add to bullet world.
        btTransform xform; xform.setIdentity();
        btVector3 inertia( 0, 0, 0 );
        btRigidBody::btRigidBodyConstructionInfo rbInfo( 0., NULL, _shape, inertia );
        rbInfo.m_friction = btScalar( 1. );
        _body = new btRigidBody( rbInfo );
        _body->setCollisionFlags( _body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
        _body->setActivationState( DISABLE_DEACTIVATION );

        btDiscreteDynamicsWorld* ddw = dynamic_cast< btDiscreteDynamicsWorld* >( _bulletWorld );
        if( ddw != NULL )
            ddw->addRigidBody( _body, btBroadphaseProxy::KinematicFilter, ~btBroadphaseProxy::CharacterFilter );
        else
            _bulletWorld->addRigidBody( _body );


        //
        // Ghost setup.
        if( _useGhost )
        {
	        _ghost = new btPairCachingGhostObject();
            _bulletWorld->getPairCache()->setInternalGhostPairCallback( new btGhostPairCallback() );

            // Hm, this has issues. Possibly change this in the future?
            _ghost->setCollisionShape( _shape );
            // For debugging / development:
            //_ghost->setCollisionShape( new btBoxShape( btVector3( _length*.5, _length, _length*.2 ) ) );

	        _ghost->setCollisionFlags( btCollisionObject::CF_CHARACTER_OBJECT );
            // We're only interested in collisions with STATIC objects. (I think.)
    	    _bulletWorld->addCollisionObject( _ghost, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter );
        }
    }

    // Set the initial Bullet transformation matrices.
    updateTransformInternal();

#ifdef USE_SIMPLE_BOX
    return;
#else
    // Let Bullet know the size of the new Aabb.
    if (_bulletWorld != NULL)
        _shape->recalculateLocalAabb();
#endif

    // Unblock physics.
    if( _pt != NULL )
        _pt->pause( false );

    // Debug. Display the _requestedPosition.
    if( _debug )
    {
        osg::Geode* geode = new osg::Geode;
        addChild( geode );

        osg::Geometry* geom = new osg::Geometry;
        geom->setUseDisplayList( false );
        _debugVerts = new osg::Vec3Array;
        _debugVerts->resize( 1 );
        geom->setVertexArray( _debugVerts.get() );
        osg::Vec4Array* c = new osg::Vec4Array;
        c->push_back( osg::Vec4( 1., 0., 0., 1. ) ); // draw requested position in red.
        geom->setColorArray( c );
        geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
        geom->addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, 1 ) );
        geom->setInitialBound( *( new osg::BoundingBox( -1., -1., -1., 1., 1., 1. ) ) );
        geode->addDrawable( geom );

        osg::StateSet* ss = geode->getOrCreateStateSet();
        ss->setAttributeAndModes( new osg::Point( 17.f ) );
        ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    }
}


void HandNode::setHandedness( const HandNode::Handedness rightOrLeft )
{
    if ( _rightOrLeft != rightOrLeft )
    {
        _rightOrLeft = rightOrLeft;
        init();
    }
}
HandNode::Handedness HandNode::getHandedness() const
{
    return _rightOrLeft;
}

void
HandNode::setHandLength( float length )
{
    if ( _length != length )
    {
        _length = length;
        init();
    }
}

float
HandNode::getHandLength() const
{
    return _length;
}


void
HandNode::setArticulation( const HandNode::Articulation part, const float radians )
{
    osg::notify( osg::INFO ) << "HandNode::setArticulation() is deprecated. Use setAll()." << std::endl;

    // TBD remove
#if 0
    if( _calibrate )
    {
        // In calibrate mode, the hand doesn't move in response to articulation changes.
        // Instead, incoming angles are stored as a calibration angle. When not in calibration
        // mode, the calibration angle is subtracted from the input angle when setting
        // the transformation matrix.
        if( part >= LAST_ACTUAL_ARTICULATION )
            setVirtualArticulation( part, radians );
        else
            _ail[ part ]._calibrateBaseAngle = radians;
        return;
    }
#endif

    // Block physics while modifying the collision shape matrices.
    if( _pt != NULL )
        _pt->pause( true );

    setArticulationInternal( part, radians );

    if( adjustPositionInternal() )
        updateTransformInternal();

    if( _pt != NULL )
        _pt->pause( false );
}
void
HandNode::setArticulationInternal( const HandNode::Articulation part, const float radians )
{
    if( part >= LAST_ACTUAL_ARTICULATION )
        setVirtualArticulation( part, radians );
    else if( _calibrate )
        _ail[ part ]._calibrateBaseAngle = radians;
    else
        _ail[ part ].setAngle( radians );
}
float HandNode::getArticulation( const HandNode::Articulation part ) const
{
    if( part >= LAST_ACTUAL_ARTICULATION )
        return( getVirtualArticulation( part ) );

    else if( _calibrate )
        return( _ail[ part ]._calibrateBaseAngle );
    else
        return( _ail[ part ].getAngle() );
}

void
HandNode::setVirtualArticulation( const HandNode::Articulation part, const float radians )
{
    typedef std::map< Articulation, float > ValueMap;
    ValueMap _valueMap;

    float specifiedAngle( radians );

    float f1, f2, f3;
    if( _calibrate )
    {
        f1 = _ail[ FINGER_1_TRANSLATE ]._calibrateBaseAngle;
        f2 = _ail[ FINGER_2_TRANSLATE ]._calibrateBaseAngle;
        f3 = _ail[ FINGER_3_TRANSLATE ]._calibrateBaseAngle;
    }
    else
    {
        f1 = _ail[ FINGER_1_TRANSLATE ].getAngle();
        f2 = _ail[ FINGER_2_TRANSLATE ].getAngle();
        f3 = _ail[ FINGER_3_TRANSLATE ].getAngle();
    }

    switch( (int)part )
    {
    case SPREAD_0_1:
    {
        const float base = f1;
        _valueMap[ FINGER_0_TRANSLATE ] = base + specifiedAngle;
        break;
    }
    case SPREAD_1_2:
    {
        const float spread01 = getVirtualArticulation( SPREAD_0_1 );
        specifiedAngle += f2;
        _valueMap[ FINGER_1_TRANSLATE ] = specifiedAngle;
        _valueMap[ FINGER_0_TRANSLATE ] = specifiedAngle + spread01;
        break;
    }
    case SPREAD_2_3:
    {
        const float spread34 = getVirtualArticulation( SPREAD_3_4 );
        specifiedAngle -= f2;
        _valueMap[ FINGER_3_TRANSLATE ] = -specifiedAngle;
        _valueMap[ FINGER_4_TRANSLATE ] = -( specifiedAngle + spread34 );
        break;
    }
    case SPREAD_3_4:
    {
        const float base = f3;
        _valueMap[ FINGER_4_TRANSLATE ] = base - specifiedAngle;
        break;
    }

    case FINGER_1_MIDDLE_OUTER:
    {
        _valueMap[ FINGER_1_ROTATE_MIDDLE ] = specifiedAngle;
        _valueMap[ FINGER_1_ROTATE_OUTER ] = specifiedAngle;
        break;
    }
    case FINGER_2_MIDDLE_OUTER:
    {
        _valueMap[ FINGER_2_ROTATE_MIDDLE ] = specifiedAngle;
        _valueMap[ FINGER_2_ROTATE_OUTER ] = specifiedAngle;
        break;
    }
    case FINGER_3_MIDDLE_OUTER:
    {
        _valueMap[ FINGER_3_ROTATE_MIDDLE ] = specifiedAngle;
        _valueMap[ FINGER_3_ROTATE_OUTER ] = specifiedAngle;
        break;
    }
    case FINGER_4_MIDDLE_OUTER:
    {
        _valueMap[ FINGER_4_ROTATE_MIDDLE ] = specifiedAngle;
        _valueMap[ FINGER_4_ROTATE_OUTER ] = specifiedAngle;
        break;
    }
    default:
        osg::notify( osg::WARN ) << "HandNode setVirtualArticulation: invalid articulation enum: " << part << std::endl;
        break;
    }

    ValueMap::const_iterator it;
    if( _calibrate )
        for( it = _valueMap.begin(); it != _valueMap.end(); ++it )
            _ail[ it->first ]._calibrateBaseAngle = it->second;
    else
        for( it = _valueMap.begin(); it != _valueMap.end(); ++it )
            _ail[ it->first ].setAngle( it->second );
}
float
HandNode::getVirtualArticulation( const HandNode::Articulation part ) const
{
    float retAngle( 0.f );

    switch( part )
    {
    case SPREAD_0_1:
    {
        if( _calibrate )
            retAngle = _ail[ FINGER_0_TRANSLATE ]._calibrateBaseAngle -
                _ail[ FINGER_1_TRANSLATE ]._calibrateBaseAngle;
        else
            retAngle = _ail[ FINGER_0_TRANSLATE ].getAngle() -
                _ail[ FINGER_1_TRANSLATE ].getAngle();
        break;
    }
    case SPREAD_1_2:
    {
        if( _calibrate )
            retAngle = _ail[ FINGER_1_TRANSLATE ]._calibrateBaseAngle -
                _ail[ FINGER_2_TRANSLATE ]._calibrateBaseAngle;
        else
            retAngle = _ail[ FINGER_1_TRANSLATE ].getAngle() -
                _ail[ FINGER_2_TRANSLATE ].getAngle();
        break;
    }
    case SPREAD_2_3:
    {
        if( _calibrate )
            retAngle = _ail[ FINGER_2_TRANSLATE ]._calibrateBaseAngle -
                _ail[ FINGER_3_TRANSLATE ]._calibrateBaseAngle;
        else
            retAngle = _ail[ FINGER_2_TRANSLATE ].getAngle() -
                _ail[ FINGER_3_TRANSLATE ].getAngle();
        break;
    }
    case SPREAD_3_4:
    {
        if( _calibrate )
            retAngle = _ail[ FINGER_3_TRANSLATE ]._calibrateBaseAngle -
                _ail[ FINGER_4_TRANSLATE ]._calibrateBaseAngle;
        else
            retAngle = _ail[ FINGER_3_TRANSLATE ].getAngle() -
                _ail[ FINGER_4_TRANSLATE ].getAngle();
        break;
    }

    // NOTE: Queries for MIDDLE_OUTER assumes both actual articulations
    // are set the same, and just returns the value for MIDDLE, and is
    // therefor equivalent to calling:
    //   getArticulation( FINGER_n_ROTATE_MIDDLE );
    case FINGER_1_MIDDLE_OUTER:
    {
        if( _calibrate )
            retAngle = _ail[ FINGER_1_ROTATE_MIDDLE ]._calibrateBaseAngle;
        else
            retAngle = _ail[ FINGER_1_ROTATE_MIDDLE ].getAngle();
        break;
    }
    case FINGER_2_MIDDLE_OUTER:
    {
        if( _calibrate )
            retAngle = _ail[ FINGER_2_ROTATE_MIDDLE ]._calibrateBaseAngle;
        else
            retAngle = _ail[ FINGER_2_ROTATE_MIDDLE ].getAngle();
        break;
    }
    case FINGER_3_MIDDLE_OUTER:
    {
        if( _calibrate )
            retAngle = _ail[ FINGER_3_ROTATE_MIDDLE ]._calibrateBaseAngle;
        else
            retAngle = _ail[ FINGER_3_ROTATE_MIDDLE ].getAngle();
        break;
    }
    case FINGER_4_MIDDLE_OUTER:
    {
        if( _calibrate )
            retAngle = _ail[ FINGER_4_ROTATE_MIDDLE ]._calibrateBaseAngle;
        else
            retAngle = _ail[ FINGER_4_ROTATE_MIDDLE ].getAngle();
        break;
    }
    default:
        osg::notify( osg::WARN ) << "HandNode setVirtualArticulation: invalid articulation enum: " << part << std::endl;
        break;
    }

    return( retAngle );
}


void
HandNode::setCalibrateMode( const bool calibrate )
{
    if( calibrate )
    {
        // Entering calibration mode.
        // Go to the default position. Do not use setPose() due to animations.
        int idx;
        for( idx=0; idx<LAST_ACTUAL_ARTICULATION; idx++ )
            _ail[ idx ].setAngle( 0.f );
    }

    _calibrate = calibrate;

    if( !_calibrate )
    {
        // Set all articulations with new calibration values.
        int idx;
        for( idx=0; idx<LAST_ACTUAL_ARTICULATION; idx++ )
            setArticulation( idx, getArticulation( idx ) );
    }
}
bool
HandNode::getCalibrateMode() const
{
    return( _calibrate );
}


void
HandNode::setGestureHandlerVector( osgbInteraction::GestureHandlerVector ghv )
{
    _ghv = ghv;
}
osgbInteraction::GestureHandlerVector&
HandNode::getGestureHandlerVector()
{
    return( _ghv );
}
void
HandNode::sendGestureCode( const unsigned int gestureCode )
{
    osg::notify( osg::INFO ) << "Got code: " << gestureCode << std::endl;

    if( _pt != NULL )
        _pt->pause( true );

    GestureHandlerVector::iterator itr;
    for( itr=_ghv.begin(); itr != _ghv.end(); ++itr )
    {
        GestureHandler* gh = (*itr).get();
        if( (*gh)( gestureCode, *this ) )
            // Gesture code was handled, don't send to
            // any other GestureHandler objects.
            break;
    }

    if( _pt != NULL )
        _pt->pause( false );
}

btRigidBody*
HandNode::findClosest() const
{
    // NOTE: Assumes physics thread is already stopped.

    btTransform handInverse = _body->getWorldTransform().inverse();
    btScalar minDistance = FLT_MAX;
    btRigidBody* closest( NULL );

    // Iterate over all collitions objects and find the one
    // that is closest to the hand, and not static.
    const btCollisionObjectArray& coa = _bulletWorld->getCollisionObjectArray();
    osg::notify( osg::INFO ) << "Looking through " << coa.size() << " collision objects." << std::endl;
    int idx;
    for( idx=0; idx<coa.size(); idx++ )
    {
        btCollisionObject* co = coa[ idx ];
        if( co == _body )
            continue; // Don't consider the hand object.
        if( ( co->getCollisionFlags() & btCollisionObject::CF_STATIC_OBJECT ) != 0 )
            continue; // Don't consider static objects.

        btRigidBody* rb = dynamic_cast< btRigidBody* >( co );
        if( rb == NULL )
            continue;

        // Get the distance.
        btTransform xformToRB = handInverse * rb->getWorldTransform();
        btScalar distance = xformToRB.getOrigin().length2();
        if( distance < minDistance )
        {
            minDistance = distance;
            closest = rb;
        }
    }

    return( closest );
}



// Support for moving to predefined hand poses.
// Possibly consider moving this class out to its own .cpp/.h files?
class MoveToPose : public osg::NodeCallback
{
public:
    MoveToPose( HandNode* hn, HandNode::Pose pose, float radiansPerSec )
      : _hn( hn ),
        _rate( radiansPerSec ),
        _lastTime( DBL_MAX ),
        _target( NULL )
    {
        switch( pose )
        {
        case HandNode::POSE_HOOK:
            _target = _poseHook;
            break;
        case HandNode::POSE_POINT:
            _target = _posePoint;
            break;
        case HandNode::POSE_FIST:
            _target = _poseFist;
            break;
        default:
            _target = _poseDefault;
            break;
        }
    }
    ~MoveToPose()
    {
    }

    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        const double time( nv->getFrameStamp()->getSimulationTime() );
        if( _lastTime == DBL_MAX )
        {
            // First frame; record the time and continue.
            _lastTime = time;
            return;
        }

        // Calculate the articulation radians as specified rate * elapsed time.
        const double delta( _rate * (time - _lastTime) );
        _lastTime = time;

        bool done( true );
        int idx;
        for( idx=0; idx<HandNode::LAST_ACTUAL_ARTICULATION; idx++ )
        {
            const double error( _target[ idx ] - _hn->getArticulation( idx ) );
            if( error == 0. ) // Already in position.
                continue;

            float setTo;
            if( error < -delta )
                setTo = _hn->getArticulation( idx ) - delta;
            else if( error > delta )
                setTo = _hn->getArticulation( idx ) + delta;
            else
                setTo = _target[ idx ];
            _hn->setArticulation( idx, setTo );
            done = false;
        }
        traverse( node, nv );

        if( done )
            // Everything is in position, remove the update callback.
            _hn->setUpdateCallback( NULL );
    }

protected:
    HandNode* _hn;
    float _rate;
    double _lastTime;

    float* _target;
    static float _poseDefault[ HandNode::LAST_ACTUAL_ARTICULATION ];
    static float _poseHook[ HandNode::LAST_ACTUAL_ARTICULATION ];
    static float _posePoint[ HandNode::LAST_ACTUAL_ARTICULATION ];
    static float _poseFist[ HandNode::LAST_ACTUAL_ARTICULATION ];
};

float MoveToPose::_poseDefault[] = {
//  F0   F1   F2   F3   F4
    0.f, 0.f, 0.f, 0.f, 0.f, // translation / flexure
    0.f, 0.f, 0.f, 0.f, 0.f, // inner knuckle
    0.f, 0.f, 0.f, 0.f, 0.f,  // middle knuckle
    0.f, 0.f, 0.f, 0.f, 0.f  // outer knuckle
};
float MoveToPose::_poseHook[] = {
    0.1f, 0.f, -0.2f, -0.1f, 0.f,
    -0.4f, 0.f, 1.05f, 1.f, 1.1f,
    0.7f, 1.f, 1.55f, 1.55f, 1.55f,
    0.1f, 0.4f, 1.55f, 1.55f, 1.55f
};
float MoveToPose::_posePoint[] = {
    0.1f, 0.f, -0.2f, -0.1f, 0.f,
    -0.4f, 0.f, 1.05f, 1.f, 1.1f,
    0.7f, 0.f, 1.55f, 1.55f, 1.55f,
    0.1f, 0.f, 1.55f, 1.55f, 1.55f
};
float MoveToPose::_poseFist[] = {
    0.1f, -0.2f, -0.2f, -0.1f, 0.f,
    -0.15f, 1.f, 1.05f, 1.f, 1.1f,
    0.55f, 1.55f, 1.55f, 1.55f, 1.55f,
    0.f, 1.55f, 1.55f, 1.55f, 1.55f
};

void HandNode::setPose( Pose pose, float radiansPerSec )
{
    setUpdateCallback( new MoveToPose( this, pose, radiansPerSec ) );
}


void HandNode::registerMultiThreaded( osgbDynamics::PhysicsThread* pt )
{
    _pt = pt;
}



HandNode::ArticulationInfo::ArticulationInfo()
  : _btChildIdx( -1 ),
    _angle( 0.f ),
    _calibrateBaseAngle( 0.f ),
    _dependent( NULL ),
    _cs( NULL ),
    _valid( false )
{
}
HandNode::ArticulationInfo::~ArticulationInfo()
{
}

void HandNode::ArticulationInfo::dump() const
{
    osg::notify( osg::ALWAYS ) <<
        "  _btChildIdx: " << _btChildIdx <<
        //"\t_angle: " << _angle <<
        "\t_cal: " << _calibrateBaseAngle <<
        "\t_dependent: " << _dependent <<
        "\t_cs: " << _cs <<
        "\t_mt: " << _mt.get() <<
        "\t_axis: " << _axis <<
        "\t_pivotPoint: " << _pivotPoint <<
        std::endl;
    osg::Matrix m;
    m = osg::computeLocalToWorld( _l2wNodePath );
    osg::notify( osg::ALWAYS ) << m << std::endl;
}

void HandNode::ArticulationInfo::setAngle( float angle )
{
    if( !_mt.valid() )
    {
        osg::notify( osg::WARN ) << "HandNode: Articulation has invalid MatrixTransform." << std::endl;
        return;
    }

    _angle = angle;

    _mt->setMatrix(
        osg::Matrix::rotate( _angle - _calibrateBaseAngle, _axis ) *
        osg::Matrix::translate( _pivotPoint )
    );

    // If this is an inner transform (close to the palm), then let's update
    // the dependent transforms further out. Even though their angles haven't
    // changed, their absolute matrices have changed.
    if( _dependent != NULL)
        _dependent->setAngle( _dependent->getAngle() );

    setBulletTransform();
}
float HandNode::ArticulationInfo::getAngle() const
{
    return( _angle );
}

void HandNode::ArticulationInfo::setBulletTransform()
{
    if( _btChildIdx >= 0 )
    {
        // Get the absolute transform for the Bullet shape and update
        // the Bullet shape transformation within the larger compound shape.
        osg::Matrix l2w = osg::computeLocalToWorld( _l2wNodePath );
        // This line of code is the fix for the hand interactions we've been seeing.
        // Apparently a Bullet interface change sometime between 2.70 and 2.72.
        _cs->updateChildTransform( _btChildIdx, osgbCollision::asBtTransform( l2w ) );
    }

    // Let Bullet know the size of the new Aabb.
    _cs->recalculateLocalAabb();
}



void HandNode::dump() const
{
    // Create OSG files of any child nodes and the hand subgraph.
    osgDB::writeNodeFile( *( (Transform*)(this) ), "debug-children.osg" );
    osgDB::writeNodeFile( *_hand, "debug-hand.osg" );

    // Display the Transform matrix.
    osg::Matrix m;
    computeLocalToWorldMatrix( m, NULL );
    osg::notify( osg::ALWAYS ) << "PAT local to world: " << m << std::endl;

    // Display all articulation angles in a manner that can easily be copied and
    // pasted into the source code as a new pose.
    osg::notify( osg::ALWAYS ) <<
        "Articulations: " << std::endl <<
        "//  F0   F1   F2   F3   F4" << std::endl <<
        "    " << getArticulation( 0 ) << ", " <<
        getArticulation( 1 ) << ", " <<
        getArticulation( 2 ) << ", " <<
        getArticulation( 3 ) << ", " <<
        getArticulation( 4 ) << ", // translation / flexure" << std::endl <<
        "    " << getArticulation( 5 ) << ", " <<
        getArticulation( 6 ) << ", " <<
        getArticulation( 7 ) << ", " <<
        getArticulation( 8 ) << ", " <<
        getArticulation( 9 ) << ", // inner knuckle" << std::endl <<
        "    " << getArticulation( 10 ) << ", " <<
        getArticulation( 11 ) << ", " <<
        getArticulation( 12 ) << ", " <<
        getArticulation( 13 ) << ", " <<
        getArticulation( 14 ) << " // middle knuckle" << std::endl <<
        "    " << getArticulation( 15 ) << ", " <<
        getArticulation( 16 ) << ", " <<
        getArticulation( 17 ) << ", " <<
        getArticulation( 18 ) << ", " <<
        getArticulation( 19 ) << " // outer knuckle" << std::endl << std::endl;
}


// osgbInteraction
}
