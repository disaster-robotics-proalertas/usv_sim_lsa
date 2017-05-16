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

#include <osgwMx/MxCore.h>
#include <osgwMx/MxUtils.h>

#include <osgwTools/Transform.h>
#include <osgwTools/Orientation.h>

#include <osg/Matrixd>
#include <osg/BoundingSphere>
#include <osg/BoundingBox>
#include <osg/Math>
#include <osg/Notify>

#include <osg/io_utils>
#include <iostream>


namespace osgwMx
{


MxCore::MxCore()
  : _viewUp( 0., 0., 1. ),
    _viewDir( 0., 1., 0. ),
    _position( 0., 0., 0. ),
    _initialUp( 0., 0., 1. ),
    _initialDir( 0., 1., 0. ),
    _initialPosition( 0., 0., 0. ),
    _orientedUp( 0., 0., 1. ),
    _orientedDir( 0., 1., 0. ),
    _orbitCenter( 0., 0., 0. ),
    _rotateScale( 1. ),
    _moveScale( 1., 1., 1. ),
    _ortho( false ),
    _aspect( 1.0 ),
    _fovy( 30.0 ),
    _initialFovy( 30.0 ),
    _fovyScale( 1.1 ),
    _clampFovyScale( true ),
    _clampFovyRange( 5.0, 160.0 ),
    _orthoBottom( 0.0 ),
    _orthoTop( 0.0 )
{
}
MxCore::MxCore( const MxCore& rhs, const osg::CopyOp& copyop )
  : Object( rhs, copyop ),
    _viewUp( rhs._viewUp ),
    _viewDir( rhs._viewDir ),
    _position( rhs._position ),
    _initialUp( rhs._initialUp ),
    _initialDir( rhs._initialDir ),
    _initialPosition( rhs._initialPosition ),
    _orientedUp( rhs._orientedUp ),
    _orientedDir( rhs._orientedDir ),
    _orbitCenter( rhs._orbitCenter ),
    _rotateScale( rhs._rotateScale ),
    _moveScale( rhs._moveScale ),
    _ortho( rhs._ortho ),
    _aspect( rhs._aspect ),
    _fovy( rhs._fovy ),
    _initialFovy( rhs._initialFovy ),
    _fovyScale( rhs._fovyScale ),
    _clampFovyScale( rhs._clampFovyScale ),
    _clampFovyRange( rhs._clampFovyRange ),
    _orthoBottom( rhs._orthoBottom ),
    _orthoTop( rhs._orthoTop )
{
}
MxCore::~MxCore()
{
}



//
// View matrix support
//

osg::Matrixd MxCore::getMatrix() const
{
    const osg::Vec3d c = getCross();
    const osg::Vec3d& u = _viewUp;
    const osg::Vec3d& d = _viewDir;
    const osg::Vec3d p = _position;

    const osg::Matrixd m = osg::Matrixd(
        c[0], c[1], c[2], 0.0,
        u[0], u[1], u[2], 0.0,
        -d[0], -d[1], -d[2], 0.0,
        p[0], p[1], p[2], 1.0 );
    return( m );
}
osg::Matrixd MxCore::getOrientationMatrix() const
{
    const osg::Vec3d c = getCross();
    const osg::Vec3d& u = _viewUp;
    const osg::Vec3d& d = _viewDir;

    const osg::Matrixd m = osg::Matrixd(
        c[0], c[1], c[2], 0.0,
        u[0], u[1], u[2], 0.0,
        -d[0], -d[1], -d[2], 0.0,
        0.0, 0.0, 0.0, 1.0 );
    return( m );
}
osg::Matrixd MxCore::getInverseMatrix() const
{
    const osg::Vec3d c = getCross();
    const osg::Vec3d& u = _viewUp;
    const osg::Vec3d& d = _viewDir;

    const osg::Matrixd m = osg::Matrixd(
        c[0], u[0], -d[0], 0.0,
        c[1], u[1], -d[1], 0.0,
        c[2], u[2], -d[2], 0.0,
        0.0, 0.0, 0.0, 1.0 );
    return( osg::Matrixd::translate( -_position ) * m );

    // Old code, for a sanity check.
    //osg::Matrixd m;
    //m.invert( getMatrix() );
    //return( m );
}

void MxCore::setInitialValues( const osg::Vec3d& up, const osg::Vec3d& dir,
    const osg::Vec3d& pos, double fovy )
{
    _initialUp = up;
    _initialDir = dir;
    _initialPosition = pos;
    _initialFovy = fovy;

    // Error check.
    _initialUp.normalize();
    _initialDir.normalize();
    if( osg::absolute< double >( _initialUp * _initialDir ) > 0.99 )
        osg::notify( osg::WARN ) << "MxCore::setInitialValues: Up and dir vectors are nearly coincident. Results are undefined." << std::endl;

    // orthonormalize
    const osg::Vec3d c = _initialDir ^ _initialUp;
    _initialUp = c ^ _initialDir;
    _initialUp.normalize();
    _initialDir.normalize();
}
void MxCore::getInitialValues( osg::Vec3d& up, osg::Vec3d& dir,
    osg::Vec3d& pos, double& fovy )
{
    up = _initialUp;
    dir = _initialDir;
    pos = _initialPosition;
    fovy = _initialFovy;
}


void MxCore::reset()
{
    _viewUp = _initialUp;
    _viewDir = _initialDir;
    orthonormalize();

    _position = _initialPosition;
    _fovy = _initialFovy;
    setOrtho( false );
}


void MxCore::setOriented( const osg::Vec3d& up, const osg::Vec3d& dir )
{
    _orientedUp = up;
    _orientedDir = dir;

    // Error check.
    _orientedUp.normalize();
    _orientedDir.normalize();
    if( osg::absolute< double >( _orientedUp * _orientedDir ) > 0.99 )
        osg::notify( osg::WARN ) << "MxCore::setOriented: Up and dir vectors are nearly coincident. Results are undefined." << std::endl;

    // orthonormalize
    const osg::Vec3d c = _orientedDir ^ _orientedUp;
    _orientedUp = c ^ _orientedDir;
    _orientedUp.normalize();
    _orientedDir.normalize();
}
void MxCore::setOriented( const osg::Vec3d& dir )
{
    setOriented( _orientedUp, dir );
}
void MxCore::getOriented( osg::Vec3d& up, osg::Vec3d& dir )
{
    up = _orientedUp;
    dir = _orientedDir;
}


void MxCore::setByMatrix( const osg::Matrixd& m )
{
    _viewUp.set( m( 1, 0 ), m( 1, 1 ), m( 1, 2 ) );
    _viewDir.set( -m( 2, 0 ), -m( 2, 1 ), -m( 2, 2 ) );
    _position.set( m( 3, 0 ), m( 3, 1 ), m( 3, 2 ) );
    orthonormalize();
}
void MxCore::setByInverseMatrix( const osg::Matrixd& m )
{
    setByMatrix( osg::Matrixd::inverse( m ) );
}
void MxCore::setOrientationByMatrix( const osg::Matrixd& m )
{
    _viewUp.set( m( 1, 0 ), m( 1, 1 ), m( 1, 2 ) );
    _viewDir.set( -m( 2, 0 ), -m( 2, 1 ), -m( 2, 2 ) );
    orthonormalize();
}
void MxCore::setOrientationByInverseMatrix( const osg::Matrixd& m )
{
    setOrientationByMatrix( osg::Matrixd::inverse( m ) );
}

void MxCore::level()
{
    _viewUp = _initialUp;

    // Check for vurrent view dir coincident with initial up vector. If so,
    // we can't preserve the current view dir and need to set it to the
    // initial view dir.
    if( osg::absolute< double >( _initialUp * _viewDir ) > 0.99 )
        _viewDir = _initialDir;
    else
        _viewDir = _viewUp ^ getCross();
    orthonormalize();
}

void MxCore::lookAtAndFit( const osg::BoundingSphere& bs )
{
    // Look at the bounding sphere center.
    osg::Vec3d newDir = bs.center() - _position;
    newDir.normalize();
    setDir( newDir );

    // Set the eve position distance so that the sphere fits into the minimum FOV.
    double minFov = ( _aspect < 1. ) ? ( _aspect * _fovy ) : _fovy;
    const double distance = osgwMx::computeInitialDistanceFromFOVY( bs, minFov );
    setPosition( bs.center() - ( newDir * distance ) );
}

void MxCore::lookAtAndFit( const osg::BoundingBox& bb )
{
    // We'll get the view matrix to project the bounding box, so pre-configure it
    // to point at the box center. Eye position doesn't matter at this point (we
    // compute the eye position at the end of the function).
    osg::Vec3d newDir = bb.center() - _position;
    newDir.normalize();
    setDir( newDir );


    // Ttransform the bounding box vertices into eye space,
    // then determine their x and y extents. We'll compare the eye
    // space bb aspect ratio against the projection _aspect to
    // determine the critical axis to fit.

    osg::ref_ptr< osg::Vec3Array > corners = new osg::Vec3Array;
    corners->resize( 8 );
    ( *corners )[ 0 ].set( bb._min );
    ( *corners )[ 1 ].set( bb._max.x(), bb._min.y(), bb._min.z() );
    ( *corners )[ 2 ].set( bb._max.x(), bb._min.y(), bb._max.z() );
    ( *corners )[ 3 ].set( bb._min.x(), bb._min.y(), bb._max.z() );
    ( *corners )[ 4 ].set( bb._max );
    ( *corners )[ 5 ].set( bb._min.x(), bb._max.y(), bb._max.z() );
    ( *corners )[ 6 ].set( bb._min.x(), bb._max.y(), bb._min.z() );
    ( *corners )[ 7 ].set( bb._max.x(), bb._max.y(), bb._min.z() );

    osgwTools::transform( getInverseMatrix(), corners.get() );
    // The 'corners' array of bb verts are now in eye space.

    // Determine max and min values for eye space x and y
    osg::Vec2 minEC( FLT_MAX, FLT_MAX ), maxEC( FLT_MIN, FLT_MIN );
    unsigned int idx;
    for( idx=0; idx<8; idx++ )
    {
        const osg::Vec3& v( ( *corners )[ idx ] );
        minEC[ 0 ] = osg::minimum< float >( v.x(), minEC[ 0 ] );
        minEC[ 1 ] = osg::minimum< float >( v.y(), minEC[ 1 ] );
        maxEC[ 0 ] = osg::maximum< float >( v.x(), maxEC[ 0 ] );
        maxEC[ 1 ] = osg::maximum< float >( v.y(), maxEC[ 1 ] );
    }
    // aspect is width (x) over height (y).
    const double ecWidth( maxEC[ 0 ] - minEC[ 0 ] );
    const double ecHeight( maxEC[ 1 ] - minEC[ 1 ] );
    const double ecAspect = ecWidth / ecHeight;


    // We'll store half the extent of interest into a dummy bounding sphere's radius.
    // We'll store the analogous fov in bestFov.
    osg::BoundingSphere bs;
    double bestFov;
    if( ecAspect > _aspect )
    {
        // Fit eye space x to the view
        bs.radius() = ecWidth * .5;
        bestFov = _aspect * _fovy;
    }
    else
    {
        // Fit eye space y to the view
        bs.radius() = ecHeight * .5;
        bestFov = _fovy;
    }

    // The wrap-up code sets the eye position at the best distance from
    // the bb center. Extra distance is added in to account for the fact
    // that the input bound probably has a larger radius than the eye coord
    // bound that we're passing to computeInitialDistanceFromFOVY().
    const double extraDistance = bb.radius() - bs.radius();
    const double distance = extraDistance +
        osgwMx::computeInitialDistanceFromFOVY( bs, bestFov );
    setPosition( bs.center() - ( newDir * distance ) );
}

void MxCore::lookAtOrbitCenter()
{
    osg::Vec3d newDir = _orbitCenter - _position;
    newDir.normalize();

    const osg::Matrixd r = osg::Matrixd::rotate( _viewDir, newDir );
    _viewDir = _viewDir * r;
    _viewUp = _viewUp * r;
    orthonormalize();
}


void MxCore::rotateLocal( double angle, const osg::Vec3d& axis )
{
    const osg::Matrixd r = osg::Matrixd::rotate( angle * _rotateScale, axis );
    _viewDir = _viewDir * r;
    _viewUp = _viewUp * r;
    orthonormalize();
}

void MxCore::rotateOrbit( double angle, const osg::Vec3d& axis )
{
    const osg::Matrixd r = osg::Matrixd::rotate( angle * _rotateScale, axis );

    _position = ( _position - _orbitCenter ) * r + _orbitCenter;
    _viewDir = _viewDir * r;
    _viewUp = _viewUp * r;
    orthonormalize();
}

void MxCore::moveLiteral( const osg::Vec3d& delta )
{
    _position += delta;
    _orbitCenter += delta;
}
void MxCore::moveLocal( const osg::Vec3d& delta )
{
    const osg::Vec3d scaledDelta( delta[0] * _moveScale[0],
        delta[1] * _moveScale[1], delta[2] * _moveScale[2] );
    _position += ( scaledDelta * getOrientationMatrix() );
    _orbitCenter += ( scaledDelta * getOrientationMatrix() );
}
void MxCore::moveConstrained( const osg::Vec3d& delta )
{
    const osg::Vec3d c = getCross();
    const osg::Vec3d& u = _initialUp;
    const osg::Vec3d back = c ^ u;
    const osg::Matrixd orient(
        c[ 0 ], c[ 1 ], c[ 2 ], 0.,
        u[ 0 ], u[ 1 ], u[ 2 ], 0.,
        back[ 0 ], back[ 1 ], back[ 2 ], 0.,
        0., 0., 0., 1. );

    const osg::Vec3d scaledDelta( delta[0] * _moveScale[0],
        delta[1] * _moveScale[1], delta[2] * _moveScale[2] );
    _position += ( scaledDelta * orient );
    _orbitCenter += ( scaledDelta * orient );
}
void MxCore::moveOriented( const osg::Vec3d& delta, const bool orientedToWorld )
{
    const osg::Vec3d c = _orientedDir ^ _orientedUp;
    const osg::Vec3d& u = _orientedUp;
    const osg::Vec3d& d = _orientedDir;
    osg::Matrixd orient(
        c[ 0 ], c[ 1 ], c[ 2 ], 0.,
        u[ 0 ], u[ 1 ], u[ 2 ], 0.,
        -d[ 0 ], -d[ 1 ], -d[ 2 ], 0.,
        0., 0., 0., 1. );
    if( orientedToWorld )
    {
        const osg::Vec3d cl = getCross();
        const osg::Vec3d& ul = _viewUp;
        const osg::Vec3d& dl = _viewDir;

        const osg::Matrixd l2w(
            cl[0], cl[1], cl[2], 0.0,
            dl[0], dl[1], dl[2], 0.0,
            ul[0], ul[1], ul[2], 0.0,
            0.0, 0.0, 0.0, 1.0 );
        orient = orient * l2w;
    }

    const osg::Vec3d scaledDelta( delta[0] * _moveScale[0],
        delta[1] * _moveScale[1], delta[2] * _moveScale[2] );
    _position += ( scaledDelta * orient );
    _orbitCenter += ( scaledDelta * orient );
}
void MxCore::moveWorld( const osg::Vec3d& delta )
{
    const osg::Vec3d& u = _initialUp;
    const osg::Vec3d& d = _initialDir;
    const osg::Vec3d c = d ^ u;
    const osg::Matrixd orient(
        c[ 0 ], c[ 1 ], c[ 2 ], 0.,
        u[ 0 ], u[ 1 ], u[ 2 ], 0.,
        -d[ 0 ], -d[ 1 ], -d[ 2 ], 0.,
        0., 0., 0., 1. );

    const osg::Vec3d scaledDelta( delta[0] * _moveScale[0],
        delta[1] * _moveScale[1], delta[2] * _moveScale[2] );
    _position += ( scaledDelta * orient );
    _orbitCenter += ( scaledDelta * orient );
}
void MxCore::moveOrbit( const float distance )
{
#if 1
   // Original, optimized code
   osg::Vec3d moveAxis = _position - _orbitCenter;
   _position += ( moveAxis * distance * _moveScale[1] );
#else
   // For debugging
   const osg::Vec3d oldPos = _position;
   osg::Vec3d moveAxis = _position - _orbitCenter;
   _position += ( moveAxis * distance * _moveScale[1] );

   osg::notify( osg::NOTICE ) << "moveOrbit:\n" <<
       "\torbit center: " << _orbitCenter << "\n" <<
       "\told pos: " << oldPos << "\n" <<
       "\tnew pos: " << _position << "\n" <<
       "\tdistance: " << (oldPos-_position).length() << std::endl;
#endif
}



void MxCore::getYawPitchRoll( double& yaw, double& pitch, double& roll, bool rightHanded ) const
{
#if 0
    osg::Matrix m( getOrientationMatrix() );
    // Reverse the view vector.
    m(2,0) = -m(2,0);
    m(2,1) = -m(2,1);
    m(2,2) = -m(2,2);

    // Orientation class assumes matrix contains only the delta transform
    // from base vectors to ypr, so subtract out the default orientation.
    const osg::Matrixd baseInv( 1., 0., 0., 0.,
        0., 0., 1., 0.,
        0., 1., 0., 0.,
        0., 0., 0., 1. );

    osg::ref_ptr< osgwTools::Orientation > orient( new osgwTools::Orientation() );
    orient->setRightHanded( rightHanded );
    orient->getYPR( baseInv * m, yaw, pitch, roll );
#else
    // Temp var for cross products.
    osg::Vec3d right;

    const osg::Vec3d viewDirXBaseUp( _viewDir ^ _initialUp );
    const double twoPi( 2. * osg::PI );


    // Yaw

    // Compute view direction, projected into plane defined by base up.
    // TBD what if _viewDir and _initialUp are coincident?
    osg::Vec3d projectedDir = _initialUp ^ viewDirXBaseUp;
    projectedDir.normalize();
    // Is the vector pointing to the left of north, or to the right?
    right = _initialDir ^ _initialUp;
    const double dotDirRight = projectedDir * right;
    // Dot product of two unit vectors is the cosine of the angle between them.
    const double dotDirNorth = osg::clampBetween<double>( projectedDir * _initialDir, -1., 1. );
    double yawRad = acos( dotDirNorth );
    if( dotDirRight > 0. )
        yawRad = osg::PI + ( osg::PI - yawRad );
    if( !rightHanded )
        yawRad = twoPi - yawRad;
    if( yawRad == twoPi )
        yawRad = 0.;
    yaw = osg::RadiansToDegrees( yawRad );


    // Pitch

    const double dotDirUp = _viewDir * _initialUp;
    const double dotUpUp = osg::clampBetween<double>( _viewUp * _initialUp, -1., 1. );
    double pitchRad = acos( osg::absolute< double >( dotUpUp ) );
    if( dotDirUp < 0. )
        pitchRad *= -1.;
    pitch = osg::RadiansToDegrees( pitchRad );


    // Roll

    // Compute base up projected onto plane defined by view direction.
    // TBD what if _viewDir and _initialUp are coincident?
    osg::Vec3d projectedBaseUp = viewDirXBaseUp ^ _viewDir;
    projectedBaseUp.normalize();
    // Is the view up vector pointing to the left of the projected base up, or to the right?
    right = _viewDir ^ projectedBaseUp;
    const double dotUpRight = _viewUp * right;
    // Dot product of two unit vectors is the cosine of the angle between them.
    const double dotUp = osg::clampBetween<double>( projectedBaseUp * _viewUp, -1., 1. );
    double rollRad = acos( dotUp );
    if( dotUpRight > 0. )
        rollRad = osg::PI + ( osg::PI - rollRad );
    if( !rightHanded )
        rollRad = twoPi - rollRad;
    if( rollRad == twoPi )
        rollRad = 0.;
    roll = osg::RadiansToDegrees( rollRad );
#endif
}



//
// Projection / FOV support
//

void MxCore::setOrtho( bool ortho, const double viewDistance )
{
    _ortho = ortho;

    // tan (fovy/2) = a / e2c.len
    _orthoTop = tan( getFovyRadians() * .5 ) * viewDistance;
    _orthoBottom = -_orthoTop;
}

void MxCore::updateFovy( osg::Matrixd& proj ) const
{
    if( _ortho )
    {
        osg::notify( osg::WARN ) << "MxCore::updateFovy: Ortho is not yet implemented. TBD." << std::endl;
    }
    else
    {
        double left, right, bottom, top, near, far;
        proj.getFrustum( left, right, bottom, top, near, far );

        const double fovBottom = atan( bottom / near );
        const double fovTop = atan( top / near );

        const double fovyRatio = getFovyRadians() /
            ( osg::absolute< double >( fovBottom ) + osg::absolute< double >( fovTop ) );

        const double newBottom = tan( fovBottom * fovyRatio ) * near;
        const double newTop = tan( fovTop * fovyRatio ) * near;
        const double xScale = newTop / top;
        left *= xScale;
        right *= xScale;
        proj = osg::Matrixd::frustum( left, right, newBottom, newTop, near, far );
    }
}
osg::Matrixd MxCore::computeProjection( const osg::Vec2d& nearFar ) const
{
    const double zNear = nearFar[ 0 ];
    const double zFar = nearFar[ 1 ];
    if( _ortho )
    {
        const double xRange = _aspect * ( _orthoTop - _orthoBottom );
        const double right = xRange * .5;

        return( osg::Matrixd::ortho( -right, right, _orthoBottom, _orthoTop, zNear, zFar ) );
    }
    else
    {
        double myNear( zNear );
        if( zNear < 0. )
            myNear = zFar / 2000.; // Default OSG z plane ratio.
        return( osg::Matrixd::perspective( _fovy, _aspect, myNear, zFar ) );
    }
}

void MxCore::setFovy( double fovy )
{
    const double ratio = fovy / _fovy;
    _orthoBottom *= ratio;
    _orthoTop *= ratio;
    _fovy = fovy;
}
double MxCore::getFovyRadians() const
{
    return( osg::DegreesToRadians( _fovy ) );
}
void MxCore::fovyScaleUp()
{
    _fovy *= _fovyScale;
    if( _clampFovyScale )
    {
        _fovy = osg::clampBelow< double >( _fovy, _clampFovyRange.y() );
    }

    _orthoBottom *= _fovyScale;
    _orthoTop *= _fovyScale;
}
void MxCore::fovyScaleDown()
{
    const double factor( 1.0 / _fovyScale );
    _fovy *= factor;
    if( _clampFovyScale )
    {
        _fovy = osg::clampAbove< double >( _fovy, _clampFovyRange.x() );
    }

    _orthoBottom *= factor;
    _orthoTop *= factor;
}
void MxCore::setClampFovyScale( bool clamp, osg::Vec2d clampFovyRange )
{
    _clampFovyScale = clamp;
    _clampFovyRange = clampFovyRange;
    if( _clampFovyScale )
    {
        _fovy = osg::clampBetween< double >( _fovy, _clampFovyRange.x(), _clampFovyRange.y() );
    }
}



void MxCore::orthonormalize()
{
    const osg::Vec3d c = getCross();
    _viewUp = c ^ _viewDir;
    _viewDir.normalize();
    _viewUp.normalize();
}


// osgwMx
}
