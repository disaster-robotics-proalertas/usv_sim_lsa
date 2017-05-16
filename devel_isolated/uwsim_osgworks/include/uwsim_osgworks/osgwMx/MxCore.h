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

#ifndef __OSGWMX_MX_CORE_H__
#define __OSGWMX_MX_CORE_H__ 1


#include <osgwMx/Export.h>
#include <osg/Object>
#include <osg/Matrixd>
#include <osg/BoundingSphere>
#include <osg/BoundingBox>
#include <cmath>



namespace osgwMx
{


/** \class MxCore MxCore.h <osgwMx/MxCore.h>
\brief A GUI-independent class for managing model, view, and projection matrices.
\details TBD.

MxCore support multiple types of move modes to translate the position:
\li moveLiteral() Add the delta directly to the current position.
\li moveLocal() Move in eye-local coordinates.
\li moveConstrained() Move in eye-local, constrained to a ground plane.
\li moveOriented() Move in the application-specified arbitrary coordinates (see setOriented() ).
\li moveOrbit() Move in altitude and azimute around an orbit center point.
\li moveWorkd() Move in the initial coordinate space (see setInitialValues() ).

MxCore support multiple tupes of rotate modes to modify the orientation:
\li rotateLocal() Change the view and up vectors, maintaining a constant position.
\li rotateOrbit() Orbit around an orbit center point.
*/
class OSGWMX_EXPORT MxCore : public osg::Object
{
public:
    MxCore();
    MxCore( const MxCore& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );

    META_Object(osgwMx,MxCore);


    /** \name Model or View Matrix Access
    \details When using MxCore to transform geometry, access the model
    matrix using the getMatrix() method. The return value is suitable
    for use with osg::MatrixTransform::setMatrix(). You can also access
    just the orientation using getOrientationMatrix().

    When using MxCore to control a view, access the view matrix using
    the getInverseMatrix() method. The return value is suitable for use
    with osg::Camera::setViewMatrix(); */
    /**\{*/

    /** \brief Get a transform matrix.
    \details Returns a matrix suitable for orienting/positioning an object in the
    world. The object's +z axis will be aligned with the view direction, and
    the object's +y axis will be aligned with the view up vector. */
    osg::Matrixd getMatrix() const;
    /** Same as getMatrix() except without the translation. */
    osg::Matrixd getOrientationMatrix() const;
    /** \brief Get a view matrix.
    \details Returns a matrix suitable for use as a view matrix. The view's
    position, direction, and up vector are taken from this class. */
    osg::Matrixd getInverseMatrix() const;

    /**\}*/


    /** \name Basis and Origin Access
    \details This section contains method for modifying orientation
    and position. */
    /**\{*/

    /** \brief Set the initial up, dir, and position vectors, and fovy.
    \details These values are used in the reset() and
    getYawPitchRoll() functions.

    \param fovy Field of view in y, in degrees.

    \c up defaults to (0., 0., 1.), \c dir defaults to (0., 1., 0.),
    \c pos defaults to (0., 0., 0.), and \c fovy defaults to 30.0. */
    void setInitialValues( const osg::Vec3d& up, const osg::Vec3d& dir,
        const osg::Vec3d& pos, double fovy=30. );
    /** \brief Get the initial up, dir, position, and fovy. */
    void getInitialValues( osg::Vec3d& up, osg::Vec3d& dir,
        osg::Vec3d& pos, double& fovy );

    /** \brief Set the current view to the initial view.
    \details Sets the current view up, view dir, view position, and fovy values to their
    initial values (see setInitialValues()), and disables orthographic projection. */
    void reset();

    /** \brief Set the oriented up and dir vectors.
    \details These values are used with the moveOriented() method.
    In typical usage, an application tracks orientation
    of a handheld device, sets that information with this method, then
    calls moveOriented() to move in this coordinate system

    \c up defaults to (0., 0., 1.) and \c dir defaults to (0., 1., 0.). */
    void setOriented( const osg::Vec3d& up, const osg::Vec3d& dir );
    /** \overload */
    void setOriented( const osg::Vec3d& dir );
    /** \brief Get the oriented up and dir vectors. */
    void getOriented( osg::Vec3d& up, osg::Vec3d& dir );


    /** \brief Access the current view up vector. */
    void setUp( const osg::Vec3d& up ) { _viewUp = up; }
    /** \brief Access the current view up vector. */
    osg::Vec3d getUp() { return( _viewUp ); }
    /** \overload */
    const osg::Vec3d& getUp() const { return( _viewUp ); }

    /** \brief Access the current view direction vector. */
    void setDir( const osg::Vec3d& dir ) { _viewDir = dir; }
    /** \brief Access the current view direction vector. */
    osg::Vec3d getDir() { return( _viewDir ); }
    /** \overload */
    const osg::Vec3d& getDir() const { return( _viewDir ); }

    /** \brief Access the current view position). */
    void setPosition( const osg::Vec3d& newPos ) { _position = newPos; }
    /** \brief Access the current view position). */
    osg::Vec3d getPosition() { return( _position ); }
    /** \overload */
    const osg::Vec3d& getPosition() const { return( _position ); }

    /** \brief Sets the current up, dir, and position from \c m. */
    void setByMatrix( const osg::Matrixd& m );
    /** \brief Same as setByMatrix(), but assume \c m is an inverse or view matrix. */
    void setByInverseMatrix( const osg::Matrixd& m );
    /** \brief Sets the current up and dir (position is ignored) from \c m. */
    void setOrientationByMatrix( const osg::Matrixd& m );
    /** \brief Same as setOrientationByMatrix(), but assume \c m is an inverse or view matrix. */
    void setOrientationByInverseMatrix( const osg::Matrixd& m );

    /** \brief Convenience routine to return the cross product of
    \c _viewDir and \c _viewUp. */
    osg::Vec3d getCross() const { return( _viewDir ^ _viewUp ); }

    /** \brief Set the current up vector to the initial up vector.
    \details Sets the current up vector to the initial up vector, with
    a minimum change to the view direction vector. If initial up
    vector and current view direction are nearly coincident, the
    view dir is set to the initial view dir. */
    void level();

    /** \brief Center and fit the bounding volume.
    \details Change the view direction to look at the \c bs center.
    Then move the view position closer to or further from the
    \c bs center in order to fit it to the current view (based
    on \c _fovy and \c _aspect). */
    void lookAtAndFit( const osg::BoundingSphere& bs );
    /** Not yet implemented. */
    void lookAtAndFit( const osg::BoundingBox& bb );

    /** \brief Look at the orbit center point.
    \details Change the view direction vector to look directly at the orbit
    center point. */
    void lookAtOrbitCenter();


    /** \brief Rotate in local mode.
    \details Changes the view direction (and possibly the up vector), but
    keeps the view position constant. To keep the up vector constant,
    pass the current up vector as the \c axis parameter. This function
    supports first person viewing. \c angle is in radians.
    See setRotateScale().
    
    This method replaces the deprecated rotate() method. */
    void rotateLocal( double angle, const osg::Vec3d& axis );
    /** \brief Rotate in orbit mode.
    \details Rotates the view position about a point in world coordinates.
    This function supports an orbit-type view. \c angle is in radians.
    See setRotateScale().

    This method replaces the deprecated rotate(double,osg::Vec3d&,osg::Vec3d&) method. */
    void rotateOrbit( double angle, const osg::Vec3d& axis );

    /** \brief Set the orbit center point used by rotateOrbit().
    \details The default is (0,0,0), the origin.
    Note: The orbit center point is in world coordinates.*/
    void setOrbitCenterPoint( const osg::Vec3d& orbitCenter ) { _orbitCenter = orbitCenter; }
    /** \brief Get the orgit center point. */
    osg::Vec3d getOrbitCenterPoint() const { return( _orbitCenter ); }

    /** \brief Set the rotation angle scale.
    \details The rotate...() family of methods multiply their \c angle
    parameter by the specified \c rotateScale value before performing the rotation.
    The default rotate scale value is 1.0 (no scaling). */
    void setRotateScale( double rotateScale ) { _rotateScale = rotateScale; }
    /** \brief Get the rotation angle scale. */
    double getRotateScale() const { return( _rotateScale ); }


    /** \brief Move by the \c delta amount.
    \details Add \c delta directly
    to the current view position \c _position. This interface is used by the kbd /
    mouse MxEventHandler to effect panning tied to the mouse position.

    This method replaces the deprecated moveWorldCoords() method. */
    void moveLiteral( const osg::Vec3d& delta );

    /** \brief Move the view position in view-relative coordinates.
    \details This is an intuitive and generally useful for movement in an arbitrary
    view-centric coordinate system. See also moveConstrainted() for a variant to allow
    movement in a typical up-oriented 3D environment.

    This method replaces the deprecated move() method.

    \c delta[0] is movement to the right (+) or left (-), \c delta[1] is movement
    up (+) or down (-), and \c delta[2] is movement backward (+) or forward (-).
    
    Note that movement is scaled (see setMoveScale()). */
    void moveLocal( const osg::Vec3d& delta );

    /** \brief Move in local coordinates constrained by the world (initial) up vector.
    \details This is probably the most useful of the move function variants, as
    it allows the user to move in a world defined by the \c _initialUp vector and
    its implied ground plane. As an example, this function allows flat horizontal
    movement even when the view is looking up or down relative to the horizon.

    Uses the initial up vector \c _initialUp to define a ground plan", and then
    moves in that environment as follows: \c delta[0] moves in the ground plane
    right (+) or left (-); \c delta[1] moves along \c _initialUp in the up (+) and
    down (-) directions; \c delta[2] moves in the ground plane backward (+) or
    forward (-).

    Note that movement is scaled (see setMoveScale()). */
    void moveConstrained( const osg::Vec3d& delta );

    /** \brief Move the view position in oriented coordinates.
    \details Use this function for movement in the coordinate system specified
    with the setOriented() method. This allows movement in an arbitrary coordinate
    system independent of world and local coordinates. In a typical use case, it
    is used for movement along the direction vector of a tracked device.

    If \c orientedToWorld is true, movement is in the concatenation of the
    local and oriented coordinate systems.

    If \c orientedToWorld is false, movement is in the oriented coordinate system
    only, as follows:
    \c delta[1] is movement along the oriented up vector. Positive values move
    along that vector, and negative in the opposite direction. \c delta[2] moves
    similarly, but along the negated dir vector. \c delta[0] moves along the cross
    product of the oriented dir and up vectors.

    Note that movement is scaled (see setMoveScale()). */
    void moveOriented( const osg::Vec3d& delta, const bool orientedToWorld=true );

    /** \brief Move the view position by a delta amount in world coordinate space.
    \details "World coordinates" means the coordinate space defined with setInitialValues():
    \c _initialDir is the negative z axis, \c _initialUp is the y axis, and the x axis
    is \c _initialDir ^ \c _initialUp (cross product).
    
    Note that movement is scaled (see setMoveScale()). */
    void moveWorld( const osg::Vec3d& delta );

    /** \brief Move closer to (-) or further away from (+) the orbit center point.
    \details Movement automaticall slows as a function of distance to \c _orbitCenter,
    alower when close and faster when further away.

    Note that movement is also scaled by  setMoveScale(). */
    void moveOrbit( const float distance );


    /** \brief Set the movement scale.
    \details The move*() family of functions perform an
    element-wise multiplication between their \c delta parameter and the specified
    \c moveScale vector before performing the translatkion. The default movement
    scale vector is (1., 1., 1.) (no scaling). */
    void setMoveScale( const osg::Vec3d& moveScale ) { _moveScale = moveScale; }
    /** \brief Get the movement scale. */
    osg::Vec3d getMoveScale() { return( _moveScale ); }
    /** \overload */
    const osg::Vec3d& getMoveScale() const { return( _moveScale ); }

    /** \brief Get current yaw/pitch/roll angles for the current view.
    \details Values are computed relative to the initial up and dir vectors
    (see setInitialValues()). All return values are in degrees.
    \param yaw Heading value. 0.0 <= yaw < 360.0.
    \param pitch Elevation value. -(pi/2.0) <= pitch <= (pi/2.0).
    \param roll Twist value. 0.0 <= roll < 360.0.
    \param rightHanded Use a right-handed coordinate system to compute
    yaw and roll angles. Turning left increases the yaw angle when rightHanded=true,
    and decreases the yaw angle when rightHanded=false. Likewise, counter-clockwise
    roll increases the roll angle when rightHanded=true, but decreases the roll angle
    when rightHanded=false. */
    void getYawPitchRoll( double& yaw, double& pitch, double& roll, bool rightHanded=false ) const;

    /**\}*/


    /** \name Projection Matrix Controls
    \details Use computeProjection() to obtain a projection matrix based on
    state specified with the following interface. Possible projection matrix
    state values include the aspect ratio, field of view in Y, and a Boolean
    toggle for orthographic mode. */
    /**\{*/

    /** \brief Specify whether or not to use an orthographic projection.
    \details Specify \c true to enable orthographic mode, and false to disable
    orthographic mode (and use perspective instead). The default is
    false (perspective). */
    void setOrtho( bool ortho, const double viewDistance=1. );
    /** \brief Returns true if using an orthographic projection, false otherwise. */
    bool getOrtho() const { return( _ortho ); }

    /** \brief Set the aspect ratio.
    \details This value is used in the computation of the projection
    matrix. */
    void setAspect( double aspect ) { _aspect = aspect; }
    /** \brief Get the aspect ratio. */
    double getAspect() const { return( _aspect ); }

    /** \brief Set the current fovy in \c proj.
    \details Modify \c proj so that it uses \c _fovy for its field of view in y,
    maintaining the same aspect ratio and near and far plane values. This
    function works for both symmetrical and assymetrical view volumes. */
    void updateFovy( osg::Matrixd& proj ) const;
    /** \brief Compute a projection matrix.
    \details Conpute a symmetrical projection matrix using the specified zNear and
    zFar planes. */
    osg::Matrixd computeProjection( const osg::Vec2d& nearFar ) const;

    /** \brief Set the field of view in y (fovy) in degrees.
    \details Default is 30.0 degrees. */
    void setFovy( double fovy );
    /** \brief Get the field of view in y, in degrees. */
    double getFovy() const { return( _fovy ); }
    /** \brief Get the field of view in y, in radians. */
    double getFovyRadians() const;

    /** \brief Scale fovy up, using the scale value set with setFovyScale(). */
    void fovyScaleUp();
    /** \brief Scale fovy down, using the scale value set with setFovyScale(). */
    void fovyScaleDown();

    /** \brief Set the fovy scale value.
    \details Percentage to increase the fovy in a fovyScaleUp() call.
    For example, to increase fovy by 120% in that call, pass 1.2. Default is
    1.1 (110%). The inverse (1.0 / \c fovyScale) is used in the fovyScaleDown()
    call. */
    void setFovyScale( double fovyScale ) { _fovyScale = fovyScale; }
    /** \brief Get the fovy scale value. */
    double getFovyScale() const { return( _fovyScale ); }

    /** \brief Set the clamp range for scaled field of view.
    \details Default is to clamp fovy scaling to the range (1.0,160.0). Pass \c false
    as first paramter to disable clamping. */
    void setClampFovyScale( bool clamp, osg::Vec2d clampFovyRange=osg::Vec2d(1.0,140.0) );

    /**\}*/

protected:
    ~MxCore();
    void orthonormalize();

    osg::Vec3d _viewUp, _viewDir, _position;
    osg::Vec3d _initialUp, _initialDir, _initialPosition;
    osg::Vec3d _orientedUp, _orientedDir;

    osg::Vec3d _orbitCenter;
    double _rotateScale;
    osg::Vec3d _moveScale;

    bool _ortho;
    double _aspect;

    double _fovy;
    double _initialFovy;
    double _fovyScale;
    bool _clampFovyScale;
    osg::Vec2d _clampFovyRange;
    double _orthoBottom, _orthoTop;
};


// osgwMx
}


// __OSGWMX_MX_CORE_H__
#endif
