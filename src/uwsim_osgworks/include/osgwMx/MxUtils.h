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

#ifndef __OSGWMX_MX_UTILS_H__
#define __OSGWMX_MX_UTILS_H__ 1


#include <osgwMx/Export.h>
#include <osgwMx/MxCore.h>
#include <osg/BoundingSphere>
#include <osg/Vec2d>

#include <vector>
#include <map>


namespace osgwMx
{


/** \defgroup mxutils Utilities for MxCore-based device interface classes.
*/
/*@{*/


/** \brief Computes an appropriate viewing field of view.
\details Compute a field of view appropriate for rendering the entire bounding
sphere \c bs when viewed from the specified \c distance.
\return Field of view in radians.
*/
double OSGWMX_EXPORT computeInitialFOVYFromDistance( const osg::BoundingSphere& bs, const double distance );

/** \brief Computes an appropriate viewing distance.
\details Computes a viewing distance appropriate for viewing the entire bounding
sphere \c bs with a perspective projection. \c fov should be the minimum
field of view, either horizontally or vertically (accounting for aspect ratio).
\param fovy Field of view in degrees.
*/
double OSGWMX_EXPORT computeInitialDistanceFromFOVY( const osg::BoundingSphere& bs, const double fov );


/** \brief Computes optimal near and far plane distances.
\details Computes near and far clip plane distance values that completely encompass the
given bounding sphere \c bs. The returned near and far values are suitable for use
in constructing an OpenGL FFP projection matrix.

For perspective projections, the function computes near and far values as
positive world coordinate unit distance values in front of \c position, the
view position. If \c position is inside the bounding sphere \c bs, the
computed near value is ( far / 2000. );

For orthographic projections, the function computes near and far values as
world coordinate unit distance values relative to \c position.

\param position The current eye position. If \c ortho is false, the returned near
and far distances are positive values in front of \c position along the current view
vector.
\param bs Returned near and far planes fit around this bounding sphere (except as
noted above for the perspective case with \c position inside the sphere).
\param ortho Pass true to compute near and far values for use with an orthographic
projection matrix. The default is false (perspective projection).
\return The returned vector contains the near value in element 0 and the
far value in element 1.
*/
osg::Vec2d OSGWMX_EXPORT computeOptimalNearFar( const osg::Vec3d& position,
    const osg::BoundingSphere& bs, bool ortho=false );


/** \brief Computes a plane orthogonal to the view for use with panning.
\details Mouse-based view panning requires the 3D point under the mouse pointer
to remain constant. Use computePanPlane() and pan() together to facilitate this
behavior.

The typical application-code use case is:
\li User depresses mouse button.
\li Application code responds by calling computePanPlane().
\li User drags mouse with button depressed.
\li For each drag event, application calls pan() and passes in both the pan plane
(obtained from computePanPlane()) and the delta NDC mouse motion. pan() returns
the literal delta amount corresponding to that motion. The application passes this
(negated) amount to MxCore::moveLiteral() to effect tha panning.

Call computePanPlane() at the start of the pan interaction to compute
a plane equation, then pass that plane equation to the pan() function while the
user pans (typically with mouse drag events).

computePanPlane() takes NDC x and y values as input, typically the mouse
location normalized to the range -1 to 1 in x and y. It back-projects this point
into world coordinates, then intersects with the \scene to obtain a world coordinate
pick point. (If the intersection fails, computePanPlane() uses a point 10 units in
front of the eye as the pick point.) The returned plane equation contains the pick
point, and has a normal obtained from mxCore->getDir() (the view direction
vector).

In order to back-prokect the NDC mouse coordnate \c ndcX and \c ndcY, the function
needs both the current view matrix and current projection matrix. The \c mxCore
projection member variables \c _aspect, \c _fovy, and \c _ortho must be configured
to return the projection matrix (see MxCore::computeProjection().

See pan() for information on how the \c panPlane pan plane equation is used during
panning.

\param scene Used for its bounding sphere (to obtain reasonable distance values during
back-projection). Also used for intersection testing to determine a pick point.
\param mxCore Used to obtain the cirrent view direction and view position. \c mxCore
must also be configured to return an appropriate projection matrix. 
\param ndcX Initial pan x location normalized to range -1 to 1.
\param ndcY Initial pan y location normalized to range -1 to 1.
*/
osg::Vec4d OSGWMX_EXPORT computePanPlane( osg::Node* scene, const osgwMx::MxCore* mxCore,
    const double ndcX, const double ndcY );

/** \brief Computes world coordinate movement in the given pan plane.
\details This function computes literal xyz delta motion in the given \c panPlane. It does
this by back-projecting the given delta NDC coordinates onto \c panPlane, along with the
back-projection of the NDC origin onto that same plane. The returned value is the
difference between these two projected values.

pan() is designed to work with computePanPlane(). Application / device handling
code should call computePanPlane() at the start of a pan interaction in order to
obtain the \c panPlane plane equation. See computePanPlane() for more information.

\param scene Used only for its bounding sphere in order to obtain reasonable distance
values during back-projection.
\param mxCore As with computePanPlane(), \c mxCore must be configured to return an
appropriate projection matrix. pan() further uses \c mxCore to obtain the cirrent view
direction and view position.
\param panPlane plane equation, obtained from computePanPlane().
\param deltaNdcX Delta x movement normalized to range -1 to 1.
\param deltaNdcY Delta y movement normalized to range -1 to 1.
\return Literal xyz motion vector. In the typical use case (to pan the entire scene),
pass the negated return value to MxCore::moveLiteral().
*/
osg::Vec3d OSGWMX_EXPORT pan( const osg::Node* scene, const osgwMx::MxCore* mxCore,
    const osg::Vec4d panPlane, const double deltaNdcX, const double deltaNdcY );


/** \brief Utility routine for performing an intersection test (pick).
\details Creates a ray from the \c mxCore view poitiion to the world coordinate
\c farPoint, then uses osgUtil to test for an intersection with \c scene. The
closest pick point is returned in \c result.
\return true if an intersection occurred and \c result is valid. False if no
intersections occurred.

TBD Future work: combine pickPoint and intersect into one function. */
bool OSGWMX_EXPORT intersect( osg::Vec3d& result, const osg::Vec3d& farPoint,
    osg::Node* scene, const osgwMx::MxCore* mxCore );


/** \brief Picks a world coordinate point from NDX xy input values.
\details This is a variant of the intersect() function that takes NDX xy values as input
and returns the picked world coordinate point. If no intersections occurred, this function
displays a warning and returns (0,0,0).

TBD Future work: combine pickPoint and intersect into one function. */
osg::Vec3d OSGWMX_EXPORT pickPoint( osg::Node* scene, const osgwMx::MxCore* mxCore,
    const double ndcX, const double ndcY );

/** Intersects a ray with a plane. OSG doesn't appear to have this utility.
\return True if an intersection occurred and \c result is valie. False otherwise. */
bool OSGWMX_EXPORT intersectRayPlane( osg::Vec3d& result, const osg::Vec4d& plane, const osg::Vec3d& p0, const osg::Vec3d& p1 );


/** \brief Computes an angle and axis of rotation for a simulated trackball.

The returned \c axis and \c angle values are intended to be passed to the MxCore::rotateLocal() function.

\param start The xy start position in normalized (-1 to 1 ) windows coordinates.
\param delta The delta xy morion in normalized (-1 to 1 ) windows coordinates.
\param orientMat The current view orientation matrix.
\param sensitivity Controls simulated trackball sensitivity. Smaller values produce less
of a rool effect, while larger values produce a more pronounced roll effect. Valid range
is 0.0 to pi/2. Default is 1.3.
\return angle The computed rotation angle in radians.
\return axis The axis to rotate about.
*/
void OSGWMX_EXPORT computeTrackball( double& angle, osg::Vec3d& axis,
    const osg::Vec2d& start, const osg::Vec2d& delta,
    const osg::Matrixd& orientMat, const double sensitivity=1.3 );


/** \class FunctionalMap MxUtils.h <osgwMx/MxUtils.h>
\brief Maps an unsigned int to a particular function.

This class is presently used for arbitrary mapping of MxGamePad buttons,
where each bit is a single bit in a 32-bit unsigned int. In this usage,
the class supports the dot OSG file format, which means a FunctionalMap
can be read from or written to a .osg file as an osg::Object, thus providing
"config file" support for game pads.

The class can be used to map arbitrary unsigned int values (not bit values),
but currently this usage is not supported with the dot OSG file format.

The MxGamePad base class owns an instance of FunctionalMap, which can be
shared amongst game pad instances. The copy constructor is fully supported.

Applications can set the function mapping in two ways:
\li FunctionalMap::configure(), passing in a specific unsigned int \c key value
and a FunctionalMap::FunctionType.
\li Read the FunctionalMap osg::Object from a .osg file (and set it in the game
pas with a call to MxGamePad::setFunctionalMap()).

Future work:
\li Support non-bitfield keys in dot OSG.
\li Possible changes to support devices other than game pad (such as space
ball, kbd/mouse, etc).
*/
class OSGWMX_EXPORT FunctionalMap : public osg::Object
{
public:
    FunctionalMap();
    FunctionalMap( const FunctionalMap& rhs, osg::CopyOp copyop=osg::CopyOp::SHALLOW_COPY );
    // Note that lib name="osgwTools" is currently required because we only have one dot OSG plugin.
    META_Object(osgwTools,FunctionalMap);

    /** Various game pad functions. */
    typedef enum {
        /** Set the view direction so it's level with the ground plane, defined
        by the initial up vector. */
        LevelView=0,

        /** Warp the position to the world coordinate origin. */
        JumpToWorldOrigin,
        /** Warp the position to the initial / home position, resetting the view
        direction and up vector. */
        JumpToHomePosition,

        /** Scale movement down (slow down). */
        MoveModifyScaleSpeedDown,
        /** Scale movement up (speed up). */
        MoveModifyScaleSpeedUp,

        /** Move in literal coordinates. */
        MoveModeLiteral,
        /** Move in eye local coordinates. */
        MoveModeLocal,
        /** Move in the ground plane (defined by the initial up vector). */
        MoveModeConstrained,
        /** Move in the oriented coordinate system. */
        MoveModeOriented,
        /** Move in world coordinates. */
        MoveModeWorld,
        /** Move along a vector between the eye and the orbit center, slowing
        when near the center and speeding up when further away. */
        MoveModeOrbit,
        /** Cycle through the available move modes. This function is not yet
        implemented. */
        CycleMoveMode,

        /** \brief Movement modifier for up/down motion.
        \details When this function is active, left stick y-axis maps to up/down
        movement. The operation of the up/down movement varies by the current move mode. */
        MoveModifyUpDown,

        /** A button interface for moving up at the gamepad left stick rate. */
        MoveUpAtRate,
        /** A button interface for moving down at the gamepad left stick rate. */
        MoveDownAtRate,

        /** Rotate from the viewpoint (turn the viewer's head). */
        RotateModeLocal,
        /** Orbit around the orbit center point. */
        RotateModeOrbit,
        /** Non-Euclidean arcball. This function is not yet implemented. */
        RotateModeArcball,
        /** Cycle through the available rotation modes. This function is not yet
        implemented. */
        CycleRotateMode,

        /** \brief Rotation modifier for barrel roll.
        \details When this function is active, left stick x-axis maps to rotation around the
        view direction, and left stick y-axis becomes a no-op. */
        RotateModifyRoll,

        /** For unused entries in the function map. This enum should always
        be last in the list of enums; it is used in the std::vector::resize()
        call at init time. */
        NoOp
    } FunctionType;
    static std::string asString( FunctionType func );
    static FunctionType asFunctionType( const std::string& str );

    static bool validMoveMode( const FunctionType func );
    static bool validRotateMode( const FunctionType func );

    static FunctionType cycleMoveMode( const FunctionType func );
    static FunctionType cycleRotateMode( const FunctionType func );

    /** Set the function for a specific unsigned int \c key. */
    void configure( const unsigned int key, FunctionType func );
    FunctionType getConfiguration( const unsigned int key ) const;

    /** Specify that the unsigned int \c key is enabled or disabled. This is
    generally called in response to a game pad button press. */
    void set( const unsigned int key, bool enable=true );
    /** Interpret the mask as a bitfield where each bit position corresponds to
    an unsigned int bit value key, and whether that position is set or not determines
    whether the corresponding function is enabled or disabled. */
    void setFromBitmask( const unsigned int mask );

    bool isSet( const unsigned int key ) const;
    bool isSet( const FunctionType func ) const;

    /** Clear the mapping and the current state. */
    void reset();

protected:
    ~FunctionalMap();

    typedef std::map< unsigned int, FunctionType > FunctionMapType;
    FunctionMapType _map;

    std::vector< bool > _state;
};


/*@}*/


// osgwMx
}


// __OSGWMX_MX_UTILS_H__
#endif
