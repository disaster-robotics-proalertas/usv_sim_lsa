/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2010 by Kenneth Mark Bryden
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

#ifndef __OSGWTOOLS_SHAPES_H__
#define __OSGWTOOLS_SHAPES_H__ 1

#include "osgwTools/Export.h"
#include <osg/Geometry>



namespace osgwTools {


/** \defgroup Shapes Convenience routines for Geometry-based shape creation

This is a set of convenience routines for configuring a Geometry to render
specific shapes. It is intended as a replacement for the OSG ShapeDrawables.
While it lacks the implicit metadata found in ShapeDrawables, it allows direct
access to vertices and vertex data, which is more widely supported than ShapeDrawables.

All convenience routines follow a common convention of allowing the app to
supply a Geometry to use. By default, the convenience routines instantiate a
Geometry. All routines set a color array with a single white color.

All shapes center on the origin. Use OSG Transform Nodes to translate
or orient the shapes.

\test shapes

*/
/*@{*/


/** \defgroup SphereShapes Sphere generation routines */
/*@{*/

/** \brief Creates a geodesic sphere.

Creates a geodesic sphere starting from the 20 triangles of an icosehedron.
Each subdivision splits a triangle into four new triangles. The generated
Geometry uses a single DrawElementsUShort GL_TRIANGLES PrimitiveSet and contains
vertex, normal, texture coordinate, and color data. The texture coordinates are
configured for unit 0 and suitable for cube mapping. */
OSGWTOOLS_EXPORT osg::Geometry* makeGeodesicSphere( const float radius=1., const unsigned int subdivisions=2, osg::Geometry* geometry=NULL );

/** \brief A transformed geodesic sphere.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeGeodesicSphere( const osg::Matrix& m, const float radius=1., const unsigned int subdivisions=2, osg::Geometry* geometry=NULL );


/** \brief Creates an alt-az (or lat-long) sphere.

Creates an alt-az sphere (also commonly called a lat-long sphere) using
GL_TRIANGLE_STRIP PrimitiveSets. subLat indicates the number of GL_TRIANGLE_STRIPS
wrapping the sphere in the xy plane. Each strip contains (subLong*2) triangles, 
including normals per vertex and texture coordinates in unit 0.
Texture coordinates are suitable for applying a geographic projection texture. 
(t coordinate ranges from 0 at the (-z) sphere bottom to 1 at the (+z) sphere top, 
and s wraps around the sphere with 0 at +x, to +y, to -x, to -y, and ending with 1.0 at +x.) */
OSGWTOOLS_EXPORT osg::Geometry* makeAltAzSphere( const float radius=1., const unsigned int subLat=8, const unsigned int subLong=16, osg::Geometry* geometry=NULL );

/** \brief A transformed alt-az sphere.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeAltAzSphere( const osg::Matrix& m, const float radius=1., const unsigned int subLat=8, const unsigned int subLong=16, osg::Geometry* geometry=NULL );


/** \brief Creates a wireframe alt-az (or lat-long) sphere.

Creates an alt-az sphere using GL_LINE_LOOP and GL_LINE_STRIP PrimitiveSets.
This function doesn't create normal or texture coordinate data, and disables 
GL_LIGHTING in the Geometry's StateSet. */
OSGWTOOLS_EXPORT osg::Geometry* makeWireAltAzSphere( const float radius=1., const unsigned int subLat=8, const unsigned int subLong=16, osg::Geometry* geometry=NULL );

/** \brief A transformed wireframe alt-az sphere.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireAltAzSphere( const osg::Matrix& m, const float radius=1., const unsigned int subLat=8, const unsigned int subLong=16, osg::Geometry* geometry=NULL );

/*@}*/


/** \defgroup PlaneShapes Plane generation routines */
/*@{*/

/** \brief Creates a tesselated plane.

Makes a plane using GL_TRIANGLE_STRIP PrimitiveSets. Use the u and v 
vector parameters to specify the plane's dimensions. 
By default, this function creates a plane 
using two triangles. It uses the subdivision parameter to increase the 
plane tessellation, which specifices how to subdivide the plane in x and y. 
The function creates normals per vertex and texture coordinates in unit 0 
with the normal computed as (u^v). The texture coordinate data
is suitable for applying an entire texture map to the plane. */
OSGWTOOLS_EXPORT osg::Geometry* makePlane( const osg::Vec3& corner, const osg::Vec3& u, const osg::Vec3& v, const osg::Vec2s& subdivisions=osg::Vec2s(1,1), osg::Geometry* geometry=NULL );

/** \brief Creates a wireframe tesselated plane.

Full documentation is TBD.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWirePlane( const osg::Vec3& corner, const osg::Vec3& u, const osg::Vec3& v, const osg::Vec2s& subdivisions=osg::Vec2s(1,1), osg::Geometry* geometry=NULL );

/*@}*/


/** \defgroup BoxShapes Box generation routines */
/*@{*/

/** \brief Creates a box.

This function makes an axis-aligned box using GL_TRIANGLE_STRIP PrimitiveSets. The dimensions
are (halfExtents*2). By default, two triangles are on each side of the box, for
a total of 12 triangles. However, the subdivision parameter can increase this, 
which specifices how to subdivide the faces of the box in x, y, and z and creates 
normal and texture coordinate (unit 0) data. The texture coordinate data
is suitable for applying an entire texture map to each face of the box. */
OSGWTOOLS_EXPORT osg::Geometry* makeBox( const osg::Vec3& halfExtents, const osg::Vec3s& subdivisions=osg::Vec3s(1,1,1), osg::Geometry* geometry=NULL );

/** \brief A transformed box.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeBox( const osg::Matrix& m, const osg::Vec3& halfExtents, const osg::Vec3s& subdivisions=osg::Vec3s(1,1,1), osg::Geometry* geometry=NULL );


/** \brief Creates a plain box with no normal, color, or texture coordinate information.

Uses a minimum number of vertices (8). Suitable for low-cost display of bounding boxes. */
OSGWTOOLS_EXPORT osg::Geometry* makePlainBox( const osg::Vec3& halfExtents, osg::Geometry* geometry=NULL );

/** \brief A transformed plain box.
*/
OSGWTOOLS_EXPORT osg::Geometry* makePlainBox( const osg::Matrix& m, const osg::Vec3& halfExtents, osg::Geometry* geometry=NULL );


/** \brief Creates a wireframe box.
This function doesn't create normal or texture coordinate data, and disables 
GL_LIGHTING in the Geometry's StateSet. */
OSGWTOOLS_EXPORT osg::Geometry* makeWireBox( const osg::Vec3& halfExtents, osg::Geometry* geometry=NULL );

/** \brief A transformed wireframe box.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireBox( const osg::Matrix& m, const osg::Vec3& halfExtents, osg::Geometry* geometry=NULL );

/*@}*/


/** \defgroup CircleShapes Circle generation routines */
/*@{*/

/** \brief Creates a circle or disk. Deprecated.
\deprecated Use makeCircle(const osg::Vec4&,const float,const unsigned int,osg::Geometry*).
*/
OSGWTOOLS_EXPORT osg::Geometry* makeCircle( const float radius=1., const unsigned int subdivisions=32, const osg::Vec3& orientation=osg::Vec3( 0., 0., 1. ), osg::Geometry* geometry=NULL );

/** \brief A transformed circle/disk. Deprecated.
\deprecated Use makeCircle(const osg::Matrix&,const osg::Vec4&,const float,const unsigned int,osg::Geometry*).
*/
OSGWTOOLS_EXPORT osg::Geometry* makeCircle( const osg::Matrix& m, const float radius=1., const unsigned int subdivisions=32, const osg::Vec3& orientation=osg::Vec3( 0., 0., 1. ), osg::Geometry* geometry=NULL );

/** \brief Creates a wireframe circle. Deprecated.
\deprecated Use makeWireCircle(const osg::Vec4&,const float,const unsigned int,osg::Geometry*).
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireCircle( const float radius=1., const unsigned int subdivisions=32, const osg::Vec3& orientation=osg::Vec3( 0., 0., 1. ), osg::Geometry* geometry=NULL );

/** \brief A transformed wireframe circle/disk. Deprecated.
\deprecated Use makeWireCircle(const osg::Matrix&,const osg::Vec4&,const float,const unsigned int,osg::Geometry*).
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireCircle( const osg::Matrix& m, const float radius=1., const unsigned int subdivisions=32, const osg::Vec3& orientation=osg::Vec3( 0., 0., 1. ), osg::Geometry* geometry=NULL );


/** \brief Creates a circle/disk in an arbitrary plane.

Makes a filled disk using a GL_TRIANGLE_FAN PrimitiveSet. The disk is centered at the origin
projected onto \c plane, and has the specified \radius. \c subdivisions specified the number
of triangles in the fan. The disk's normal is taken from the \c plane equation. The function
creates normals per vertex and texture coordinates in unit 0. The texture coordinate data is
suitable for applying an entire texture map to the face of the disk.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeCircle( const osg::Vec4& plane, const float radius=1., const unsigned int subdivisions=32, osg::Geometry* geometry=NULL );

/** \brief A transformed circle/disk in an arbitrary plane.

Equivalent to the non-transformed variant, but transforms all vertices and
normals. See \ref Transform.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeCircle( const osg::Matrix& m, const osg::Vec4& plane, const float radius=1., const unsigned int subdivisions=32, osg::Geometry* geometry=NULL );

/** \brief Wireframe circle in an arbitrary plane.

Equivalent to makeCircle(), but uses a single GL_LINE_LOOP primitive, doesn't create normals
or texture coordinates, and disables lighting and texture mapping.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireCircle( const osg::Vec4& plane, const float radius=1., const unsigned int subdivisions=32, osg::Geometry* geometry=NULL );

/** \brief A transformed wireframe circle/disk in an arbitrary plane.

Equivalent to the non-transformed variant, but transforms all vertices and
normals. See \ref Transform.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireCircle( const osg::Matrix& m, const osg::Vec4& plane, const float radius=1., const unsigned int subdivisions=32, osg::Geometry* geometry=NULL );

/*@}*/


/** \defgroup MiscShapes Miscellaneous shape generation routines */
/*@{*/

/** \brief Creates an arrow shape.

Makes a unit-length arrow pointing toward +z with base at the origin.
Current implementation uses a minimal number of vertices. In the future,
this code could use cylinder/cone support routines to provide complete
control over approximation.
Adds vertex and normal data with a single white color. */
OSGWTOOLS_EXPORT osg::Geometry* makeArrow( osg::Geometry* geometry=NULL );

/** \brief A transformed arrow shape.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeArrow( const osg::Matrix& m, osg::Geometry* geometry=NULL );

/*@}*/


/** \defgroup CylinderShapes Cylinder, cone, and capsule shape generation routines */
/*@{*/

/** \brief Creates an open-ended cylinder.

Creates a cylinder centered on the z axis with ends at z = 0 and z = \c length.
If the end radii \c radius0 and \c radius1 are not equal, the cylinder radius
varies linearly along the cylinger length. Element 0 of \c subdivisions controls
the number of sub-cylinders along the cylinder length, and must be >= 1. Element
1 of \c subdivisions controls the angular approxumation around the cylinder and
must be >= 3.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeOpenCylinder( const double length=1., const double radius0=1., const double radius1=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief A transformed open-ended cylinder.

Equivalent to the non-transformed variant, but transforms all vertices and
normals. See \ref Transform.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeOpenCylinder( const osg::Matrix& m, const double length=1., const double radius0=1., const double radius1=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief Creates an cylinder with optional end caps.

Calls makeOpenCylinder to create a cylinder, then calls makeCircle to create
end caps. Control end cap creation with \c end0 and \c end1.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeClosedCylinder( const double length=1., const double radius0=1., const double radius1=1., const bool cap0=true, const bool cap1=true, const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief Creates a transformed cylinder with optional end caps.

Equivalent to the non-transformed variant, but transforms all vertices and
normals. See \ref Transform.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeClosedCylinder( const osg::Matrix& m, const double length=1., const double radius0=1., const double radius1=1., const bool cap0=true, const bool cap1=true, const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief Creates a wireframe cylinder.

Equivalent to makeOpenCylinder, but uses line primitive types and disables both
lighting and texture mapping.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireCylinder( const double length=1., const double radius0=1., const double radius1=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief Creates a transformed wireframe cylinder.

Equivalent to the non-transformed variant, but transforms all vertices and
normals. See \ref Transform.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireCylinder( const osg::Matrix& m, const double length=1., const double radius0=1., const double radius1=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );


/** \brief Creates a solid cone.

This convenience routine is identical to:

\code
makeClosedCylinder( length, radius, 0., true, false, subdivisions, geometry )( length, radius, 0, syubdivisions, geometry );
\endcode
*/
OSGWTOOLS_EXPORT osg::Geometry* makeCone( const double length=1., const double radius=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief Creates a transformed solid cone.

This convenience routine is identical to:

\code
makeClosedCylinder( m, length, radius, 0., true, false, subdivisions, geometry )( length, radius, 0, syubdivisions, geometry );
\endcode
*/
OSGWTOOLS_EXPORT osg::Geometry* makeCone( const osg::Matrix& m, const double length=1., const double radius=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );


/** \brief Creates a capsule shape.

A capsule is a cylinder with closed hemispherical end caps. The cylinder / capsule
is centered on the z axis. Specify the capsule length with \c length, and the capsule
will have ends at z = 0 and z = \c length. Element 0
of \c subdivisions controls the number of sub-cylinders along the capsule's
cylinder body length, and must be >= 1. Element 1 of \c subdivisions controls the
angular approxumation around the cylinder / capsule and must be >= 3.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeCapsule( const double length=1., const double radius=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief A transformed capsule shape.

Equivalent to the non-transformed variant, but transforms all vertices and
normals. See \ref Transform.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeCapsule( const osg::Matrix& m, const double length=1., const double radius=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief Creates a wireframe capsule shape.

Equivalent to makeCapsule, but uses line primitive types and disables both
lighting and texture mapping.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireCapsule( const double length=1., const double radius=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/** \brief A transformed wireframe capsule shape.

Equivalent to the non-transformed variant, but transforms all vertices and
normals. See \ref Transform.
*/
OSGWTOOLS_EXPORT osg::Geometry* makeWireCapsule( const osg::Matrix& m, const double length=1., const double radius=1., const osg::Vec2s& subdivisions=osg::Vec2s( 1, 8 ), osg::Geometry* geometry=NULL );

/*@}*/

/*@}*/


// namespace osgwTools
}

// __OSGWTOOLS_SHAPES_H__
#endif
