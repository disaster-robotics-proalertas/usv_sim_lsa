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

#ifndef __OSGBCOLLISION_VERSION_H__
#define __OSGBCOLLISION_VERSION_H__ 1

#include <osgbCollision/Export.h>
#include <string>


namespace osgbCollision {


// Please keep in sync with top-level CMakeLists.txt OSGBULLET_VERSION variable.
#define OSGBCOLLISION_MAJOR_VERSION (3)
#define OSGBCOLLISION_MINOR_VERSION (0)
#define OSGBCOLLISION_SUB_VERSION (0)

// C preprocessor integrated version number.
// The form is Mmmss, where:
//   M is the major version
//   mm is the minor version (zero-padded)
//   ss is the sub version (zero padded)
// Use this in version-specific code, for example:
//   #if( OSGBCOLLISION_VERSION < 10500 )
//      ... code specific to releases before v1.05
//   #endif
#define OSGBCOLLISION_VERSION ( \
        ( OSGBCOLLISION_MAJOR_VERSION * 10000 ) + \
        ( OSGBCOLLISION_MINOR_VERSION * 100 ) + \
          OSGBCOLLISION_SUB_VERSION )

// Returns OSGBCOLLISION_VERSION.
unsigned int OSGBCOLLISION_EXPORT getVersionNumber();

// Pretty string.
std::string OSGBCOLLISION_EXPORT getVersionString();


// Backwards compatibility
#define OSGBBULLET_MAJOR_VERSION OSGBCOLLISION_MAJOR_VERSION
#define OSGBBULLET_MINOR_VERSION OSGBCOLLISION_MINOR_VERSION
#define OSGBBULLET_SUB_VERSION OSGBCOLLISION_SUB_VERSION
#define OSGBBULLET_VERSION OSGBCOLLISION_VERSION


// namespace osgbCollision
}


// __OSGBCOLLISION_VERSION_H__
#endif


/** \mainpage osgBullet Documentation

\section Introduction Introduction

osgBullet is a set of software tools for applications that use both
<a href="http://www.openscenegraph.org/">OpenSceneGraph (OSG)</a> and
<a href="http://code.google.com/p/bullet/">Bullet</a>. The osgBullet library
is the result of collaborative work between Paul Martz
(<a href="http://www.skew-matrix.com/">Skew Matrix Software</a>),
<a href="http://www.ameslab.gov/">Ames Lab</a>, and
<a href="http://www.pica.army.mil/PicatinnyPublic/index.asp">ARDEC</a>.
It's used as part of the 
<a href="http://www.ve-suite.org/">VE-Suite</a> project, as well as
other applications and software projects. osgBullet is open source and
available under the GNU LGPL v2.1 software license. 

osgBullet <a href="http://code.google.com/p/osgbullet/">source and
issue tracking</a> are on Google Code.

An osgBullet <a href="http://groups.google.com/group/osgbullet-users?hl=en">discussion
group</a> is available on Google Groups.


\subsection phil Philosophy

osgBullet is a set of tools to facilitate developing software that uses Bullet
for physics simultation and collision detection, and uses OSG for rendering.
osgBullet doesn't attempt to wrap either OSG or Bullet. Instead, it allows you
(the programmer) direct access to both APIs. osgBullet gets out of your way
so that your application can use the full feature set of both OSG and Bullet.

osgBullet plays a key role in this scenario by providing useful tools for applications
that use both APIs.

\subsection feat Features

osgBullet's most useful feature is its ability to accomodate Bullet's strict coordinate
system and transformation requirements and still support the arbitrary coordinate systems
and transformations often encountered in OSG (and other 3D) applications. Bullet collision
shapes must be created so that the center of mass corresponds to the coordinate origin,
but this is not how most 3D models are built. Bullet doesn't support scaling, but 3D models
are often scaled up or down to match a world coordinate system. OSG transformations are
hierarchical, but Bullet wants to completely own the local-to-world transformation. All
of these issues are handled by the
\link collisionshapes collision shape creation utilities \endlink
in combination with the \ref osgbDynamics::MotionState "MotionState" class. osgBullet also provides
\link rigidbody rigid body creation utilities \endlink
that wrap collision shape and \ref osgbDynamics::MotionState "MotionState" creation.

Other features include the following.

\li Support for running physics and rendering in separate threads.
\li Supports using Bullet for both rigid body dynamics as well as collision detection only.
\li A set of osgGA-based GUIEventHandlers for interacting with the physics simulation, including
the osgbInteraction::HandNode class to support data glove usage.
\li Routines for creating Bullet collision shapes from OSG geometry.
\li An OSG-based implementation of \c \c btIDebugDraw.
\li OSG reference counting for Bullet objects.
\li Functions to convert between OSG and Bullet matrices and vectors.


\section appsexamples Applications and Examples

\li The \ref osgbpp "osgbpp" application allows you to preview a physics simultation on a model.
\li The \ref examplecom "centerofmass" example demonstrates application-specified center of mass.
\li The \ref collision "collision" example demonstrates using osgBullet and Bullet for collision detection only.
\li The \ref diceexample "dice" example, just for fun.
\li The \ref handphysicsexample "hand physics" example demonstrates using the \c HandNode to interact with the scene.
\li The \ref hingelowlevel "hinge" example demonstrates creating a hinge constraint.
\li The \ref multithreaded "multithreaded" example demonstrates running Bullet in a separate thread.
\li The \ref saverestoreexample "saverestore" example demonstrates saving/restoring a physics simultation to/from disk.
\li The \ref sliderlowlevel "slider" example demonstrates creating a slider constraint.

\section libraries Libraries

\subsection osgbcollision osgbCollision

Collision detection and collision shape support. Facilities for
creating Bullet collision shapes from OSG scene graphs, and vice
versa.

\subsection osgbdynamics osgbDynamics

Rigid body dynamics and constraints support.

\subsection osgbinteraction osgbInteraction

Support for user interaction with the physics simultation, such as dragging
objects, resetting the simulation to a save point, and launching models into
the scene. An articulatable hand model supports an immersive simultation
experience, and includes aupport for the P5 data glove.

\subsection osgdbosgbdynamics osgdb_osgbDynamics

Dot OSG file support for classes and objects in the osgbDynamics library.

Note: This library currently needs to be redesigned. It provides support for,
and depends on, all three libraries: osgbCollision, osgbDynamics, and
osgbInteraction. This makes it impossible for an app to both use the dot OSG
file support, and link with only osgbDynamics (it must also link with
osgbInteraction).

\subsection osgdbsgb osgdb_sgb

Support for .SGB, an osgBullet file format for storing physics state.

*/
