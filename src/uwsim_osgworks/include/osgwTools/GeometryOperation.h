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

#ifndef __GEOMETRY_OPERATION_H__
#define __GEOMETRY_OPERATION_H__

#include <osg/Object>
#include <osg/Geometry>
#include <osgwTools/Export.h>

namespace osgwTools {


/** \brief Base class for performing operations on osg::Geometry objects in collaboration
with the GeometryModifier class

\test geometryop
*/
class OSGWTOOLS_EXPORT GeometryOperation : public osg::Object
{
public:
    GeometryOperation();

    /** Derived classes must override this method and return
     a pointer to a new Geometry object. */
    virtual osg::Geometry* operator()( osg::Geometry& geom ) = 0;

    /** \brief Currently for benefit of ShortEdgeOp only. */
    virtual void setMaxSteps( unsigned int maxSteps){ };

protected:
    virtual ~GeometryOperation();
};

}

#endif

/** \page geomopt Overview of osgWorks Polygon Decimation Tools

The osgWorks Polygon Decimation tools reduce scene graph geometry. The 
decimation process output is a simplified geometric representation suitable 
for use as a Bullet triangle mesh collision shape and not necessarily suitable 
for visual display.

We've created an infrastructure that allows generic operations on an osg::Geometry object. 
The GeometryModifier class is a NodeVisitor with a reference to a GeometryOperation object. 
The GeometryModifier calls the GeometryOperation for every Geometry object in the scene graph.

To use the GeometryModifier, you create and configure a GeometryOperation subclass and pass 
it to the GeometryModifier constructor. Then call accept() to invoke the GeometryModifier 
visitor on your scene graph.

In essence, the GeometryModifier itself is somewhat uninteresting. The subclasses 
(specializations) of GeometryOperation perform the actual geometric reduction. We've created 
three subclasses, which are

\li SimplifierOp
\li DecimatorOp
\li ReducerOp 

The following is each subclass behavior.

<b>SimplifierOp</b>

This is a wrapper around the osgUtil::Simplifier. It invokes the Simplifier on the current 
Geometry and serves as a baseline test for the overall GeometryModifier infrastructure. It 
should produce identical results to running the Simplifier, and doesn't provide any additional 
features beyond what the Simplifier provides.

As background information, the Simplifier uses the edge collapse algorithm to reduce vertex 
and triangle count. It only modifies interior vertices and doesn't affect edge geometry.

\image html geom-edgecollapse.jpg

<em> (Figure description: </em><b>The osgUtil::Simplifier edge collapse algorithm.</b> Starting with the mesh 
on the left, consisting of ten triangles and ten vertices, the Simplifier selects the two 
highlighted vertices and triangles, and collapses the edge between them, introducing a new 
vertex. The resulting simplified mesh consists of eight triangles and nine vertices.)

<b>DecimatorOp</b>

The DecimatorOp is a modification of the Simplifier that uses the half-edge collapse algorithm 
and performs operations on edge geometry. Unlike the edge collapse algorithm, the half-edge 
collapse removes vertices without computing and introducing new vertices. Combined with the 
ability to operate on edge geometry, the DecimatorOp is a more general decimation tool than 
the SimplifierOp.

\image html geom-halfedgecollapse.jpg

<em>(Figure description:</em> <b>The DecimatorOp half-edge collapse algorithm.</b> Initially, the algorithm 
selects three edges for collapse, and removes one vertex from each of the three edges, leaving 
six triangles and seven vertices. In the second iteration, the algorithm selects a final edge 
for collapse and removes one of its vertices, leaving four triangles and six vertices.)

The DecimatorOp features the following control parameters:
<table border="0">
  <tr>
    <td><b>Percentage of triangles to retain</b></td>
	<td>The DecimatorOp collapses edges in order from least 
to greatest error, until the specified percentage of triangles remain.</td>
</tr>
<tr>
<td><b>Max error</b></td>
<td>The DecimatorOp does not collapse an edge if it introduces more error than 
this threshold specifies when computed as a change in normals of adjacent triangles. The 
default is FLT_MAX, which lets the DecimatorOp collapse as many edges as necessary.</td>
</tr>
<tr>
<td><b>Smoothing</b></td>
<td>If this Boolean is true, the DecimatorOp runs the osgUtil SmoothingVisitor 
to generate per-vertex normals. The default is false. </td>
</tr>
<tr>
<td><b>Tristrip</b></td>
<td>If this Boolean is true, the DecimatorOp runs the osgUtil TristripVisitor. The 
default is false.</td>
</tr>
<tr>
<td><b>Ignore boundaries</b></td>
<td>If this Boolean is true, the DecimatorOp does not collapse any boundary 
edges. The default is false (it processes boundary edges). </td>
</tr>
<tr>
<td><b>Minimum Primitives</b></td> 
<td>The DecimatorOp operates only on Geometry with an initial primitive 
count equal to or greater than this value. The default is six. </td>
</tr>
</table>

The DecimatorOp differs from the osgUtil::Simplifier in the following functional areas:

<table border="0">
<tr>
<td><b>ComputeOptimalPoint</b></td> 
<td>This function formerly computed the mid-point on an edge to be collapsed. 
In DecimatorOp, this function determines which vertex to remove on the candidate edge, based on 
minimizing error and preserving geometric features.</td>
</tr>
<tr>
<td><b>ComputeErrorMetric</b></td>
<td>We modified this function to not eliminate boundary edges from 
consideration.</td>
</tr>
</table>

<b>ReducerOp</b>

The ReducerOp is an alternative to the DecimatorOp based on concepts described here:
http://www.cg.tuwien.ac.at/courses/Seminar/SS2002/Knapp_paper.pdf

For each vertex, the set of triangles sharing that vertex is considered. The set of triangles 
is broken into multiple groups based on the group threshold. If the vertex is within the center 
of a group, the algorithm removes that vertex.  If the vertex is on the edge of a group, the 
algorithm removes it if the boundary edges do not exceed the maximum edge error. In either case, 
if the algorithm removes the vertex, we triangulate the remaining vertices from the triangle 
to preserve the vertex winding order. 

\image html geom-reducergroup.jpg

<em>(Figure description:</em> <b>ReducerOp group algorithm.</b> The set of triangles on the left all share a 
common vertex (gray highlight), but have different normals (the triangle color indicates this). 
ReducerOp groups triangles together if their normals are within the group threshold. In this 
case, it identifies two groups, as the center image shows. ReducerOp then considers each group 
individually to determine if the current vertex is removable, as those on the right show.)

\image html geom-reducerremove.jpg

<em>(Figure description: </em> <b>ReducerOp vertex removal.</b> For each group, ReducerOp determines whether the 
current vertex is completely contained (left) or on an edge (right). If completely contained, 
ReducerOp removes the vertex. If on an edge, ReducerOp calculates the angle subtended by the 
adjacent boundary edges. If the angle is less than the max error edge, the vertex is removed. 
In either case, when a vertex is removed, ReducerOp triangulates the remaining vertices 
arbitrarily, but in such a way as to preserve vertex winding order.)

ReducerOp features the following controls:

* Group threshold. Triangles form a group when they share one or more vertices and when 
their normals differ by less than this specified angle in degrees. The default is ten degrees.
* Max edge error. ReducerOp only removes a vertex from the edge of a triangle group if the 
angle the group boundary edges subtends differs by less than this specified angle in degrees. 
The default is ten degrees.
* Remove degenerate and redundant triangles. If set to true, ReducerOp removes degenerate 
triangles (triangles with less than three unique vertices) and redundant triangles (triangles 
that appear more than once). If set to false, ReducerOp does  not remove vertices referenced 
by degenerate and redundant triangles reference.

<em>ReducerOp Limitations</em>

ReducerOp only operates on Geometry objects with Vec3Array-type vertices, and DrawElementsUInt 
PrimitiveSets with TRIANGLES mode. This limits the scene graphs ReducerOp can operate on, and 
this limitation will be removed in a future release of osgWorks.

If ReducerOp currently fails a test case, you can demonstrate it with the following command:

\c DecimatorDemo-reducer \c dectest32.osg 

<em>Comparing ReducerOp to DecimatorOp</em>

The ReducerOp removes vertices by removing indexes from the OSG PrimitiveSet, and does not 
remove vertices from the VertexArray. As a result, ReducerOp is extremely fast compared to 
the DecimatorOp. We currently do not understand why DecimatorOp is  slow, but it is comparable 
in performance to SimplifierOp.

DecimatorOp lets the app specify a desired percentage, and it removes as many vertices as 
needed to achieve that percentage goal. ReducerOp does not provide this interface, and instead 
only removes vertices the specified thresholds allow. As a result, DecimatorOp can negatively 
impact the visual appearance of models when it runs aggressively, whereas ReducerOp generally 
does a better job of preserving model appearance.

*/

