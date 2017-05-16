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

#ifndef __DECIMATIONTEST_MODEL_H__
#define __DECIMATIONTEST_MODEL_H__

#include <osgwTools/Export.h>
#include <osgwTools/GeometryOperation.h>
#include <osg/Geode>
#include <osg/Geometry>

namespace osgwTools {

class DecimationTestModel
{
public:
    DecimationTestModel(int numParts)  { _decimationTestGeode = buildModel(numParts); }

    osg::Geode* getModel(void) const { return (_decimationTestGeode); }

	osg::Geode* _decimationTestGeode;
    #define decimationTestSidePoints 63
    #define decimationTestEdgePoints (decimationTestSidePoints * 2)

    void buildOpening(osg::Vec3Array* vertices, osg::Vec3Array* normals, float const xOff, float const yOff, float const zOff) const
    {
	    vertices->push_back( osg::Vec3( xOff, 1 + yOff, zOff) ); // 0
	    vertices->push_back( osg::Vec3( xOff, 2 + yOff, zOff) ); // 1
	    vertices->push_back( osg::Vec3( 1 + xOff, 3 + yOff, zOff) ); // 2
	    vertices->push_back( osg::Vec3( 2 + xOff, 3 + yOff, zOff) ); // 3
	    vertices->push_back( osg::Vec3( 3 + xOff, 2 + yOff, zOff) ); // 4
	    vertices->push_back( osg::Vec3( 3 + xOff, 1 + yOff, zOff) ); // 5
	    vertices->push_back( osg::Vec3( 2 + xOff,  yOff, zOff) ); // 6
	    vertices->push_back( osg::Vec3( 1 + xOff,  yOff, zOff) ); // 7

        unsigned int loops = vertices->size() - normals->size();
        float normalDirection = zOff > 0.0 ? 1.0: -1.0;
        for (unsigned int i=0; i<loops; ++i)
        {
            normals->push_back( osg::Vec3( 0, 0, normalDirection) );
        }
    }

    void buildSide(osg::Vec3Array* vertices, osg::Vec3Array* normals, float const zOff) const
    {
	    vertices->push_back( osg::Vec3( 0, 5, zOff) ); // 0
	    vertices->push_back( osg::Vec3( 1, 3, zOff) ); // 1
	    vertices->push_back( osg::Vec3( 3, 1, zOff) ); // 2 
	    vertices->push_back( osg::Vec3( 5, 0, zOff) ); // 3 
	    vertices->push_back( osg::Vec3( 32, 0, zOff) ); // 4
	    vertices->push_back( osg::Vec3( 35, 1, zOff) ); // 5
	    vertices->push_back( osg::Vec3( 37, 3, zOff) ); // 6
	    vertices->push_back( osg::Vec3( 38, 5, zOff) ); // 7
	    vertices->push_back( osg::Vec3( 38, 23, zOff) ); // 8
	    vertices->push_back( osg::Vec3( 37, 25, zOff) ); // 9
	    vertices->push_back( osg::Vec3( 35, 27, zOff) ); // 10
	    vertices->push_back( osg::Vec3( 33, 28, zOff) ); // 11
	    vertices->push_back( osg::Vec3( 31, 28, zOff) ); // 12
	    vertices->push_back( osg::Vec3( 30, 27, zOff) ); // 13
	    vertices->push_back( osg::Vec3( 23, 27, zOff) ); // 14
	    vertices->push_back( osg::Vec3( 23, 26, zOff) ); // 15
	    vertices->push_back( osg::Vec3( 22, 25, zOff) ); // 16
	    vertices->push_back( osg::Vec3( 19, 25, zOff) ); // 17
	    vertices->push_back( osg::Vec3( 19, 27, zOff) ); // 18
	    vertices->push_back( osg::Vec3( 5, 27, zOff) ); // 19
	    vertices->push_back( osg::Vec3( 3, 26, zOff) ); // 20
	    vertices->push_back( osg::Vec3( 1, 24, zOff) ); // 21
	    vertices->push_back( osg::Vec3( 0, 22, zOff) ); // 22

        unsigned int loops = vertices->size() - normals->size();
        float normalDirection = zOff > 0.0 ? 1.0: -1.0;
        for (unsigned int i=0; i<loops; ++i)
        {
            normals->push_back( osg::Vec3( 0, 0, normalDirection) );
        }

        buildOpening(vertices, normals, 5.0, 4.0, zOff);
        buildOpening(vertices, normals, 16.0, 4.0, zOff);
        buildOpening(vertices, normals, 20.0, 4.0, zOff);
        buildOpening(vertices, normals, 25.0, 4.0, zOff);
        buildOpening(vertices, normals, 34.0, 4.0, zOff);
    }

    bool triangulateSide(osg::Geometry* geometry, osg::Vec3Array* vertices, int winding, int vertexOffset) const
    {
        if (vertices->size() < decimationTestSidePoints)
        {
            osg::notify( osg::FATAL ) << "Can't create input model. Incorrect number of vertices for side." << std::endl;
            return false;
        }

        // make a list of the vertices to make triangles from
        osg::Vec3sArray* triangles = new osg::Vec3sArray;
        triangles->push_back( osg::Vec3s(0,23,24) );
        triangles->push_back( osg::Vec3s(0,24,25) );
        triangles->push_back( osg::Vec3s(0,25,22) );
        triangles->push_back( osg::Vec3s(22,25,26) );
        triangles->push_back( osg::Vec3s(22,26,17) );
        triangles->push_back( osg::Vec3s(22,17,21) );
        triangles->push_back( osg::Vec3s(21,17,19) );
        triangles->push_back( osg::Vec3s(21,19,20) );
        triangles->push_back( osg::Vec3s(19,17,18) );
        triangles->push_back( osg::Vec3s(17,26,33) );
        triangles->push_back( osg::Vec3s(17,33,34) );
        triangles->push_back( osg::Vec3s(17,34,41) );
        triangles->push_back( osg::Vec3s(17,41,16) );
        triangles->push_back( osg::Vec3s(16,41,42) );
        triangles->push_back( osg::Vec3s(16,42,49) );
        triangles->push_back( osg::Vec3s(16,49,50) );
        triangles->push_back( osg::Vec3s(16,50,57) );
        triangles->push_back( osg::Vec3s(16,57,8) );
        triangles->push_back( osg::Vec3s(16,8,13) );
        triangles->push_back( osg::Vec3s(16,13,15) );
        triangles->push_back( osg::Vec3s(15,13,14) );
        triangles->push_back( osg::Vec3s(13,8,10) );
        triangles->push_back( osg::Vec3s(13,10,11) );
        triangles->push_back( osg::Vec3s(13,11,12) );
        triangles->push_back( osg::Vec3s(10,8,9) );
        triangles->push_back( osg::Vec3s(8,57,58) );
        triangles->push_back( osg::Vec3s(8,58,59) );
        triangles->push_back( osg::Vec3s(8,59,7) );
        triangles->push_back( osg::Vec3s(7,59,60) );
        triangles->push_back( osg::Vec3s(7,60,61) );
        triangles->push_back( osg::Vec3s(7,61,6) );
        triangles->push_back( osg::Vec3s(6,61,62) );
        triangles->push_back( osg::Vec3s(6,62,5) );
        triangles->push_back( osg::Vec3s(5,62,55) );
        triangles->push_back( osg::Vec3s(5,55,4) );
        triangles->push_back( osg::Vec3s(4,55,52) );
        triangles->push_back( osg::Vec3s(4,52,53) );
        triangles->push_back( osg::Vec3s(4,53,54) );
        triangles->push_back( osg::Vec3s(4,54,45) );
        triangles->push_back( osg::Vec3s(4,45,46) );
        triangles->push_back( osg::Vec3s(4,46,37) );
        triangles->push_back( osg::Vec3s(4,37,3) );
        triangles->push_back( osg::Vec3s(3,37,38) );
        triangles->push_back( osg::Vec3s(3,38,31) );
        triangles->push_back( osg::Vec3s(3,31,28) );
        triangles->push_back( osg::Vec3s(3,28,29) );
        triangles->push_back( osg::Vec3s(3,29,30) );
        triangles->push_back( osg::Vec3s(3,30,2) );
        triangles->push_back( osg::Vec3s(2,30,23) );
        triangles->push_back( osg::Vec3s(2,23,1) );
        triangles->push_back( osg::Vec3s(1,23,0) );
        triangles->push_back( osg::Vec3s(26,27,33) );
        triangles->push_back( osg::Vec3s(27,32,33) );
        triangles->push_back( osg::Vec3s(27,28,32) );
        triangles->push_back( osg::Vec3s(28,31,32) );
        triangles->push_back( osg::Vec3s(34,35,41) );
        triangles->push_back( osg::Vec3s(35,40,41) );
        triangles->push_back( osg::Vec3s(35,36,40) );
        triangles->push_back( osg::Vec3s(36,39,40) );
        triangles->push_back( osg::Vec3s(36,37,39) );
        triangles->push_back( osg::Vec3s(37,46,39) );
        triangles->push_back( osg::Vec3s(42,43,49) );
        triangles->push_back( osg::Vec3s(43,48,49) );
        triangles->push_back( osg::Vec3s(43,44,48) );
        triangles->push_back( osg::Vec3s(44,47,48) );
        triangles->push_back( osg::Vec3s(44,45,47) );
        triangles->push_back( osg::Vec3s(45,54,47) );
        triangles->push_back( osg::Vec3s(50,51,57) );
        triangles->push_back( osg::Vec3s(51,56,57) );
        triangles->push_back( osg::Vec3s(51,52,56) );
        triangles->push_back( osg::Vec3s(52,55,56) );

        // create the triangle primitives
        for (osg::Vec3sArray::iterator it = triangles->begin(); it != triangles->end(); ++it)
        {
            osg::DrawElementsUInt* newFace;

	        newFace = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 );
            if (winding > 0)
            {
	            newFace->push_back(it->x() + vertexOffset);
	            newFace->push_back(it->y() + vertexOffset);
	            newFace->push_back(it->z() + vertexOffset);
            }
            else
            {
	            newFace->push_back(it->x() + vertexOffset);
	            newFace->push_back(it->z() + vertexOffset);
	            newFace->push_back(it->y() + vertexOffset);
            }
	        geometry->addPrimitiveSet(newFace);
        }

        return true;
    }

    bool triangulateEdge(osg::Geometry* geometry, osg::Vec3Array* vertices, int vertexOffset) const
    {
        if (vertices->size() < decimationTestEdgePoints)
        {
            osg::notify( osg::FATAL ) << "Can't create input model. Incorrect number of vertices for edge." << std::endl;
            return false;
        }

        // make a list of the vertices to make triangles from
        osg::Vec3sArray* triangles = new osg::Vec3sArray;
        for (int ptCt = 0; ptCt < 22; ++ptCt)
        {
            triangles->push_back( osg::Vec3s(ptCt, ptCt + 1 + decimationTestSidePoints, ptCt + decimationTestSidePoints) );
            triangles->push_back( osg::Vec3s(ptCt, ptCt + 1, ptCt + 1 + decimationTestSidePoints) );
        }
        triangles->push_back( osg::Vec3s(22, decimationTestSidePoints, 22 + decimationTestSidePoints) );
        triangles->push_back( osg::Vec3s(22, 0, decimationTestSidePoints) );

        for (int holeCt = 0; holeCt < 5; ++holeCt)
        {
            int holeCtOffset = 23 + holeCt * 8;
            for (int ptCt = holeCtOffset; ptCt < holeCtOffset + 7; ++ptCt)
            {
                triangles->push_back( osg::Vec3s(ptCt, ptCt + 1 + decimationTestSidePoints, ptCt + decimationTestSidePoints) );
                triangles->push_back( osg::Vec3s(ptCt, ptCt + 1, ptCt + 1 + decimationTestSidePoints) );
            }
        triangles->push_back( osg::Vec3s(holeCtOffset + 7, holeCtOffset + decimationTestSidePoints, holeCtOffset + 7 + decimationTestSidePoints) );
        triangles->push_back( osg::Vec3s(holeCtOffset + 7, holeCtOffset, holeCtOffset + decimationTestSidePoints) );
        }

        // create the triangle primitives
        for (osg::Vec3sArray::iterator it = triangles->begin(); it != triangles->end(); ++it)
        {
            osg::DrawElementsUInt* newFace;

	        newFace = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 );
            newFace->push_back(it->x() + vertexOffset);
            newFace->push_back(it->y() + vertexOffset);
            newFace->push_back(it->z() + vertexOffset);
	        geometry->addPrimitiveSet(newFace);
        }
        return true;
    }

    osg::Geode* buildModel(int buildParts) const
    {
	    osg::Geode* complexGeode = new osg::Geode();

        if (buildParts == 0 || buildParts == 4)
        {
            // build all model connected as one drawable with all points shared or not
            // detailed notes are in else block below
            // buildparts = 0 means that two sets of vertices are created - one for the front and one for the back of the thin model. front and back are connected with
            // triangles using the same two sets of vertices.
            // buildParts = 4 means that there are four sets of vertices, two for the front and two for the back so that edge triangles do not share vertices with front and back
            // In both cases, 0 and 4, all primitives will be contained in one drawable
	        osg::Geometry* complexGeometry = new osg::Geometry();
            complexGeode->addDrawable(complexGeometry); 
            osg::Vec3Array* complexVertices = new osg::Vec3Array;
            osg::Vec3Array* complexNormals = new osg::Vec3Array;
            float zOff = 1.0;
            buildSide(complexVertices, complexNormals, 0.0);
            if (buildParts == 4)
                buildSide(complexVertices, complexNormals, 0.0);
            buildSide(complexVertices, complexNormals, zOff);
            if (buildParts == 4)
                buildSide(complexVertices, complexNormals, zOff);

            //Associate this set of vertices with the geometry associated with the geode we added to the scene.
            complexGeometry->setVertexArray( complexVertices ); 
            complexGeometry->setNormalArray( complexNormals ); 
            complexGeometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX ); 

            // create primitives
            triangulateSide(complexGeometry, complexVertices, 1, 0);
            if (buildParts == 0)
            {
                triangulateSide(complexGeometry, complexVertices, -1, decimationTestSidePoints);
                triangulateEdge(complexGeometry, complexVertices, 0);
            }
            else
            {
                triangulateEdge(complexGeometry, complexVertices, decimationTestSidePoints);
                triangulateSide(complexGeometry, complexVertices, -1, decimationTestSidePoints * 3);
            }
        }
        else
        {
            // build one side of model
            for (int part = 0; part < buildParts; ++part)
            {
	            osg::Geometry* complexGeometry = new osg::Geometry();

	            //Next we need to associate the pyramid geometry with the pyramid geode and add the pyramid geode to the root node of the scene graph.

	            complexGeode->addDrawable(complexGeometry); 

                if (part == 0 || part == 2)
                {
                    // build a side of the model separated by 1 unit
	                //Declare an array of vertices. Each vertex will be represented by a triple -- an instances of the vec3 class. An instance of osg::Vec3Array can be used to store these triples. Since osg::Vec3Array is derived from the STL vector class, we can use the push_back method to add array elements. Push back adds elements to the end of the vector, thus the index of first element entered is zero, the second entries index is 1, etc.[[BR]]
	                //Using a right-handed coordinate system with 'z' up, array elements represent points in the object

	                osg::Vec3Array* complexVertices = new osg::Vec3Array;
	                osg::Vec3Array* complexNormals = new osg::Vec3Array;
                    float zOff = ( part == 0 ? 0.0: 1.0 );
                    buildSide(complexVertices, complexNormals, zOff);

	                //Associate this set of vertices with the geometry associated with the geode we added to the scene.
	                complexGeometry->setVertexArray( complexVertices ); 
	                complexGeometry->setNormalArray( complexNormals ); 
                    complexGeometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX ); 

                    // create primitives
                    triangulateSide(complexGeometry, complexVertices, part == 0 ? 1: -1, 0);
                }
                else
                {
                    // build model edge
	                osg::Vec3Array* complexVertices = new osg::Vec3Array;
	                osg::Vec3Array* complexNormals = new osg::Vec3Array;
                    float zOff = 1.0;
                    // build two side points
                    buildSide(complexVertices, complexNormals, 0.0);
                    buildSide(complexVertices, complexNormals, zOff);
         	        
                    //Associate this set of vertices with the geometry associated with the geode we added to the scene.
	                complexGeometry->setVertexArray( complexVertices ); 
	                complexGeometry->setNormalArray( complexNormals ); 
                    complexGeometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX ); 

                    // create primitives
                    triangulateEdge(complexGeometry, complexVertices, 0);
                }
            }
        }

	    return complexGeode;
    } // osg::Geode *buildModel

};

}

#endif