/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2011 by Kenneth Mark Bryden
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
attribute vec4 a_tangent;
attribute vec4 a_binormal;

varying vec3 v_lightVector;
varying vec3 v_viewVector;

void main()
{
    //
    gl_Position = ftransform();

    //Get the texture coordinates
    gl_TexCoord[ 0 ] = gl_TextureMatrix[ 0 ] * gl_MultiTexCoord0;

    //Convert the vertex position into eye coordinates
    vec3 ecPosition = vec3( gl_ModelViewMatrix * gl_Vertex );

    //Convert tangent, binormal, and normal into eye coordinates
    mat3 TBNMatrix = mat3( gl_ModelViewMatrix[0].xyz,gl_ModelViewMatrix[1].xyz,gl_ModelViewMatrix[2].xyz ) *
                     mat3( a_tangent.xyz, a_binormal.xyz, gl_Normal );

    //Convert light vector into tangent space
    v_lightVector = gl_LightSource[ 0 ].position.xyz - ecPosition;
    v_lightVector *= TBNMatrix;

    //Convert view vector into tangent space
    v_viewVector = ecPosition;
    v_viewVector *= TBNMatrix;
}
