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

#include <osgbCollision/ComputeCylinderVisitor.h>
#include <osgbCollision/BoundingCylinder.h>

#include <osg/Transform>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/PrimitiveSet>

using namespace osg;

namespace osgbCollision
{


/* \cond */
struct ComputeCylinderBound : public osg::PrimitiveFunctor
{
    ComputeCylinderBound()
    {
        _vertices2f = 0;
        _vertices3f = 0;
        _vertices4f = 0;
        _vertices2d = 0;
        _vertices3d = 0;
        _vertices4d = 0;
    }

    virtual void setAxis( const osg::Vec3 a )
    {
        _bb.setAxis( a );
    }

    virtual void setVertexArray( unsigned int,
                                 const Vec2 * vertices )
    {
        _vertices2f = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const Vec3 * vertices )
    {
        _vertices3f = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const Vec4 * vertices )
    {
        _vertices4f = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const Vec2d * vertices )
    {
        _vertices2d = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const Vec3d * vertices )
    {
        _vertices3d = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const Vec4d * vertices )
    {
        _vertices4d = vertices;
    }

    template< typename T >
    void _drawArrays( T * vert,
                      T * end )
    {
        for( ; vert < end; ++vert )
        {
            vertex( *vert );
        }
    }

    template< typename T, typename I >
    void _drawElements( T * vert,
                        I * indices,
                        I * end )
    {
        for( ; indices < end; ++indices )
        {
            vertex( vert[ *indices ] );
        }
    }

    virtual void drawArrays(                                                                                GLenum,
                                                                                                            GLint first,
                                                                                                            GLsizei count )
    {
        if( _vertices3f )
        {
            _drawArrays( _vertices3f + first, _vertices3f + ( first + count ) );
        }
        else if( _vertices2f )
        {
            _drawArrays( _vertices2f + first, _vertices2f + ( first + count ) );
        }
        else if( _vertices4f )
        {
            _drawArrays( _vertices4f + first, _vertices4f + ( first + count ) );
        }
        else if( _vertices2d )
        {
            _drawArrays( _vertices2d + first, _vertices2d + ( first + count ) );
        }
        else if( _vertices3d )
        {
            _drawArrays( _vertices3d + first, _vertices3d + ( first + count ) );
        }
        else if( _vertices4d )
        {
            _drawArrays( _vertices4d + first, _vertices4d + ( first + count ) );
        }
    }

    virtual void drawElements(                                                                                                                                  GLenum,
                                                                                                                                                                GLsizei count,
                                                                                                                                                                const GLubyte * indices )
    {
        if( _vertices3f )
        {
            _drawElements( _vertices3f, indices, indices + count );
        }
        else if( _vertices2f )
        {
            _drawElements( _vertices2f, indices, indices + count );
        }
        else if( _vertices4f )
        {
            _drawElements( _vertices4f, indices, indices + count );
        }
        else if( _vertices2d )
        {
            _drawElements( _vertices2d, indices, indices + count );
        }
        else if( _vertices3d )
        {
            _drawElements( _vertices3d, indices, indices + count );
        }
        else if( _vertices4d )
        {
            _drawElements( _vertices4d, indices, indices + count );
        }
    }

    virtual void drawElements( GLenum, GLsizei count, const GLushort* indices )
    {
        if( _vertices3f )
        {
            _drawElements( _vertices3f, indices, indices + count );
        }
        else if( _vertices2f )
        {
            _drawElements( _vertices2f, indices, indices + count );
        }
        else if( _vertices4f )
        {
            _drawElements( _vertices4f, indices, indices + count );
        }
        else if( _vertices2d )
        {
            _drawElements( _vertices2d, indices, indices + count );
        }
        else if( _vertices3d )
        {
            _drawElements( _vertices3d, indices, indices + count );
        }
        else if( _vertices4d )
        {
            _drawElements( _vertices4d, indices, indices + count );
        }
    }

    virtual void drawElements( GLenum,  GLsizei count, const GLuint* indices )
    {
        if( _vertices3f )
        {
            _drawElements( _vertices3f, indices, indices + count );
        }
        else if( _vertices2f )
        {
            _drawElements( _vertices2f, indices, indices + count );
        }
        else if( _vertices4f )
        {
            _drawElements( _vertices4f, indices, indices + count );
        }
        else if( _vertices2d )
        {
            _drawElements( _vertices2d, indices, indices + count );
        }
        else if( _vertices3d )
        {
            _drawElements( _vertices3d, indices, indices + count );
        }
        else if( _vertices4d )
        {
            _drawElements( _vertices4d, indices, indices + count );
        }
    }

    virtual void begin( GLenum )
    {
    }

    virtual void vertex( const Vec2 & vert )
    {
        _bb.expandBy( osg::Vec3( vert[ 0 ], vert[ 1 ], 0.0f ) );
    }

    virtual void vertex( const Vec3 & vert )
    {
        _bb.expandBy( vert );
    }

    virtual void vertex( const Vec4 & vert )
    {
        if( vert[ 3 ] != 0.0f )
        {
            _bb.expandBy( osg::Vec3( vert[ 0 ], vert[ 1 ], vert[ 2 ] ) / vert[ 3 ] );
        }
    }

    virtual void vertex( const Vec2d & vert )
    {
        _bb.expandBy( osg::Vec3( vert[ 0 ], vert[ 1 ], 0.0f ) );
    }

    virtual void vertex( const Vec3d & vert )
    {
        _bb.expandBy( vert );
    }

    virtual void vertex( const Vec4d & vert )
    {
        if( vert[ 3 ] != 0.0f )
        {
            _bb.expandBy( osg::Vec3( vert[ 0 ], vert[ 1 ], vert[ 2 ] ) / vert[ 3 ] );
        }
    }

    virtual void vertex( float x,
                         float y )
    {
        _bb.expandBy( x, y, 1.0f );
    }

    virtual void vertex( float x,
                         float y,
                         float z )
    {
        _bb.expandBy( x, y, z );
    }

    virtual void vertex( float x,
                         float y,
                         float z,
                         float w )
    {
        if( w != 0.0f )
        {
            _bb.expandBy( x / w, y / w, z / w );
        }
    }

    virtual void vertex( double x,
                         double y )
    {
        _bb.expandBy( x, y, 1.0f );
    }

    virtual void vertex( double x,
                         double y,
                         double z )
    {
        _bb.expandBy( x, y, z );
    }

    virtual void vertex( double x,
                         double y,
                         double z,
                         double w )
    {
        if( w != 0.0f )
        {
            _bb.expandBy( x / w, y / w, z / w );
        }
    }

    virtual void end()
    {
    }

    const Vec2 *      _vertices2f;
    const Vec3 *      _vertices3f;
    const Vec4 *      _vertices4f;
    const Vec2d *     _vertices2d;
    const Vec3d *     _vertices3d;
    const Vec4d *     _vertices4d;
    BoundingCylinder _bb;
};
/* \endcond */


ComputeCylinderVisitor::ComputeCylinderVisitor( osg::NodeVisitor::TraversalMode traversalMode )
    : osg::NodeVisitor( traversalMode )
{
}

void ComputeCylinderVisitor::reset()
{
    stack.clear();
    bc.init();
    bc.setAxis( axis );
}

void ComputeCylinderVisitor::apply( osg::Transform & transform )
{
    osg::Matrix matrix;

    if( !stack.empty() )
    {
        matrix = stack.back();
    }

    transform.computeLocalToWorldMatrix( matrix, this );

    pushMatrix( matrix );

    traverse( transform );

    popMatrix();
}

void ComputeCylinderVisitor::apply( osg::Geode & geode )
{
    for( unsigned int i = 0; i < geode.getNumDrawables(); ++i )
    {
        applyDrawable( geode.getDrawable( i ) );
    }
}

void ComputeCylinderVisitor::applyDrawable( osg::Drawable * drawable )
{
    ComputeCylinderBound cbc;

    cbc.setAxis( axis );
    drawable->accept( cbc );

    if( stack.empty() )
    {
        bc.expandBy( cbc._bb );
    }
    else
    {
        BoundingCylinder newbc;
        osg::Matrix & matrix = stack.back();
        newbc.setAxis( osg::Matrix::transform3x3( cbc._bb.getAxis(), matrix ) );
        newbc.setLength( cbc._bb.getLength() );
        newbc.setRadius( cbc._bb.getRadius() );
        bc.expandBy( newbc );
    }
}


// osgbCollision
}
