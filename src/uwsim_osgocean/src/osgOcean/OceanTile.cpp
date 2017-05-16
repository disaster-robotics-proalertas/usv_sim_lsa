/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/
#include <stdlib.h> // Need to include this for linux compatibility not sure why.
#include <osgOcean/OceanTile>

#ifdef DEBUG_DATA
#include <osgDB/WriteFile>
#include <fstream>
#include <sstream>
#endif

using namespace osgOcean;

OceanTile::OceanTile( void )
    :_resolution   (0)
    ,_rowLength    (0)
    ,_numVertices  (0)
    ,_spacing      (0)
    ,_maxDelta     (0)
    ,_averageHeight(0)
    ,_maxHeight    (0)
{}

OceanTile::OceanTile( osg::FloatArray* heights, 
                      unsigned int resolution, 
                      const float spacing, 
                      osg::Vec2Array* displacements,
                      bool useVBO)
    
    :_resolution ( resolution )
    ,_rowLength  ( _resolution + 1 )
    ,_numVertices( _rowLength*_rowLength )
    ,_vertices   ( new osg::Vec3Array )
    ,_normals    ( new osg::Vec3Array(_numVertices) )
    ,_spacing    ( spacing )
    ,_maxDelta   ( 0.f )
    ,_useVBO     ( useVBO )
{
    _vertices->reserve( _numVertices );

#ifdef DEBUG_DATA
    static int count = 0;
    std::stringstream ss;
    ss << "Tile_" << count << ".txt";
    std::ofstream outFile( ss.str().c_str() );
    outFile << _rowLength << std::endl;
    ++count;
#endif

    unsigned int x1,y1;
    float sumHeights = 0.f;
    float maxHeight = -FLT_MAX;
    osg::Vec3f v;

    for(int y = 0; y <= (int)_resolution; ++y )
    {
        y1 = y % _resolution;

        for(int x = 0; x <= (int)_resolution; ++x )
        {
            x1 = x % _resolution;

            unsigned int ptr = array_pos(x1,y1,_resolution);

            if (_useVBO) 
            {
                v.x() = x * spacing;
                v.y() = -y * spacing;
            }
            else
            {
                v.x() = 0.0;
                v.y() = 0.0;
            }
            if (displacements)      // Displacements are optional, default value is NULL
            {
                v.x() = v.x() + displacements->at(ptr).x();
                v.y() = v.y() + displacements->at(ptr).y();
            }

            v.z() = heights->at( ptr );

#ifdef DEBUG_DATA
            outFile << v.x() << std::endl;
            outFile << v.y() << std::endl;
            outFile << v.z() << std::endl;
#endif
            sumHeights += v.z();
            maxHeight = osg::maximum(maxHeight, v.z());

            _vertices->push_back( v );
        }
    }

#ifdef DEBUG_DATA
    outFile.close();
#endif

    _averageHeight = sumHeights / (float)_vertices->size();
    _maxHeight = maxHeight;

    computeNormals();
    //computeMaxDelta();
}

OceanTile::OceanTile( const OceanTile& tile, 
                      unsigned int resolution, 
                      const float spacing )
    :_resolution ( resolution )
    ,_rowLength  ( _resolution + 1 )
    ,_numVertices( _rowLength*_rowLength )
    ,_vertices   ( new osg::Vec3Array(_numVertices) )
    ,_normals    ( new osg::Vec3Array(_numVertices) )
    ,_spacing    ( spacing )
    ,_maxDelta   ( 0.f )
    ,_useVBO     ( tile.getUseVBO() )
{
    unsigned int parentRes = tile.getResolution();
    unsigned int inc = parentRes/_resolution;
    unsigned int inc2 = inc/2;

    // Take an average of four points
    for (unsigned int y = 0; y < parentRes; y+=inc) 
    {
        for (unsigned int x = 0; x < parentRes; x+=inc) 
        {
            const osg::Vec3f& a = tile.getVertex( x,      y      );
            const osg::Vec3f& b = tile.getVertex( x+inc2, y      );
            const osg::Vec3f& c = tile.getVertex( x,      y+inc2 );
            const osg::Vec3f& d = tile.getVertex( x+inc2, y+inc2 );

            osg::Vec3f sum = a + b + c + d;

            (*_vertices)[ array_pos(x/inc, y/inc, _rowLength) ] = sum * 0.25f;
        }
    }

    for( unsigned int i = 0; i < _rowLength-1; ++i )
    {
        // Copy top row into skirt
        (*_vertices)[ array_pos( i, _rowLength-1, _rowLength) ] = (*_vertices)[ i ];

        // Copy first column into skirt
        (*_vertices)[ array_pos( _rowLength-1, i, _rowLength) ] = (*_vertices)[ i*_rowLength ];        
    }

    // Copy corner value
    (*_vertices)[ array_pos( _rowLength-1, _rowLength-1, _rowLength ) ] = (*_vertices)[0];
    
    computeNormals();
}

OceanTile::OceanTile( const OceanTile& copy )
    :_vertices       ( copy._vertices )
    ,_normals        ( copy._normals )
    ,_resolution     ( copy._resolution )
    ,_rowLength      ( copy._rowLength )
    ,_numVertices    ( copy._numVertices )
    ,_spacing        ( copy._spacing )
    ,_maxDelta       ( copy._maxDelta )
    ,_averageHeight  ( copy._averageHeight )
    ,_maxHeight      ( copy._maxHeight )
    ,_useVBO         ( copy._useVBO )
{

}

OceanTile::~OceanTile( void )
{}

OceanTile& OceanTile::operator=(const OceanTile& rhs) 
{
    if (this != &rhs) 
    {
        _vertices      = rhs._vertices;
        _normals       = rhs._normals;
        _resolution    = rhs._resolution;
        _rowLength     = rhs._rowLength;
        _numVertices   = rhs._numVertices;
        _spacing       = rhs._spacing;
        _maxDelta      = rhs._maxDelta;
        _averageHeight = rhs._averageHeight;
        _maxHeight     = rhs._maxHeight;
        _useVBO        = rhs._useVBO;
    }
    return *this;
}

void OceanTile::computeNormals( void )
{
    int x1,x2,y1,y2;
    osg::Vec3f a,b,c,d,v1,v2,v3,n1,n2;

    const osg::Vec3f s2 = osg::Vec3f( 0.f,      -_spacing, 0.f );
    const osg::Vec3f s3 = osg::Vec3f( _spacing, 0.f,       0.f ); 
    const osg::Vec3f s4 = osg::Vec3f( _spacing, -_spacing, 0.f );

    // Compute normals for an N+2 x N+2 grid to ensure continuous
    // normals for tiles of the same resolution
    // Using first row as last row and first column as last column.

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array( (_rowLength+2)*(_rowLength+2) );

    for( int y = -1; y < (int)_rowLength; ++y )
    {    
        y1 = (y+_rowLength) % _rowLength;
        y2 = (y+1) % _rowLength;

        for( int x = -1; x < (int)_rowLength; ++x )
        {
            x1 = (x+_rowLength) % _rowLength;
            x2 = (x+1) % _rowLength;

            a = getVertex(x1, y1);        //      a|   /|c
            b = getVertex(x1, y2);        //       |  / |
            c = getVertex(x2, y1);        //       | /  |
            d = getVertex(x2, y2);        //      b|/   |d
            
            if (_useVBO) 
            {
                if ( x < 0 ){
                    a.x() = a.x() - _rowLength*_spacing;
                    b.x() = b.x() - _rowLength*_spacing;
                } 
                else if ( x+1 >= (int) _rowLength ) {
                    c.x() = c.x() + _rowLength*_spacing ;
                    d.x() = d.x() + _rowLength*_spacing ;
                }

                if ( y < 0 ){
                    a.y() = a.y() + _rowLength*_spacing;
                    c.y() = c.y() + _rowLength*_spacing;
                } 
                else if (y+1 >= (int) _rowLength) {
                    b.y() = b.y() - _rowLength*_spacing;
                    d.y() = d.y() - _rowLength*_spacing;
                }
            }
            else
            {
               a =      getVertex(x1, y1);        //      a|   /|c
               b = s2 + getVertex(x1, y2);        //       |  / |
               c = s3 + getVertex(x2, y1);        //       | /  |
               d = s4 + getVertex(x2, y2);        //      b|/   |d
            }

            v1 = b - a;
            v2 = b - c;
            v3 = b - d;

            n1 = v2 ^ v1;
            n2 = v3 ^ v2;

            (*normals)[ array_pos(x+1, y+1, _rowLength+2) ] += n1;        // a|  /c
            (*normals)[ array_pos(x+1, y+2, _rowLength+2) ] += n1;        //  | /
            (*normals)[ array_pos(x+2, y+1, _rowLength+2) ] += n1;        // b|/

            (*normals)[ array_pos(x+1, y+2, _rowLength+2) ] += n2;        //    /|c
            (*normals)[ array_pos(x+2, y+1, _rowLength+2) ] += n2;        //   / |
            (*normals)[ array_pos(x+2, y+2, _rowLength+2) ] += n2;        // b/__|d
        }
    }

    for( osg::Vec3Array::iterator itr = normals->begin(); 
          itr != normals->end(); 
          ++itr )
    {
        itr->normalize();
    }

    // copy normals into member normal array discarding first row and column;

    unsigned int ptr = 0;

    for(unsigned int y = 1; y <= _rowLength; ++y )
    {
        for(unsigned int x = 1; x <= _rowLength; ++x )
        {
            (*_normals)[ptr] = (*normals)[ array_pos(x,y,_rowLength+2) ];
            ++ptr;
        }
    }
}

void OceanTile::computeMaxDelta( void )
{
    float deltaMax = 0;

    int step = 2;
    int numLevels = 6;

    for (int level=1; level < numLevels; ++level)
    {
        for( unsigned int i=0; i < _resolution; ++i) 
        {
            int posY = i/step * step;
            
            for( unsigned int j=0; j < _resolution; ++j) 
            {
                if (i%step != 0 || j%step != 0) 
                {
                    int posX = j/step * step;

                    float delta = biLinearInterp(posX, posX+step, posY, posY+step, j, i);
                    delta -= getVertex(j, i).z();
                    delta = fabs(delta);
                    deltaMax = std::max(deltaMax, delta);
                }
            }
        }
        step *= 2;
    }
}

float OceanTile::biLinearInterp(int lx, int hx, int ly, int hy, int tx, int ty ) const
{
    float s00 = getVertex(lx, ly).z();
    float s01 = getVertex(hx, ly).z();
    float s10 = getVertex(lx, hy).z();
    float s11 = getVertex(hx, hy).z();

    int dx = hx - lx;
    int dtx = tx - lx;
    float v0 = (s01 - s00)/dx*dtx + s00;
    float v1 = (s11 - s10)/dx*dtx + s10;
    float value = (v1 - v0)/(hy - ly)*(ty - ly) + v0;

    return value;
}

float OceanTile::biLinearInterp(float x, float y ) const
{
    if (x >= 0.0 && y >= 0.0)
    {
        float dx = x/_spacing;
        float dy = y/_spacing;
        unsigned int ix = (unsigned int)dx;
        unsigned int iy = (unsigned int)dy;
        dx -= ix;
        dy -= iy;

        float s00 = getVertex(ix  , iy  ).z();
        float s01 = getVertex(ix+1, iy  ).z();
        float s10 = getVertex(ix  , iy+1).z();
        float s11 = getVertex(ix+1, iy+1).z();

        return s00*(1.f-dx)*(1.f-dy) + s01*dx*(1.f-dy) + s10*(1.f-dx)*dy + s11*dx*dy;
    }

    return 0.0;
}

osg::Vec3f OceanTile::normalBiLinearInterp(float x, float y ) const
{
    if (x >= 0.0 && y >= 0.0)
    {
        float dx = x / _spacing;
        float dy = y / _spacing;

        unsigned int ix = (unsigned int) dx;
        unsigned int iy = (unsigned int) dy;

        dx -= ix;
        dy -= iy;

        osg::Vec3f s00 = getNormal(ix,iy);
        osg::Vec3f s01 = getNormal(ix + 1,iy);
        osg::Vec3f s10 = getNormal(ix,iy + 1);
        osg::Vec3f s11 = getNormal(ix + 1,iy + 1);

        return s00*(1.f - dx)*(1.f-dy) + s01*dx*(1.f-dy) + s10*(1.f - dx)*dy + s11*dx*dy;
    }

    return osg::Vec3f(0, 0, 1);
}

osg::ref_ptr<osg::Texture2D> OceanTile::createNormalMap( void ) 
{
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;

    unsigned char* pixels = new unsigned char[_resolution*_resolution*3];    

    unsigned int idx = 0;
    unsigned int i = 0;

    for(unsigned int r = 0; r < _resolution; ++r )
    {
        for(unsigned int c = 0; c < _resolution; ++c )
        {
            idx = i*3;
            osg::Vec3f n = getNormal(c,r);

            pixels[idx]   = (unsigned char)(127.f * n.x() + 128.f);
            pixels[idx+1] = (unsigned char)(127.f * n.y() + 128.f);
            pixels[idx+2] = (unsigned char)(127.f * n.z() + 128.f);

            i++;
        }
    }

    osg::Image* img = new osg::Image;
    img->setImage(_resolution, _resolution, 1, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, pixels, osg::Image::USE_NEW_DELETE, 1);

#ifdef DEBUG_DATA
    // saves normal map as image
    static int count = 0;
    std::stringstream ss;
    ss << "Tile_" << count << ".bmp";
    osgDB::writeImageFile( *img, ss.str() );
    ++count;
#endif

    texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    texture->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::REPEAT );
    texture->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::REPEAT );
    texture->setImage ( img );

    return texture;
}
