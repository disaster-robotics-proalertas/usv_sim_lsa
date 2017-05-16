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

#include "osgOcean/MipmapGeometryVBO"
#include <stdlib.h>

namespace osgOcean
{
    MipmapGeometryVBO::MipmapGeometryVBO( void )
        :_numLevels      ( 0 )
        ,_level          ( -1 )
        ,_levelRight     ( -1 )
        ,_levelBelow     ( -1 )
        ,_rowLen         ( 0 )
        ,_maxResolution  ( 0 )
        ,_resolution     ( 0 )
        ,_resRight       ( 0 )
        ,_resBelow       ( 0 )
        ,_worldSize      ( 0.f )
    {
        setDataVariance ( osg::Object::DYNAMIC );
        setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
        setColorBinding ( osg::Geometry::BIND_OVERALL );
        setUseDisplayList( false );
        setUseVertexBufferObjects( true );
    }

    MipmapGeometryVBO::MipmapGeometryVBO( unsigned int numLevels, float worldSize )
        :_numLevels      ( numLevels )
        ,_level          ( -1 )
        ,_levelRight     ( -1 )
        ,_levelBelow     ( -1 )
        ,_maxResolution  ( calcResolution(0,numLevels) )
        ,_rowLen         ( _maxResolution+1 )
        ,_resolution     ( 0 )
        ,_resRight       ( 0 )
        ,_resBelow       ( 0 )
        ,_worldSize      ( worldSize )
    {
        setDataVariance ( osg::Object::DYNAMIC );
        setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
        setColorBinding ( osg::Geometry::BIND_OVERALL );
        setUseDisplayList( false );
        setUseVertexBufferObjects( true );
    }

    MipmapGeometryVBO::~MipmapGeometryVBO(  )
    {

    }

    MipmapGeometryVBO::MipmapGeometryVBO( const MipmapGeometryVBO& copy, const osg::CopyOp& copyop )
        :osg::Geometry ( copy, copyop )
        ,_numLevels    ( copy._numLevels )
        ,_level        ( copy._level )
        ,_levelRight   ( copy._levelRight )
        ,_levelBelow   ( copy._levelBelow )
        ,_rowLen       ( copy._rowLen )
        ,_maxResolution( copy._maxResolution )
        ,_resolution   ( copy._resolution )
        ,_resRight     ( copy._resRight )
        ,_resBelow     ( copy._resBelow )
        ,_worldSize    ( copy._worldSize )
        ,_offset       ( copy._offset )
        ,_mainBody     ( copy._mainBody )
        ,_rightBorder  ( copy._rightBorder )
        ,_belowBorder  ( copy._belowBorder )
        ,_cornerPiece  ( copy._cornerPiece )
    {
    }

    osg::BoundingBox MipmapGeometryVBO::computeBound( void ) const 
    {
        osg::BoundingBox bb;
        
        bb.xMin() = _offset.x();
        bb.xMax() = _offset.x()+_worldSize;
        bb.yMin() = _offset.y()-_worldSize;
        bb.yMax() = _offset.y();
        bb.zMin() = -5.f;
        bb.zMax() = 5.f;

        return bb;
    }

    bool MipmapGeometryVBO::updatePrimitives( unsigned int level, unsigned int levelRight, unsigned int levelBelow )
    {
        if( checkPrimitives(level,levelRight,levelBelow) )
        {
            assignPrimitives();
            return true;
        }

        return false;
    }

    bool MipmapGeometryVBO::checkPrimitives( unsigned int level, unsigned int levelRight, unsigned int levelBelow )
    {
#ifndef NDEBUG 
        if( abs( (int)level-(int)levelRight) > 1 || 
            abs( (int)level-(int)levelBelow) > 1 )
        {   
            osg::notify(osg::WARN) << "osgOcean::MipmapGeometryVBO() - Mipmap level difference is greater than 1" << std::endl;
        }
#endif 

        bool updateMain  = _level      == (int)level      ? false : true;
        bool updateRight = _levelRight == (int)levelRight ? false : true;
        bool updateBelow = _levelBelow == (int)levelBelow ? false : true;

        // if there's no change return immediately
        if( !updateMain && !updateRight && !updateBelow )
            return false;

        _level      = level;
        _levelRight = levelRight;
        _levelBelow = levelBelow;

        _resolution = calcResolution(_level,      _numLevels);
        _resRight   = calcResolution(_levelRight, _numLevels);
        _resBelow   = calcResolution(_levelBelow, _numLevels);

        // if the resolution is 1 and there is an update in any of the
        // border or body pieces then we must recompute the zero tile 
        // due to the tessellation.
        if( _resolution == 1 )
        {
            addZeroTile();
            return true;
        }
        // if the level of the main body is different, then all primitives
        // need to be updated.
        if( updateMain )
        {
            addMainBody();

            if(_resRight == 1 || _resBelow == 1){
                addZeroCornerPiece();
            }
            else{
                addRightBorder();
                addBottomBorder();
                addCornerPiece();
            }
            return true;
        }
        
        // if we don't do a main body update then we must require
        // a border/corner update.
        if(_resRight == 1 || _resBelow == 1){
            addZeroCornerPiece();
            return true;
        }
        else{
            // requires the additional check here because adding a zero corner piece
            // leaves the border pieces empty 
            if( updateRight || _rightBorder.size() == 0)
                addRightBorder();
            if( updateBelow || _belowBorder.size() == 0)
                addBottomBorder();

            addCornerPiece(); 

            return true;
        }

        assert(0);
        return false;
    }

//#define NO_DEGENERATE_TRIANGLES
    void MipmapGeometryVBO::addMainBody( void )
    {
        _rowLen = _resolution + 1;

        unsigned inc = _maxResolution / _resolution;
        unsigned rowLimit = (_maxResolution+1)-(inc*2);
        unsigned colLimit = (_maxResolution+1)-(inc);

        _mainBody.clear();

        // Degenerate triangles seem to cause problems on some cards so leave the original 
        // version in here. The degenerate version does appear to provide a noticeable
        // difference in draw time.
#ifdef NO_DEGENERATE_TRIANGLES
        
        _mainBody.resize(_resolution-1);

        unsigned indices =_resolution*2;

        int p = 0;
        for( unsigned r = 0; r < _resolution-1; ++r )
        {
            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, indices );
            int i = 0;
            unsigned row = r*inc;

            for( unsigned c = 0; c < _resolution; ++c )
            {
                unsigned col = c*inc;
                (*primitive)[i++] = getIndex( col, row );
                (*primitive)[i++] = getIndex( col, row+inc );
            }

            _mainBody[p++]=primitive;
        }
#else
        unsigned indices = (_resolution*2)*(_resolution) - 4;
        osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, indices );

        int i = 0;
        for( unsigned r = 0; r < _resolution-1; ++r )
        {
            unsigned row = r*inc;

            for( unsigned c = 0; c < _resolution; ++c )
            {
                unsigned col = c*inc;
                (*primitive)[i++] = getIndex( col, row );
                (*primitive)[i++] = getIndex( col, row+inc );

                if( (c == _resolution-1) && (r+1 != _resolution-1) )
                {
                    (*primitive)[i++] = getIndex( col, row+inc );
                    (*primitive)[i++] = getIndex( 0,   row+inc );
                }
            }
        }

        _mainBody.push_back( primitive );
#endif
    }

    void MipmapGeometryVBO::addZeroTile( void )
    {
        _mainBody.clear();
        _belowBorder.clear();
        _rightBorder.clear();
        _cornerPiece.clear();

        unsigned incBelow = _maxResolution / _resBelow;

        if( _resRight == 1 && _resBelow == 1 )
        {
            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, 4 );

            (*primitive)[0] = getIndex( 0,              0              );
            (*primitive)[1] = getIndex( 0,              _maxResolution );
            (*primitive)[2] = getIndex( _maxResolution, 0              );
            (*primitive)[3] = getIndex( _maxResolution, _maxResolution );

            _mainBody.push_back( primitive );
            return;
        }
        else
        {
            if( _resRight == 1 )
            {
                osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 5 );

                (*primitive)[0] = getIndex( _maxResolution,  0              );
                (*primitive)[1] = getIndex( 0,               0              );
                (*primitive)[2] = getIndex( 0,               _maxResolution );
                (*primitive)[3] = getIndex( incBelow,        _maxResolution );
                (*primitive)[4] = getIndex( _maxResolution,  _maxResolution );

                _mainBody.push_back( primitive );
            }
            else
            {
                osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 0 );
                primitive->reserve(6);

                primitive->push_back( getIndex( 0, 0 ) );
                primitive->push_back( getIndex( 0, _maxResolution ) );

                if( _resBelow > _resolution )
                    primitive->push_back( getIndex( _maxResolution/2, _maxResolution) );

                primitive->push_back( getIndex( _maxResolution, _maxResolution ) );

                if( _resRight > _resolution )
                    primitive->push_back( getIndex( _maxResolution, _maxResolution/2 ) );

                primitive->push_back( getIndex( _maxResolution, 0 ) );

                _mainBody.push_back( primitive );
            }
        }
    }

    void MipmapGeometryVBO::addZeroCornerPiece(void)
    {
        _belowBorder.clear();
        _rightBorder.clear();
        _cornerPiece.clear();

        unsigned inc =      _maxResolution / _resolution;
        unsigned incRight = _maxResolution / _resRight;
        unsigned incBelow = _maxResolution / _resBelow;

        osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);

        primitive->push_back( getIndex( inc, inc ) );
        primitive->push_back( getIndex( 0,   inc ) );

        for(unsigned c = 0; c <= _maxResolution; c+=incBelow )
            primitive->push_back( getIndex( c, _maxResolution ) );

        for(int r = _maxResolution-incRight; r >= 0; r-=incRight )
            primitive->push_back( getIndex( _maxResolution, r ) );

        primitive->push_back( getIndex( inc, 0 ) );

        _cornerPiece.push_back( primitive );
    }

    void MipmapGeometryVBO::addBottomBorder(void)
    {
        _belowBorder.clear();

        unsigned inc      = _maxResolution / _resolution;
        unsigned incBelow = _maxResolution / _resBelow;
        
         // same res to the right
        if(_level == _levelBelow )
        {
            unsigned indicies = (_rowLen-1)*2;
            //unsigned indicies = _maxResolution/inc*2;
            
            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, indicies);
            
            int i = 0;
            for(unsigned c = 0; c < _maxResolution; c+=inc)
            {
                (*primitive)[i++] = getIndex( c, _maxResolution-inc );
                (*primitive)[i++] = getIndex( c, _maxResolution     );
            }

            _belowBorder.push_back( primitive );
        }
        // lower res to the right
        else if(_level < _levelBelow )
        { 
            unsigned colLimit = _maxResolution-incBelow;

            for( unsigned int c = 0; c < colLimit; c+=incBelow )
            {
                osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 5 );   

                (*primitive)[0] = getIndex(c,           _maxResolution     );
                (*primitive)[1] = getIndex(c+incBelow,  _maxResolution     );
                (*primitive)[2] = getIndex(c+incBelow,  _maxResolution-inc );
                (*primitive)[3] = getIndex(c+inc,       _maxResolution-inc );
                (*primitive)[4] = getIndex(c,           _maxResolution-inc );

                _belowBorder.push_back(primitive);
            }
        }
        // higher res to the right
        else if(_level > _levelBelow)
        {
            unsigned colLimit = _maxResolution-inc;

            for( unsigned int c = 0; c < colLimit; c+=inc )
            {
                osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 5 );   

                (*primitive)[0] = getIndex(c+inc,       _maxResolution-inc );
                (*primitive)[1] = getIndex(c,           _maxResolution-inc );
                (*primitive)[2] = getIndex(c,           _maxResolution     );
                (*primitive)[3] = getIndex(c+incBelow,  _maxResolution     );
                (*primitive)[4] = getIndex(c+inc,       _maxResolution     );

                _belowBorder.push_back(primitive);
            }
        }
        else
        {
            osg::notify(osg::NOTICE) << "BELOW BORDER ERROR" << std::endl;
        }
    }

    void MipmapGeometryVBO::addRightBorder( void )
    {
        _rightBorder.clear();

        unsigned inc =      _maxResolution / _resolution;
        unsigned incRight = _maxResolution / _resRight;

        // same res to the right
        if(_level == _levelRight)
        {
            unsigned rowLimit = _maxResolution-inc;
            unsigned indicies = _maxResolution/inc*2;

            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, indicies);

            int i = 0;
            for(int r = rowLimit; r >= 0; r-=inc)
            {
                (*primitive)[i++] = getIndex( _maxResolution-inc, r );
                (*primitive)[i++] = getIndex( _maxResolution,     r );
            }

            _rightBorder.push_back( primitive );
        }
        // lower res to the right
        else if(_level < _levelRight )
        {   
            unsigned rowLimit = _maxResolution-incRight;
            
            for( unsigned r = 0; r < rowLimit; r+=incRight )
            {
                osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 5 );   

                (*primitive)[0] = getIndex(_maxResolution,      r          );
                (*primitive)[1] = getIndex(_maxResolution-inc,  r          );
                (*primitive)[2] = getIndex(_maxResolution-inc,  r+inc      );
                (*primitive)[3] = getIndex(_maxResolution-inc,  r+incRight );
                (*primitive)[4] = getIndex(_maxResolution,      r+incRight );

                _rightBorder.push_back(primitive);
            }
        }
        // higher res to the right
        else if( _level > _levelRight )
        {
            unsigned rowLimit = _maxResolution-inc;

            for(unsigned r = 0; r < rowLimit; r+=inc )
            {
                osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 5 );

                (*primitive)[0] = getIndex(_maxResolution-inc, r+inc      );
                (*primitive)[1] = getIndex(_maxResolution,     r+inc      );
                (*primitive)[2] = getIndex(_maxResolution,     r+incRight );
                (*primitive)[3] = getIndex(_maxResolution,     r          );
                (*primitive)[4] = getIndex(_maxResolution-inc, r          );
                
                _rightBorder.push_back( primitive );
            }
        }
    }

    void MipmapGeometryVBO::addCornerPiece(void)
    {
        _cornerPiece.clear();

        unsigned inc =      _maxResolution / _resolution;
        unsigned incRight = _maxResolution / _resRight;
        unsigned incBelow = _maxResolution / _resBelow;

        if( _level == _levelBelow && _level == _levelRight )
        {
            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 4);

            (*primitive)[0] = getIndex( _maxResolution-inc, _maxResolution-inc  );
            (*primitive)[1] = getIndex( _maxResolution-inc, _maxResolution      );
            (*primitive)[2] = getIndex( _maxResolution,     _maxResolution-inc  );
            (*primitive)[3] = getIndex( _maxResolution,     _maxResolution      );

            _cornerPiece.push_back( primitive );
        }
        else if( _levelBelow >= _level && _levelRight <= _level )
        {
            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);
            
            primitive->push_back( getIndex( _maxResolution - incBelow, _maxResolution ) );
            primitive->push_back( getIndex( _maxResolution,            _maxResolution ) );
            
            for(int r = _maxResolution; r >= int(_maxResolution-inc); r-=incRight )
                primitive->push_back( getIndex( _maxResolution, r ) );

            for(int c = _maxResolution; c >= int(_maxResolution-incBelow); c-=inc )
                primitive->push_back( getIndex( c, _maxResolution-inc ) );
            
            _cornerPiece.push_back(primitive);
        }
        else if(_levelBelow < _level && _levelRight > _level)
        {
            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);

            primitive->push_back( getIndex( _maxResolution,             _maxResolution-incRight    ));
            primitive->push_back( getIndex( _maxResolution - inc,       _maxResolution-incRight    ));
            primitive->push_back( getIndex( _maxResolution - inc,       _maxResolution-inc         ));
            primitive->push_back( getIndex( _maxResolution - inc,       _maxResolution             ));
            primitive->push_back( getIndex( _maxResolution - incBelow,  _maxResolution             ));
            primitive->push_back( getIndex( _maxResolution,             _maxResolution             ));

            _cornerPiece.push_back(primitive);
        }
        else if( _levelBelow <= _level && _levelRight <= _level )
        {
            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);

            primitive->push_back( getIndex( _maxResolution - inc, _maxResolution - inc ) );

            for(unsigned c = _maxResolution-inc; c < _maxResolution; c+=incBelow)
                primitive->push_back( getIndex( c, _maxResolution ) );

            for(int r = _maxResolution; r >= int(_maxResolution-inc); r-=incRight )
                primitive->push_back( getIndex( _maxResolution, r ) );

            _cornerPiece.push_back(primitive);
        }
        else if( _levelBelow >= _level && _levelRight >= _level )
        {
            osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);
            primitive->reserve(7);

            primitive->push_back( getIndex( _maxResolution,          _maxResolution ) );
            primitive->push_back( getIndex(_maxResolution,      _maxResolution-incRight ));
            primitive->push_back( getIndex(_maxResolution-inc,  _maxResolution-incRight ));
            primitive->push_back( getIndex(_maxResolution-inc,  _maxResolution-inc      ));
            if(incBelow > inc)
                primitive->push_back( getIndex(_maxResolution-incBelow, _maxResolution-inc ));

            primitive->push_back( getIndex( _maxResolution-incBelow, _maxResolution ) );

            _cornerPiece.push_back(primitive);
        }
    }

    void MipmapGeometryVBO::assignPrimitives( void )
    {
        unsigned newListSize = _mainBody.size() + _rightBorder.size() + _belowBorder.size() + _cornerPiece.size();

        _primitives.clear();
        _primitives.reserve( newListSize );

        _primitives.insert( _primitives.end(), _mainBody.begin(),    _mainBody.end() );
        _primitives.insert( _primitives.end(), _rightBorder.begin(), _rightBorder.end() );
        _primitives.insert( _primitives.end(), _belowBorder.begin(), _belowBorder.end() );
        _primitives.insert( _primitives.end(), _cornerPiece.begin(), _cornerPiece.end() );
    }
}

