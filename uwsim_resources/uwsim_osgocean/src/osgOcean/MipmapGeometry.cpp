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

#include <osgOcean/MipmapGeometry>

namespace osgOcean
{
    MipmapGeometry::MipmapGeometry( void ):
        _level      ( 0 ),
        _numLevels  ( 0 ),
        _resolution ( 0 ),
        _rowLen     ( 0 ),
        _colLen     ( 0 ),
        _startIdx   ( 0 ),
        _border     ( BORDER_NONE )
    {

    }

    MipmapGeometry::MipmapGeometry( unsigned int level, 
                                    unsigned int numLevels, 
                                    unsigned int startIdx,
                                    BORDER_TYPE border ):
        _level      ( level ),
        _numLevels  ( numLevels ),
        _resolution ( level != (numLevels-1) ? 2 << (numLevels-(level+2) ) : 1 ),
        _rowLen     ( border==BORDER_X || border==BORDER_XY ? _resolution+1 : _resolution),
        _colLen     ( border==BORDER_Y || border==BORDER_XY ? _resolution+1 : _resolution),
        _startIdx   ( startIdx ),
        _border     ( border )
    {
    }

    MipmapGeometry::~MipmapGeometry( void )
    {

    }

    MipmapGeometry::MipmapGeometry( const MipmapGeometry& copy, const osg::CopyOp& copyop ):
        osg::Geometry ( copy, copyop ),
        _level        ( copy._level ),
        _numLevels    ( copy._numLevels ),
        _resolution   ( copy._resolution ),
        _rowLen       ( copy._rowLen ),
        _colLen       ( copy._colLen ),
        _startIdx     ( copy._startIdx ),
        _border       ( copy._border )
    {
        
    }
}
