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

#ifndef __OSGCOLLISION_CHART_H__
#define __OSGCOLLISION_CHART_H__ 1


#include <osgbCollision/Export.h>

#include <osg/Referenced>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Image>



namespace osgbCollision
{


/** \class Chart Chart.h <osgbCollision/Chart.h>
\brief Used by GLDebugDrawer to render a 2D HUD graph of intersection points.

*/
class OSGBCOLLISION_EXPORT Chart : public osg::Referenced
{
public:
    ///Constructor
    Chart();
    ///Destructor
    ~Chart();

    ///Set the value to be plotted
    ///\param idx The index into the data array for the value storage
    ///\param value The value to be plotted
    void setValue( int idx, float value );
    ///Set the background color
    ///\param bgColor The background color
    void setBackgroundColor( osg::Vec4& bgColor );
    ///Set the foreground color
    ///\param fgColor The foreground color
    void setForegroundColor( osg::Vec4& fgColor );
    ///Set the starting location and size values
    ///\param x The x starting location
    ///\param y The y starting location
    ///\param w The width of the chart
    ///\param h The hieght of the chart
    void setChartLocationAndSize( float x, float y, float w, float h );
    ///Get the chart
    ///\return The osg::Geode for the chart
    osg::Geode* get() const;
    ///Create the chart for the data display
    void createChart();

protected:
    float _x, _y, _w, _h;
    float _yScale;
    int _texW;

    float* _xValues;
    osg::ref_ptr< osg::Image > _image;

    osg::ref_ptr< osg::Geode > _geode;
    osg::ref_ptr< osg::Geometry > _geom;

    osg::ref_ptr< osg::Vec3Array > _verts;
    osg::ref_ptr< osg::Vec2Array > _tc;
    osg::Vec4 _bg, _fg, _overrun;

    osg::ref_ptr< osg::Uniform > _fgUniform, _bgUniform;
};


// osgbDynamics
}


// __OSGCOLLISION_CHART_H__
#endif
