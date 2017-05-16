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

#ifndef __OSGWCONTROLS_SLIDER_CONTROL_H__
#define __OSGWCONTROLS_SLIDER_CONTROL_H__ 1

#include "osgwControls/Export.h"

#include <osgGA/GUIEventHandler>
#include <osg/MatrixTransform>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Timer>
#include <osgText/Text>
#include <osg/BoundingBox>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>


namespace osgwControls {


/** \brief A 3D slider device.
*/
class OSGWCONTROLS_EXPORT SliderControl
{
public:
    SliderControl();

    void setCurrentValue( double v );
    void setCurrentPos( double x );
    double getCurrentValue() const;

    osg::Node* getSliderControlSubgraph();

    void setDisplayArea( float len );
    void setValueRange( double valueMin, double valueMax );
    void setTimeRange( double time );

    enum PlayMode {
        STOP,
        FORWARD,
        REVERSE
    };
    void setPlayMode( PlayMode pm );
    PlayMode getPlayMode() const;

    void setLoopAnimation( bool loop );
    bool getLoopAnimation() const;
    void update(double reftime);

protected:
    virtual ~SliderControl();
    float _w,_h;
    double _minVal, _maxVal, _time;
    double _simTime;
    osg::ref_ptr< osg::Group > _root;

    double _currentValue;
    PlayMode _playMode;
    bool _loop;
    
    void refreshButtons();
    osg::Matrix _matrix;
    osg::ref_ptr< osg::MatrixTransform > _mt;
    osg::ref_ptr<osg::Geometry> _forward,_reverse,_repeat;
};


// namespace osgwControls
}

// __OSGWCONTROLS_SLIDER_CONTROL_H__
#endif
