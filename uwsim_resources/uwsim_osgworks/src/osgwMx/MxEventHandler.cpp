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

#include <osgwMx/MxEventHandler.h>
#include <osgwMx/MxCore.h>
#include <osgwMx/MxUtils.h>

#include <osg/Notify>


namespace osgwMx
{


MxEventHandler::MxEventHandler()
  : _mxCore( new MxCore ),
    _scene( NULL ),
    _cameraUpdateCallback( NULL ),
    _worldUp( osg::Vec3d( 0., 0., 1. ) ),
    _lastX( 0.0 ),
    _lastY( 0.0 ),
    _lastXPixel( 0.f ),
    _lastYPixel( 0.f ),
    _leftDragging( false ),
    _leftClick( false ),
    _moveScale( 10.f )
{
}
MxEventHandler::MxEventHandler( const MxEventHandler& rhs, const osg::CopyOp& copyop )
  : osgGA::GUIEventHandler( rhs, copyop ),
    _mxCore( rhs._mxCore ),
    _scene( rhs._scene ),
    _cameraUpdateCallback( rhs._cameraUpdateCallback ),
    _worldUp( rhs._worldUp ),
    _lastX( rhs._lastX ),
    _lastY( rhs._lastY ),
    _lastXPixel( rhs._lastXPixel ),
    _lastYPixel( rhs._lastYPixel ),
    _leftDragging( rhs._leftDragging ),
    _leftClick( rhs._leftClick ),
    _moveScale( rhs._moveScale )
{
}
MxEventHandler::~MxEventHandler()
{
}

bool MxEventHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
    const bool ctrlKey( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL ) != 0 );
    const bool shiftKey( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT ) != 0 );

    bool handled( false );
    switch( ea.getEventType() )
    {
    case osgGA::GUIEventAdapter::PUSH:
    {
        _lastX = ea.getXnormalized();
        _lastY = ea.getYnormalized();
        _lastXPixel = ea.getX();
        _lastYPixel = ea.getY();

        _leftClick = ( ( ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) != 0 );
        if( _leftClick && _scene.valid() )
            _panPlane = osgwMx::computePanPlane( _scene.get(), _mxCore.get(), _lastX, _lastY );

        handled = true;
        break;
    }
    case osgGA::GUIEventAdapter::DRAG:
    {
        const double deltaX = ea.getXnormalized() - _lastX;
        const double deltaY = ea.getYnormalized() - _lastY;

        if( ( ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) != 0 )
        {
            // Allow for some sloppiness in the DRAG event in order to properly detect a click.
            _leftDragging = ( _leftDragging ||
                // or manhatten distance > 5:
                ( ( osg::absolute< float >( ea.getX() - _lastXPixel ) +
                    osg::absolute< float >( ea.getY() - _lastYPixel ) ) > 5 ) );

            if( shiftKey && _scene.valid() )
            {
                // Left mouse, shifted: pan
                osg::Vec3d panDelta = osgwMx::pan( _scene.get(), _mxCore.get(), _panPlane, deltaX, deltaY );
                _mxCore->moveLiteral( -panDelta );
                handled = true;
            }
            else
            {
                // Not shifted.
                if( ctrlKey )
                {
                    // Left mouse, ctrl but no shift: turn head.
                    _mxCore->rotateLocal( deltaX, _mxCore->getUp() );
                    _mxCore->rotateLocal( -deltaY, _mxCore->getCross() );
                    handled = true;
                }
                else
                {
                    // Left mouse, no shift and no ctrl: orbit
                    double angle;
                    osg::Vec3d axis;
                    osgwMx::computeTrackball( angle, axis,
                        osg::Vec2d( _lastX, _lastY ), osg::Vec2d( deltaX, deltaY ),
                        _mxCore->getOrientationMatrix() );

                    _mxCore->rotateOrbit( angle, axis );

                    handled = true;
                }
            }
        }
        else if( ( ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON ) != 0 )
        {
            if( shiftKey )
                // Shift right mouse: move left-right and up-down.
                _mxCore->moveLocal( osg::Vec3d( deltaX, deltaY, 0. ) * _moveScale );
            else
                // Right mouse, no shift: dolly forward-backward
                _mxCore->moveLocal( osg::Vec3d( 0., 0., deltaY ) * _moveScale );
            handled = true;
        }
        _lastX = ea.getXnormalized();
        _lastY = ea.getYnormalized();
        break;
    }
    case osgGA::GUIEventAdapter::RELEASE:
    {
        if( _leftClick )
        {
            if( !_leftDragging && ctrlKey && shiftKey && _scene.valid() )
            {
                // Parameters are NDC coordinates in range -1.0,1.0.
                osg::Vec3d orbitCenter = pickPoint( _scene.get(), _mxCore.get(),
                    ea.getXnormalized(), ea.getYnormalized() );
                _mxCore->setOrbitCenterPoint( orbitCenter );
            }
            _leftDragging = false;
            _leftClick = false;
        }
        break;
    }
    case osgGA::GUIEventAdapter::SCROLL:
    {
        // Scroll wheel: FOV
        if( ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP )
        {
            _mxCore->fovyScaleUp();
            handled = true;
        }
        else if( ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_DOWN )
        {
            _mxCore->fovyScaleDown();
            handled = true;
        }
        break;
    }
    case osgGA::GUIEventAdapter::KEYDOWN:
    {
        switch (ea.getKey())
        {
        case 'o': // toggle orthographic
        {
            const double viewDistance = ( _mxCore->getOrbitCenterPoint() - _mxCore->getPosition() ).length();
            _mxCore->setOrtho( !( _mxCore->getOrtho() ), viewDistance );
            handled = true;
            break;
        }
        case 'd': // Display current yaw / pitch / roll values
        {
            double y, p, r;
            _mxCore->getYawPitchRoll( y, p, r );
            osg::notify( osg::ALWAYS ) << "Yaw: " << y << "   Pitch: " << p << "   Roll: " << r << std::endl;
            break;
        }
        case ' ': // Reset to initial values.
        {
            _mxCore->reset();
            handled = true;
            break;
        }
        }
        break;
    }
    default:
    {
        break;
    }
    }

    if( handled )
        us.requestRedraw();

    return( handled );
}


void MxEventHandler::setMxCore( osgwMx::MxCore* mxCore )
{
    _mxCore = mxCore;
}

MxCore* MxEventHandler::getMxCore()
{
    return( _mxCore.get() );
}

osg::NodeCallback* MxEventHandler::getGenericMatrixCallback()
{
    return( getMatrixCallback() );
}
CameraUpdateCallback* MxEventHandler::getMatrixCallback()
{
    if( !( _cameraUpdateCallback.valid() ) )
        _cameraUpdateCallback = new CameraUpdateCallback( _mxCore.get() );
    return( _cameraUpdateCallback.get() );
}



// osgwMx
}
