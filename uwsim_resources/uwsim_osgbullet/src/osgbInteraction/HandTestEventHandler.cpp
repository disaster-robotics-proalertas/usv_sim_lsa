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

#include <osgbInteraction/HandTestEventHandler.h>
#include <osgbInteraction/HandNode.h>
#include <osgwTools/Quat.h>


namespace osgbInteraction
{


HandTestEventHandler::HandTestEventHandler( osgbInteraction::HandNode* hn )
  : _hand( hn ),
    _mode( osgbInteraction::HandNode::FINGER_0_TRANSLATE ),
    _h( 0.f ),
    _p( 0.f ),
    _r( 0.f )
{
    osg::Quat q = _hand->getAttitude();
}

bool
HandTestEventHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
{
    const unsigned int mod = ea.getModKeyMask();
    const bool ctrl = ( (mod&osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) ||
        (mod&osgGA::GUIEventAdapter::MODKEY_RIGHT_CTRL) );
    const bool shift = ( (mod&osgGA::GUIEventAdapter::MODKEY_LEFT_SHIFT) ||
        (mod&osgGA::GUIEventAdapter::MODKEY_RIGHT_SHIFT) );

    const unsigned int buttonMask( ea.getButtonMask() );
    const bool ourLeft( (ctrl || shift) && (buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) );
    const bool ourRight( (ctrl || shift) && (buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) );

    switch( ea.getEventType() )
    {
        case osgGA::GUIEventAdapter::KEYUP:
        {
            if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Home)
            {
                _hand->setPose( osgbInteraction::HandNode::POSE_DEFAULT );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_End)
            {
                _hand->setPose( osgbInteraction::HandNode::POSE_HOOK );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Page_Up)
            {
                _hand->setPose( osgbInteraction::HandNode::POSE_POINT );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Page_Down)
            {
                _hand->setPose( osgbInteraction::HandNode::POSE_FIST );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Delete)
            {
                _hand->dump();
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F1)
            {
                _mode = osgbInteraction::HandNode::FINGER_0_TRANSLATE;
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F2)
            {
                _mode = osgbInteraction::HandNode::FINGER_1_TRANSLATE;
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F3)
            {
                _mode = osgbInteraction::HandNode::FINGER_2_TRANSLATE;
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F4)
            {
                _mode = osgbInteraction::HandNode::FINGER_3_TRANSLATE;
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F5)
            {
                _mode = osgbInteraction::HandNode::FINGER_4_TRANSLATE;
                return true;
            }

            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Left)
            {
                _hand->setArticulation( _mode,
                    _hand->getArticulation( _mode ) + 0.1 );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Right)
            {
                _hand->setArticulation( _mode,
                    _hand->getArticulation( _mode ) - 0.1 );
                return true;
            }
            return false;
        }

        case osgGA::GUIEventAdapter::SCROLL:
        {
            const unsigned int mod = ea.getModKeyMask();
            const bool k1 = ( (mod&osgGA::GUIEventAdapter::MODKEY_LEFT_SHIFT) ||
                (mod&osgGA::GUIEventAdapter::MODKEY_RIGHT_CTRL) );
            const bool k0 = ( !k1 || ( (mod&osgGA::GUIEventAdapter::MODKEY_LEFT_ALT) ||
                (mod&osgGA::GUIEventAdapter::MODKEY_RIGHT_ALT) ) );

            float delta( 0.05 );
            osgGA::GUIEventAdapter::ScrollingMotion sm = ea.getScrollingMotion();
            if (sm == osgGA::GUIEventAdapter::SCROLL_UP)
                delta = -delta;

            if( _mode == osgbInteraction::HandNode::FINGER_0_TRANSLATE )
            {
                if (k0) _hand->setArticulation( _mode + 10 , _hand->getArticulation( _mode+10  ) + delta );
                if (k1) _hand->setArticulation( _mode + 15, _hand->getArticulation( _mode+15 ) + delta );
            }
            else
            {
                if (k0) _hand->setArticulation( _mode + 5 , _hand->getArticulation( _mode+5  ) + delta );
                if (k1) _hand->setArticulation( _mode + 10, _hand->getArticulation( _mode+10 ) + delta );
            }
            return true;
        }
        case osgGA::GUIEventAdapter::PUSH:
        {
            if( !ourLeft && !ourRight )
                return false;

            _lastX = ea.getXnormalized();
            _lastY = ea.getYnormalized();
            return true;
        }
        case osgGA::GUIEventAdapter::DRAG:
        {
            // Right mouse drag + ctrl: xy motion
            // Right mouse drag + shift: z motion
            // Left mouse drag + ctrl: heading and pitch
            // Left mouse drag + shift: roll

            if( ourRight )
            {
                osg::Vec3 move;
                if( ctrl )
                {
                    move[ 0 ] = _lastX - ea.getXnormalized();
                    move[ 1 ] = _lastY - ea.getYnormalized();
                }
                else if( shift )
                    move[ 2 ] = ea.getYnormalized() - _lastY;
                _lastX = ea.getXnormalized();
                _lastY = ea.getYnormalized();

                osg::Quat q = _hand->getAttitude();
                osg::Vec3 tmove = q * move * 5.f;
                _hand->setPosition( tmove + _hand->getPosition() );
                return true;
            }

            if( !ourLeft )
                return false;

            if( ctrl )
            {
                // X = Heading
                // Y = Pitch
                _h += ( _lastX - ea.getXnormalized() ) * 2.;
                _p += ( _lastY - ea.getYnormalized() ) * 2.;
            }
            else if( shift )
            {
                // X = Roll
                _r += ( _lastX - ea.getXnormalized() ) * 2.;
            }
            _lastX = ea.getXnormalized();
            _lastY = ea.getYnormalized();

            osg::Quat q = osgwTools::makeHPRQuat(
                osg::RadiansToDegrees( _h ),
                osg::RadiansToDegrees( _p ),
                osg::RadiansToDegrees( _r ) );
            _hand->setAttitude( q );
            return true;
        }
        default:
        break;
    }
    return false;
}





VirtualHandTestEventHandler::VirtualHandTestEventHandler( osgbInteraction::HandNode* hn )
  : _hand( hn ),
    _finger( &_params._finger0 ),
    _h( 0.f ),
    _p( 0.f ),
    _r( 0.f )
{
}

//             ctrl  shift  scroll
//   Sprd01     X      X      X    // select with F1
//   Sprd12     X      X      X    // select with F2
//   Sprd23     X      X      X    // select with F3
//   Sprd34     X      X      X    // select with F4
//   Sprd all   X      X      X    // select with F5
//   Thumbk1                  X    // select with F1
//   Thumbk2           X      X
//   f1k0                     X    // select with F2
//   f1k12             X      X
//   f2k0                     X    // select with F3
//   f2k12             X      X
//   f3k0                     X    // select with F4
//   f3k12             X      X
//   f4k0                     X    // select with F5
//   f4k12             X      X

// Left/Right arror -- Thumb rotate at knuckle 0, not accessible by data glove.

// Home    Default post
// End     Hook pose
// PgUp    Point pose
// PgDn    Fist pose

// Delete  Dump info to console, data files to current directory
// '='     Toggle calibrate mode.
// H/h     Toggle right/left.
// V/v     Toggle hand visibility on/off.

// Movement:
//             ctrl  shift  drag  mouse
//  xy pos      X            X    right
//  z pos              X     X    right
//  h/p att     X            X    left
//  r att              X     X    left

bool
VirtualHandTestEventHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
{
    const unsigned int mod = ea.getModKeyMask();
    const bool ctrl = ( (mod&osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) ||
        (mod&osgGA::GUIEventAdapter::MODKEY_RIGHT_CTRL) );
    const bool shift = ( (mod&osgGA::GUIEventAdapter::MODKEY_LEFT_SHIFT) ||
        (mod&osgGA::GUIEventAdapter::MODKEY_RIGHT_SHIFT) );

    const unsigned int buttonMask( ea.getButtonMask() );
    const bool ourLeft( (ctrl || shift) && (buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) );
    const bool ourRight( (ctrl || shift) && (buttonMask == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) );

    bool setParams = false;
    _hand->getAll( _params );

    switch( ea.getEventType() )
    {
        case osgGA::GUIEventAdapter::SCROLL:
        {
            float delta( 0.05 );
            osgGA::GUIEventAdapter::ScrollingMotion sm = ea.getScrollingMotion();
            if (sm == osgGA::GUIEventAdapter::SCROLL_DOWN)
                delta = -delta;

            if( ctrl && shift )
            {
                // Articulate the spread angles.
                if( _finger == &_params._finger0 )
                    _params._spread01 += delta;
                else if( _finger == &_params._finger1 )
                    _params._spread12 += delta;
                else if( _finger == &_params._finger2 )
                    _params._spread23 += delta;
                else if( _finger == &_params._finger3 )
                    _params._spread34 += delta;
                else
                {
                    _params._spread01 += delta;
                    _params._spread12 += delta;
                    _params._spread23 += delta;
                    _params._spread34 += delta;
                }
                setParams = true;
            }
            else if( shift )
            {
                (*_finger)[ 1 ] += delta;
                setParams = true;
            }
            else
            {
                (*_finger)[ 0 ] += delta;
                setParams = true;
            }
            break;
        }
        case osgGA::GUIEventAdapter::KEYUP:
        {
            if( ea.getKey()==osgGA::GUIEventAdapter::KEY_Left )
            {
                _hand->setArticulation( osgbInteraction::HandNode::FINGER_0_ROTATE_INNER,
                    _hand->getArticulation( osgbInteraction::HandNode::FINGER_0_ROTATE_INNER ) + 0.05 );
                return true;
            }
            else if( ea.getKey()==osgGA::GUIEventAdapter::KEY_Right )
            {
                _hand->setArticulation( osgbInteraction::HandNode::FINGER_0_ROTATE_INNER,
                    _hand->getArticulation( osgbInteraction::HandNode::FINGER_0_ROTATE_INNER ) - 0.05 );
                return true;
            }
            else if( ( ea.getKey() == 'H' ) || ( ea.getKey() == 'h' ) )
            {
                if( _hand->getHandedness() == osgbInteraction::HandNode::RIGHT )
                    _hand->setHandedness( osgbInteraction::HandNode::LEFT );
                else
                    _hand->setHandedness( osgbInteraction::HandNode::RIGHT );
                return( true );
            }
            else if( ( ea.getKey() == 'V' ) || ( ea.getKey() == 'v' ) )
            {
                _hand->setTraverseHand( !( _hand->getTraverseHand() ) );
                return( true );
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Home)
            {
                _hand->setPose( osgbInteraction::HandNode::POSE_DEFAULT );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_End)
            {
                _hand->setPose( osgbInteraction::HandNode::POSE_HOOK );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Page_Up)
            {
                _hand->setPose( osgbInteraction::HandNode::POSE_POINT );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Page_Down)
            {
                _hand->setPose( osgbInteraction::HandNode::POSE_FIST );
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Delete)
            {
                _hand->dump();
                return true;
            }
            else if( ea.getKey() == '=' )
            {
                _hand->setCalibrateMode( !( _hand->getCalibrateMode() ) );
                return true;
            }

            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F1)
            {
                _finger = &_params._finger0;
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F2)
            {
                _finger = &_params._finger1;
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F3)
            {
                _finger = &_params._finger2;
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F4)
            {
                _finger = &_params._finger3;
                return true;
            }
            else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_F5)
            {
                _finger = &_params._finger4;
                return true;
            }
            else
                return false;
            break;
        }

        case osgGA::GUIEventAdapter::PUSH:
        {
            if( !ourLeft && !ourRight )
                return false;

            _lastX = ea.getXnormalized();
            _lastY = ea.getYnormalized();
            return true;
            break;
        }
        case osgGA::GUIEventAdapter::DRAG:
        {
            if( ourRight )
            {
                osg::Vec3 move;
                if( ctrl )
                {
                    move[ 0 ] = _lastX - ea.getXnormalized();
                    move[ 1 ] = _lastY - ea.getYnormalized();
                }
                else if( shift )
                    move[ 2 ] = ea.getYnormalized() - _lastY;
                _lastX = ea.getXnormalized();
                _lastY = ea.getYnormalized();

                _params._pos += _params._att * move * 5.f;
                setParams = true;
            }
            else if( ourLeft )
            {
                if( ctrl )
                {
                    // X = Heading
                    // Y = Pitch
                    _h += ( _lastX - ea.getXnormalized() ) * 2.;
                    _p += ( _lastY - ea.getYnormalized() ) * 2.;
                }
                else if( shift )
                {
                    // X = Roll
                    _r += ( _lastX - ea.getXnormalized() ) * 2.;
                }
                _lastX = ea.getXnormalized();
                _lastY = ea.getYnormalized();

                osg::Quat q = osgwTools::makeHPRQuat(
                    osg::RadiansToDegrees( _h ),
                    osg::RadiansToDegrees( _p ),
                    osg::RadiansToDegrees( _r ) );
                _params._att = q;
                setParams = true;
            }
            else
                return( false );
            break;
        }
        default:
            return( false );
        break;
    }

    if( setParams )
    {
        _hand->setAll( _params );
        return( true );
    }
    else
        return( false );
}

// namespace osgbInteraction
}
