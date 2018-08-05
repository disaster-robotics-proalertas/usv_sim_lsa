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

#ifndef __OSGBINTERACTION_HAND_TEST_EVENT_HANDLER_H__
#define __OSGBINTERACTION_HAND_TEST_EVENT_HANDLER_H__


#include <osgbInteraction/Export.h>
#include <osgbInteraction/HandNode.h>
#include <osgGA/GUIEventHandler>


namespace osgbInteraction
{


/** \class HandTestEventHandler HandTestEventHandler.h <osgbInteraction/HandTestEventHandler.h>
\brief Allows keyboard and mouse control of the HandNode.

*/
class OSGBINTERACTION_EXPORT HandTestEventHandler : public osgGA::GUIEventHandler
{
public:
    HandTestEventHandler( osgbInteraction::HandNode* hn );

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& );

protected:
    osg::ref_ptr< osgbInteraction::HandNode > _hand;
    osgbInteraction::HandNode::Articulation _mode;

    float _lastX, _lastY;
    float _h, _p, _r;
};


/** \class VirtualHandTestEventHandler HandTestEventHandler.h <osgbInteraction/HandTestEventHandler.h>
\brief Allows keyboard and mouse control of the HandNode.

Contains support for the virtual articulations (spread angle between fingers, and
middle/outer knuckle synchronization).

Also supports calibration mode, toggle right/left, and visibility. See source file
comment block for the handle() function.
*/
class OSGBINTERACTION_EXPORT VirtualHandTestEventHandler : public osgGA::GUIEventHandler
{
public:
    VirtualHandTestEventHandler( osgbInteraction::HandNode* hn );

    /** \brief Handle events.
    \li Home: Go to the default position.
    \li End: Hook the index finger.
    \li PgUp: Point the index finger/
    \li PgDn: Make a fist.
    \li F1-F5: Specify a single finger to articulate.
    \li Scroll: With shift, ctrl, and alt, exercises articulations for the selected finger.
    \li Right mouse drag + ctrl: xy motion.
    \li Right mouse drag + shift: z motion.
    \li Left mouse drag + ctrl: heading and pitch
    \li Left mouse drag + shift: roll
    */
    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& );

protected:
    osg::ref_ptr< osgbInteraction::HandNode > _hand;

    osgbInteraction::HandNode::AllParams _params;
    osg::Vec2f* _finger;

    float _lastX, _lastY;
    float _h, _p, _r;
};


// namespace osgbInteraction
}

// __OSGBINTERACTION_HAND_TEST_EVENT_HANDLER_H__
#endif
