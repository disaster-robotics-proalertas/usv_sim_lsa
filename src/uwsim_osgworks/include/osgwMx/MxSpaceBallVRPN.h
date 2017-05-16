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

#ifndef __OSGWMX_MX_SPACE_BALL_VRPN_H__
#define __OSGWMX_MX_SPACE_BALL_VRPN_H__ 1


#include <osgwMx/Export.h>
#include <osgwMx/MxSpaceBall.h>
#include <osg/Object>

#define VRPN_CLIENT_ONLY
#include <vrpn_Configure.h>
#include <vrpn_Analog.h>
#include <vrpn_Button.h>

#include <list>


namespace osgwMx {


/** \class MxSpaceBallVRPN MxSpaceBallVRPN.h <osgwMx/MxSpaceBallVRPN.h>
\brief A DirectInput implementation of the MxSpaceBall class.
*/
class OSGWMX_EXPORT MxSpaceBallVRPN : public MxSpaceBall
{
public:
    MxSpaceBallVRPN(const std::string &deviceName = "");
    MxSpaceBallVRPN( const MxSpaceBallVRPN& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwMx,MxSpaceBallVRPN);

    /** \brief Poll the Spaceball and send device state to the base class.

    Calling application should invoke this function once per frame.

    If you are implementing another MxSpaceBall-based class, and it is
    non-polling, you'll need to code up some kind of function to handle
    your events. Use poll() as a model for how to call into the base
    class in response to the events you receive.
    */
    bool poll( const double elapsedSeconds );

protected:
    virtual ~MxSpaceBallVRPN();


    //
    // VRPN Support
    //

    /** \brief Handle VRPN events and send device state to the base class.
    VRPN will invoke this when new analog events arrive.
	Must be static so it can be called as a C-style function without a this pointer.
    */
	static void VRPN_CALLBACK handle_analog( void* userData, const vrpn_ANALOGCB a );

    /** \brief Handle VRPN events and send device state to the base class.
    VRPN will invoke this when new button events arrive.
	Must be static so it can be called as a C-style function without a this pointer.
    */
	static void VRPN_CALLBACK handle_button( void* userData, const vrpn_BUTTONCB b );

    bool openVRPNDevice(const char *deviceName);
    void freeVRPNDevice();

	void processButtons( const vrpn_BUTTONCB b );
    void processAxes( const vrpn_ANALOGCB a );

	/** Sotred device name string */
	std::string _deviceName;

	/** VRPN device handle for buttons */
	vrpn_Button_Remote* _vrpnButton;

	/** VRPN device handle for analog axes */
	vrpn_Analog_Remote* _vrpnAnalog;

	/** Number of seconds since last poll */
	double _deltaSeconds;

	/** current button state */
	unsigned int _buttons;


};


// osgwMx
}


// __OSGWMX_MX_SPACE_BALL_VRPN_H__
#endif
