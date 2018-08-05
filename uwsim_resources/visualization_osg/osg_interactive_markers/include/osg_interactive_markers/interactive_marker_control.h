/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * - May 2012: Port to OpenSceneGraph. Mario Prats
 */

#ifndef OSG_INTERACTIVE_MARKER_CONTROL_H_
#define OSG_INTERACTIVE_MARKER_CONTROL_H_

#include <visualization_msgs/InteractiveMarkerControl.h>
#include "draggers.h"
#include <osgManipulator/Selection>

#include <osg_markers/marker_base.h>

#include <boost/shared_ptr.hpp>

#include <osg/Node>
#include <osg/Vec3>
#include <osg/Vec2>
#include <osg/Quat>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg_utils/osg_utils.h>


namespace osg_interactive_markers
{
class InteractiveMarker;

/**
 * A single control element of an InteractiveMarker.
 */
class InteractiveMarkerControl
{
public:

	InteractiveMarkerControl(const visualization_msgs::InteractiveMarkerControl &message,
			osg::ref_ptr<CustomCompositeDragger> int_marker_node, InteractiveMarker *parent );

	virtual ~InteractiveMarkerControl();

	// called when interactive mode is globally switched on/off
	virtual void enableInteraction(bool enable);


	/** Update the pose of the interactive marker being controlled,
	 * relative to the reference frame.  Each InteractiveMarkerControl
	 * maintains its pose relative to the reference frame independently,
	 * so when the parent InteractiveMarker mvoes, it calls this
	 * function on all its child controls. */
	void interactiveMarkerPoseChanged( osg::Vec3d int_marker_position, osg::Quat int_marker_orientation );

	bool isInteractive() { return interaction_mode_ != visualization_msgs::InteractiveMarkerControl::NONE; }

	// Called every frame by parent's update() function.
	void update();

	void setVisible( bool visible );

	bool dragging_;

protected:

	osg::ref_ptr<CustomCompositeDragger> int_marker_node_;
	osg::ref_ptr<osg::MatrixTransform> markers_node_;

	// interaction mode
	int interaction_mode_;
	int orientation_mode_;

	// if in view facing mode, the markers should be
	// view facing as well
	// if set to false, they will follow the parent's transformations
	bool independent_marker_orientation_;

	/** Defines the axis / plane along which to transform.  This is not
	 * keeping track of rotations applied to the control by the user,
	 * this is just a copy of the "orientation" parameter from the
	 * InteractiveMarkerControl message. */
	osg::Quat control_orientation_;

	bool always_visible_;

	std::string description_;

	std::string name_;

	typedef boost::shared_ptr<osg_markers::MarkerBase> MarkerBasePtr;
	std::vector< MarkerBasePtr > markers_;

	InteractiveMarker *parent_;

	/** Stores the rotation around the x axis of the control.  Only
	 * relevant for fixed-orientation rotation controls. */
	double rotation_;

	/** Stores the rotation around the x axis of the control as it was
	 * when the mouse-down event happened.  Only relevant for
	 * fixed-orientation rotation controls. */
	double rotation_at_mouse_down_;

	osg::Quat intitial_orientation_;

	/** The 3D position of the mouse click when the mouse button is
	 * pressed, relative to the reference frame. */
	osg::Vec3d grab_point_;

	// The 2D position in pixel coordinates of the mouse-down location.
	osg::Vec2 grab_pixel_;
	// The position of the parent when the mouse button is pressed.
	osg::Vec3d parent_position_at_mouse_down_;

	/** The orientation of the control_frame_node_ when the mouse button
	 * is pressed. */
	osg::Quat control_frame_orientation_at_mouse_down_;

	/** The orientation of the parent when the mouse button
	 * is pressed. */
	osg::Quat parent_orientation_at_mouse_down_;

	/** The direction vector of the axis of rotation during a mouse
	 * drag, relative to the reference frame.  Computed on mouse down
	 * event. */
	osg::Vec3d rotation_axis_;

	/** The center of rotation during a mouse drag, relative to the
	 * control frame.  Computed on mouse down event. */
	osg::Vec3d rotation_center_rel_control_;

	/** The grab point during a mouse drag, relative to the control
	 * frame.  Computed on mouse down event. */
	osg::Vec3d grab_point_rel_control_;

	bool has_focus_;
	bool interaction_enabled_;

	bool visible_;
};

}

#endif /* OSG_INTERACTIVE_MARKER_CONTROL_H_ */
