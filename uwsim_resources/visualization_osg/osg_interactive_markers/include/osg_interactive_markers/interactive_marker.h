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

#ifndef OSG_INTERACTIVE_MARKER_H_
#define OSG_INTERACTIVE_MARKER_H_

#include "interactive_marker_control.h"
#include "draggers.h"

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <geometry_msgs/Pose.h>

#include <osg/PositionAttitudeTransform>
#include <osgManipulator/Selection>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/Vec3d>
#include <osg/Quat>
#include <osg_utils/osg_utils.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

#include <ros/publisher.h>

namespace osg_interactive_markers {

class InteractiveMarkerDisplay;

class InteractiveMarker
{
public:
	InteractiveMarker( InteractiveMarkerDisplay *owner, osg::Node *root, std::string topic_ns, std::string client_id );
	virtual ~InteractiveMarker();

	// reset contents to reflect the data from a new message
	// @return success
	bool processMessage( visualization_msgs::InteractiveMarkerConstPtr message );

	// reset contents to reflect the data from a new message
	// @return success
	void processMessage( visualization_msgs::InteractiveMarkerPoseConstPtr message );

	// called every frame update
	void update(float wall_dt);

	// set the pose of the parent frame, relative to the fixed frame
	void setReferencePose( osg::Vec3d position, osg::Quat orientation );

	// directly set the pose, relative to parent frame
	// if publish is set to true, publish the change
	void setPose( osg::Vec3d position, osg::Quat orientation, const std::string &control_name );

	void translate( osg::Vec3d delta_position, const std::string &control_name );
	void rotate( osg::Quat delta_orientation, const std::string &control_name );

	// schedule a pose reset once dragging is finished
	void requestPoseUpdate( osg::Vec3d position, osg::Quat orientation );

	void startDragging();
	void stopDragging();

	const osg::Vec3d getPosition() { return position_; }
	const osg::Quat& getOrientation() { return orientation_; }

	float getSize() { return scale_; }
	const std::string &getReferenceFrame() { return reference_frame_; }
	const std::string& getName() { return name_; }

	// show name above marker
	void setShowDescription( bool show );

	// fill in current marker pose & name, publish
	void publishFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback,
			bool mouse_point_valid = false,
			const osg::Vec3d& mouse_point_rel_world = osg::Vec3d(0,0,0) );

protected:

	void publishPose();

	void reset();

	void updateReferencePose();

	InteractiveMarkerDisplay *owner_;

	std::string reference_frame_;
	ros::Time reference_time_;
	bool frame_locked_;

	osg::ref_ptr<osg::MatrixTransform> reference_node_;

	osg::ref_ptr<CustomCompositeDragger> int_marker_node_;
	osg::ref_ptr<osgManipulator::Selection> int_marker_node_selection_;

	// pose being controlled, relative to reference frame
	osg::Vec3 position_;
	osg::Quat orientation_;

	// has the pose changed since the last feedback was sent?
	bool pose_changed_;
	double time_since_last_feedback_;

	typedef boost::shared_ptr<InteractiveMarkerControl> InteractiveMarkerControlPtr;
	std::list<InteractiveMarkerControlPtr> controls_;

	std::string name_;
	std::string description_;

	bool dragging_;
	std::string old_target_frame_;

	// pose being controlled
	bool pose_update_requested_;
	osg::Vec3 requested_position_;
	osg::Quat requested_orientation_;

	float scale_;

	// which control has popped up the menu
	std::string last_control_name_;

	double heart_beat_t_;

	// visual aids
	osg::ref_ptr<osg::MatrixTransform> tf_node_;

	InteractiveMarkerControlPtr description_control_;

	ros::Publisher feedback_pub_;
	std::string topic_ns_;
	std::string client_id_;

	boost::recursive_mutex mutex_;

};

}

#endif /* INTERACTIVE_MARKER_H_ */
