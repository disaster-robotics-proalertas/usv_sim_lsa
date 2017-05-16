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

#include <osg_utils/osg_utils.h>
#include <osg_utils/frame_manager.h>
#include "osg_interactive_markers/interactive_marker.h"
#include "osg_interactive_markers/draggers.h"
#include <interactive_markers/tools.h>
#include <boost/make_shared.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>

#include <osg/NodeVisitor>
#include <osg/Matrix>

namespace osg_interactive_markers {

InteractiveMarker::InteractiveMarker( InteractiveMarkerDisplay *owner, osg::Node *root, std::string topic_ns, std::string client_id ) :
		  owner_(owner)
, pose_changed_(false)
, time_since_last_feedback_(0)
, dragging_(false)
, pose_update_requested_(false)
, heart_beat_t_(0)
, topic_ns_(topic_ns)
, client_id_(client_id)
{
	ros::NodeHandle nh;
	std::string feedback_topic = topic_ns+"/feedback";
	feedback_pub_ = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>( feedback_topic, 100, false );

	tf_node_ = new osg::MatrixTransform();
	tf_node_->setName("InteractiveMarker tf_node_");
	reference_node_=new osg::MatrixTransform();
	reference_node_->setName("InteractiveMarker reference_node_");

	int_marker_node_=new CustomCompositeDragger();
	int_marker_node_selection_=new osgManipulator::Selection;

	root->asGroup()->addChild(reference_node_);
	reference_node_->asGroup()->addChild(tf_node_);
	tf_node_->asGroup()->addChild(int_marker_node_);
}

InteractiveMarker::~InteractiveMarker()
{
	osg::Group *root=reference_node_->getParent(0);
	if (root->getNumChildren()>0)
		root->removeChild(reference_node_);
}

void InteractiveMarker::reset()
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	controls_.clear();
	//menu_entries_.clear();
}

void InteractiveMarker::processMessage( visualization_msgs::InteractiveMarkerPoseConstPtr message )
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	osg::Vec3d position( message->pose.position.x, message->pose.position.y, message->pose.position.z );
	osg::Quat orientation( message->pose.orientation.x,
			message->pose.orientation.y, message->pose.orientation.z, message->pose.orientation.w );

	if ( orientation.w() == 0 && orientation.x() == 0 && orientation.y() == 0 && orientation.z() == 0 )
	{
		orientation.set(0,0,0,1);
	}

	reference_time_ = message->header.stamp;
	reference_frame_ = message->header.frame_id;
	frame_locked_ = (message->header.stamp == ros::Time(0));

	requestPoseUpdate( position, orientation );
}

bool InteractiveMarker::processMessage( visualization_msgs::InteractiveMarkerConstPtr message )
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	reset();

	visualization_msgs::InteractiveMarker auto_message = *message;
	interactive_markers::autoComplete( auto_message );

	// copy values
	name_ = auto_message.name;
	description_ = auto_message.description;

	if ( auto_message.controls.size() == 0 )
	{
		return false;
	}

	scale_ = auto_message.scale;

	reference_frame_ = auto_message.header.frame_id;
	reference_time_ = auto_message.header.stamp;
	frame_locked_ = (auto_message.header.stamp == ros::Time(0));

	position_ = osg::Vec3d(
			auto_message.pose.position.x,
			auto_message.pose.position.y,
			auto_message.pose.position.z );

	orientation_ = osg::Quat(
			auto_message.pose.orientation.x,
			auto_message.pose.orientation.y,
			auto_message.pose.orientation.z,
			auto_message.pose.orientation.w);

	pose_changed_ =false;
	time_since_last_feedback_ = 0;

	// setup axes
	osg::Matrixd m;
	m.setTrans(position_);
	m.setRotate(orientation_);
	reference_node_->setMatrix(m);

	updateReferencePose();

	for ( unsigned i=0; i<auto_message.controls.size(); i++ )
	{
		controls_.push_back( boost::make_shared<InteractiveMarkerControl>(
				auto_message.controls[i], int_marker_node_, this ) );
	}

	description_control_ = boost::make_shared<InteractiveMarkerControl>(
			interactive_markers::makeTitle( auto_message ), int_marker_node_, this );
	controls_.push_back( description_control_ );

	//update composite dragger
	int_marker_node_->setParentDragger(int_marker_node_->getParentDragger());
	int_marker_node_->addTransformUpdating(int_marker_node_selection_);
	int_marker_node_->setHandleEvents(true);
	int_marker_node_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);


	//TODO: create menus
	return true;
}


void InteractiveMarker::updateReferencePose()
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	osg::Vec3d reference_position;
	osg::Quat reference_orientation;

	// if we're frame-locked, we need to find out what the most recent transformation time
	// actually is so we send back correct feedback
	if ( frame_locked_ )
	{
		std::string fixed_frame = osg_utils::FrameManager::instance()->getFixedFrame();
		if ( reference_frame_ == fixed_frame )
		{
			// if the two frames are identical, we don't need to do anything.
			reference_time_ = ros::Time::now();
		}
		else
		{
			std::string error;
			int retval = osg_utils::FrameManager::instance()->getTFClient()->getLatestCommonTime(
					reference_frame_, fixed_frame, reference_time_, &error );
			if ( retval != tf::NO_ERROR )
			{
				std::ostringstream s;
				s <<"Error getting time of latest transform between " << reference_frame_
						<< " and " << fixed_frame << ": " << error << " (error code: " << retval << ")";
				ROS_WARN_STREAM("Error getting time of latest transform between " << reference_frame_ << " and " << fixed_frame << ": " << error << " (error code: " << retval << ")" );
				return;
			}
		}
	}

	if (!osg_utils::FrameManager::instance()->getTransform( reference_frame_, reference_time_,
			reference_position, reference_orientation ))
	{
		std::string error;
		osg_utils::FrameManager::instance()->transformHasProblems(reference_frame_, reference_time_, error);
		ROS_WARN_STREAM("InteractiveMarker::updateReferencePose(): Transform error: " << error );
		return;
	}

	osg::Matrixd transform;
	transform.setTrans(reference_position);
	transform.setRotate(reference_orientation);
	tf_node_->setMatrix(transform);
}

void InteractiveMarker::update(float wall_dt)
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	time_since_last_feedback_ += wall_dt;
	if ( frame_locked_ )
	{
		updateReferencePose();
	}

	std::list<InteractiveMarkerControlPtr>::iterator it;
	for ( it = controls_.begin(); it != controls_.end(); it++ )
	{
		(*it)->update();
	}

	if ( dragging_ )
	{
		if ( pose_changed_ )
		{
			publishPose();
		}
		else if ( time_since_last_feedback_ > 0.25 )
		{
			//send keep-alive so we don't use control over the marker
			visualization_msgs::InteractiveMarkerFeedback feedback;
			feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::KEEP_ALIVE;
			publishFeedback( feedback );
		}
	}
}

void InteractiveMarker::publishPose()
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	visualization_msgs::InteractiveMarkerFeedback feedback;
	feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;
	feedback.control_name = last_control_name_;
	publishFeedback( feedback );
	pose_changed_ = false;
}

void InteractiveMarker::requestPoseUpdate( osg::Vec3d position, osg::Quat orientation )
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	if ( dragging_ )
	{
		pose_update_requested_ = true;
		requested_position_ = position;
		requested_orientation_ = orientation;
	}
	else
	{
		updateReferencePose();
		setPose( position, orientation, "" );
	}
}

void InteractiveMarker::setPose( osg::Vec3d position, osg::Quat orientation, const std::string &control_name )
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	position_ = position;
	orientation_ = orientation;
	pose_changed_ = true;
	last_control_name_ = control_name;

	std::list<InteractiveMarkerControlPtr>::iterator it;
	for ( it = controls_.begin(); it != controls_.end(); it++ )
	{
		(*it)->interactiveMarkerPoseChanged( position_, orientation_ );
	}
}

void InteractiveMarker::setShowDescription( bool show )
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	if ( description_control_.get() )
	{
		description_control_->setVisible( show );
	}
}

void InteractiveMarker::translate( osg::Vec3d delta_position, const std::string &control_name )
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	setPose( position_+delta_position, orientation_, control_name );
}

void InteractiveMarker::rotate( osg::Quat delta_orientation, const std::string &control_name )
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	setPose( position_, delta_orientation * orientation_, control_name );
}

void InteractiveMarker::startDragging()
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	dragging_ = true;
	pose_changed_ = false;
}

void InteractiveMarker::stopDragging()
{
	boost::recursive_mutex::scoped_lock lock(mutex_);
	if ( pose_changed_ )
	{
		publishPose();
	}
	dragging_ = false;
	if ( pose_update_requested_ )
	{
		updateReferencePose();
		setPose( requested_position_, requested_orientation_, "" );
		pose_update_requested_ = false;
	}
}



void InteractiveMarker::publishFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback,
		bool mouse_point_valid,
		const osg::Vec3d& mouse_point_rel_world )
{
	boost::recursive_mutex::scoped_lock lock(mutex_);

	feedback.client_id = client_id_;
	feedback.marker_name = name_;

	if ( frame_locked_ )
	{
		osg::Matrixd transform=int_marker_node_->getMatrix();
		
		feedback.header.frame_id = reference_frame_;
		feedback.header.stamp = reference_time_;
		feedback.pose.position.x = transform.getTrans().x();
		feedback.pose.position.y = transform.getTrans().y();
		feedback.pose.position.z = transform.getTrans().z();
		feedback.pose.orientation.x = transform.getRotate().x();
		feedback.pose.orientation.y = transform.getRotate().y();
		feedback.pose.orientation.z = transform.getRotate().z();
		feedback.pose.orientation.w = transform.getRotate().w();

		feedback.mouse_point_valid = mouse_point_valid;
		if( mouse_point_valid )
		{
			//TODO
			//osg::Vec3d mouse_rel_reference = reference_node_->convertWorldToLocalPosition( mouse_point_rel_world );
			//feedback.mouse_point.x = mouse_rel_reference.x();
			//feedback.mouse_point.y = mouse_rel_reference.y();
			//feedback.mouse_point.z = mouse_rel_reference.z();
		}
	}
	else
	{
		//TODO: This is probably wrong... still not tested
		feedback.header.frame_id = std::string(osg_utils::FrameManager::instance()->getFixedFrame());
		feedback.header.stamp = ros::Time::now();

		osg::Vec3d world_position = osg_utils::getWorldCoords(tf_node_)->getTrans();
		osg::Quat world_orientation = osg_utils::getWorldCoords(tf_node_)->getRotate();

		feedback.pose.position.x = world_position.x();
		feedback.pose.position.y = world_position.y();
		feedback.pose.position.z = world_position.z();
		feedback.pose.orientation.x = world_orientation.x();
		feedback.pose.orientation.y = world_orientation.y();
		feedback.pose.orientation.z = world_orientation.z();
		feedback.pose.orientation.w = world_orientation.w();

		feedback.mouse_point_valid = mouse_point_valid;
		feedback.mouse_point.x = mouse_point_rel_world.x();
		feedback.mouse_point.y = mouse_point_rel_world.y();
		feedback.mouse_point.z = mouse_point_rel_world.z();
	}

	feedback_pub_.publish( feedback );

	time_since_last_feedback_ = 0;
}

}
