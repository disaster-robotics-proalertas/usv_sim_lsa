/*
 * Copyright (c) 2009, Willow Garage, Inc.
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


#include <ros/assert.h>

#include "osg_markers/shape_marker.h"
#include <osg/Shape>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>

namespace osg_markers 
{

ShapeMarker::ShapeMarker( osg::Node* parent_node ) :
		  MarkerBase(parent_node)
{
}

ShapeMarker::~ShapeMarker()
{
}

void ShapeMarker::onNewMessage( const MarkerConstPtr& old_message,
		const MarkerConstPtr& new_message )
{
	if (shape_.valid() || !old_message || old_message->type != new_message->type)
	{
		shape_.release();

		switch (new_message->type)
		{
		case visualization_msgs::Marker::CUBE:
		{
			shape_ = new osg::ShapeDrawable(new osg::Box());
			osg::Geode *geode = new osg::Geode();
			geode->addDrawable(shape_);
			scene_node_->asGroup()->addChild(geode);
		}
		break;

		case visualization_msgs::Marker::CYLINDER:
		{
			shape_ = new osg::ShapeDrawable(new osg::Cylinder());
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			geode->addDrawable(shape_);
			scene_node_->asGroup()->addChild(geode);
		}
		break;

		case visualization_msgs::Marker::SPHERE:
		{
			shape_ = new osg::ShapeDrawable(new osg::Sphere());
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			geode->addDrawable(shape_);
			scene_node_->asGroup()->addChild(geode);
		}
		break;

		default:
			ROS_BREAK();
			break;
		}
	}

	osg::Vec3d pos, scale;
	pos.set(new_message->pose.position.x, new_message->pose.position.y, new_message->pose.position.z  );
	scale.set(new_message->scale.x, new_message->scale.y, new_message->scale.z  );
	osg::Quat orient;
	orient.set(new_message->pose.orientation.x, new_message->pose.orientation.y, new_message->pose.orientation.z, new_message->pose.orientation.w  );

	setPosition(pos);
	setOrientation(orient);
	setScale(scale);

	setColor(osg::Vec4d(new_message->color.r, new_message->color.g,new_message->color.b, new_message->color.a));
}

}
