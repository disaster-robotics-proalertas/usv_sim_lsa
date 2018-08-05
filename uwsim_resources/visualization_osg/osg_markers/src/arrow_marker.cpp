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


#include "osg_markers/arrow_marker.h"


#include <tf/transform_listener.h>


#include <osg/Shape>
#include <osg/Geode>

namespace osg_markers
{

ArrowMarker::ArrowMarker( osg::Node* parent_node)
: MarkerBase(parent_node)
{
	cylinder_shape_=new osg::Cylinder();
	cylinder_drawable_= new osg::ShapeDrawable(cylinder_shape_);
	cone_shape_=new osg::Cone();
	cone_drawable_= new osg::ShapeDrawable(cone_shape_);
	osg::Geode *geode = new osg::Geode();
	geode->addDrawable(cylinder_drawable_);
	geode->addDrawable(cone_drawable_);

	arrow_=new osg::PositionAttitudeTransform();
	arrow_->asGroup()->addChild(geode);
	scene_node_->asGroup()->addChild(arrow_);
}

ArrowMarker::~ArrowMarker()
{

}

void ArrowMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
	ROS_ASSERT(new_message->type == visualization_msgs::Marker::ARROW);

	if (!new_message->points.empty() && new_message->points.size() < 2)
	{
		ROS_ERROR("Arrow marker only specified one point of a point to point arrow.");
		return;
	}

	osg::Vec3 pos(new_message->pose.position.x, new_message->pose.position.y, new_message->pose.position.z);
	osg::Quat orient(new_message->pose.orientation.x, new_message->pose.orientation.y, new_message->pose.orientation.z, new_message->pose.orientation.w);

	setPosition(pos);
	setOrientation( orient );
    //setScale(scale);	

	setColor(osg::Vec4d(new_message->color.r, new_message->color.g,new_message->color.b, new_message->color.a));

	// compute translation & rotation from the two points
	if (new_message->points.size() == 2)
	{
		osg::Vec3d point1( new_message->points[0].x, new_message->points[0].y, new_message->points[0].z );
		osg::Vec3d point2( new_message->points[1].x, new_message->points[1].y, new_message->points[1].z );

		osg::Vec3d direction = point2 - point1;
		float distance = direction.length();

		float head_length = 0.1*distance;
		if ( new_message->scale.z != 0.0 )
		{
			head_length = new_message->scale.z;
		}
		float shaft_length = distance - head_length;
		float head_radius=new_message->scale.y;

		cylinder_shape_->setRadius(new_message->scale.x / 4.0f);
		cylinder_shape_->setHeight(shaft_length);
		cylinder_shape_->setCenter(osg::Vec3d(0,0,shaft_length/2.0));

		cone_shape_->setRadius(head_radius / 4.0f);
		cone_shape_->setHeight(head_length);
		cone_shape_->setCenter(osg::Vec3d(0,0,shaft_length));

		direction.normalize();

		osg::Quat local_orient;
		local_orient.makeRotate(osg::Vec3d(0,0,1),direction);
		arrow_->setPosition(point1);
		arrow_->setAttitude( local_orient );
	}
	else
	{
		//TODO
	}
}

}
