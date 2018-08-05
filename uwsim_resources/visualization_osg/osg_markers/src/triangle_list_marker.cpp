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
#include "osg_markers/triangle_list_marker.h"
#include <osg/PositionAttitudeTransform>



namespace osg_markers
{

TriangleListMarker::TriangleListMarker(osg::Node* parent_node)
: MarkerBase(parent_node)
{
}

TriangleListMarker::~TriangleListMarker()
{
}

void TriangleListMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
	ROS_ASSERT(new_message->type == visualization_msgs::Marker::TRIANGLE_LIST);

	if (!manual_object_)
	{
		geom_ = new osg::Geometry;
		triangle_list_ = new osg::Vec3Array;
		color_list_ = new osg::Vec4Array;
		geom_->setVertexArray( triangle_list_.get() );
		geom_->setColorArray( color_list_.get() );
		geom_->setColorBinding(osg::Geometry::BIND_OVERALL);
		prset_=new osg::DrawArrays( osg::PrimitiveSet::TRIANGLES);
		geom_->addPrimitiveSet(prset_);	
		manual_object_geode_ = new osg::Geode;
		manual_object_geode_->addDrawable( geom_.get() );
		scene_node_->asGroup()->addChild(manual_object_geode_.get());
	}

	size_t num_points = new_message->points.size();
	if ((num_points % 3) != 0)
	{
		std::stringstream ss;
		ss << "TriMesh marker has a point count which is not divisible by 3 [" << num_points <<"]";

		ROS_DEBUG("%s", ss.str().c_str());

		return;
	}

	osg::Vec3 pos(new_message->pose.position.x, new_message->pose.position.y, new_message->pose.position.z);
	osg::Vec3 scale(new_message->scale.x, new_message->scale.y, new_message->scale.z);
	osg::Quat orient(new_message->pose.orientation.x, new_message->pose.orientation.y, new_message->pose.orientation.z, new_message->pose.orientation.w);

	setPosition(pos);
	setOrientation(orient);
	setScale(scale);

	// If we have the same number of tris as previously, just update the object
	if (old_message && num_points == old_message->points.size())
	{
		//manual_object_->beginUpdate(0);
	}
	else // Otherwise clear it and begin anew
	{
		triangle_list_->clear();
		triangle_list_->resize(num_points);
		color_list_->clear();
		color_list_->resize(num_points);
	}

	bool has_vertex_colors = new_message->colors.size() == num_points;

	if (has_vertex_colors)
	{
		for (size_t i = 0; i < num_points; ++i)
		{
			(*triangle_list_)[i]= osg::Vec3(new_message->points[i].x, new_message->points[i].y, new_message->points[i].z) ;
			(*color_list_)[i]=osg::Vec4(new_message->colors[i].r, new_message->colors[i].g, new_message->colors[i].b, new_message->color.a);
			geom_->setVertexArray(triangle_list_);
			geom_->setColorArray(color_list_);
			geom_->setColorBinding(osg::Geometry::BIND_OVERALL);
			((osg::DrawArrays*)prset_)->setFirst(0);
			((osg::DrawArrays*)prset_)->setCount(num_points);
		}
	}
	else
	{
		for (size_t i = 0; i < num_points; ++i)
		{
			(*triangle_list_)[i]= osg::Vec3(new_message->points[i].x, new_message->points[i].y, new_message->points[i].z) ;
			(*color_list_)[i]=osg::Vec4(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);
			geom_->setVertexArray(triangle_list_);
			geom_->setColorArray(color_list_);
			geom_->setColorBinding(osg::Geometry::BIND_OVERALL);
			((osg::DrawArrays*)prset_)->setFirst(0);
			((osg::DrawArrays*)prset_)->setCount(num_points);
		}
	}

}


}

