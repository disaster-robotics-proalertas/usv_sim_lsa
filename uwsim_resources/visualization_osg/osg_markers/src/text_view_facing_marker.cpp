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
#include "osg_markers/text_view_facing_marker.h"

#include <osg/Vec3>
#include <osg/Quat>

namespace osg_markers
{

TextViewFacingMarker::TextViewFacingMarker(osg::Node* parent_node)
: MarkerBase(parent_node)
, text_(0)
{
}

TextViewFacingMarker::~TextViewFacingMarker()
{
}

void TextViewFacingMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
	ROS_ASSERT(new_message->type == visualization_msgs::Marker::TEXT_VIEW_FACING);

	if (!text_)
	{
		geode_= new osg::Geode;
		text_ = new osgText::Text;
		text_->setFont("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf");
		text_->setCharacterSize(0.3);
		text_->setPosition(osg::Vec3d(0,0,0));
		text_->setColor(osg::Vec4d(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a));
		//For some reason SCREEN alignment doesn't work under draggers :(
		//text_->setAxisAlignment(osgText::Text::SCREEN);
		text_->setAxisAlignment(osgText::Text::REVERSED_XZ_PLANE);

		// reproduce outline bounding box compute problem with backdrop on.
		//text_->setBackdropType(osgText::Text::OUTLINE);
		//text_->setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);
		text_->setDrawMode(osgText::Text::TEXT);

		text_->setText(new_message->text);
		geode_->addDrawable(text_);
		scene_node_->asGroup()->addChild(geode_);
	}

	setPosition(osg::Vec3d(new_message->pose.position.x, new_message->pose.position.y, new_message->pose.position.z));
	//setScale(osg::Vec3d(new_message->scale.x, new_message->scale.y, new_message->scale.z));
}


}

