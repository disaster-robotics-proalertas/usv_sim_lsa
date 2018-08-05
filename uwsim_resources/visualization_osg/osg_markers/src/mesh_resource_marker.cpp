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
#include "osg_markers/mesh_resource_marker.h"
#include <ros/package.h>
#include <osgDB/ReadFile>

namespace osg_markers
{

osg::Node *loadMeshFromResource(std::string file) {
	size_t spos=file.find_first_of(':');
	std::string source=file.substr(0,spos);
	if (source!=std::string("package")) {
		ROS_WARN("MeshResourceMarker: Only package:// resources supported for now");
		//TODO: Process also file:// resources
	}
	std::string package=file.substr(spos+3,file.find_first_of('/', spos+3)-spos-3);

	std::string package_path=ros::package::getPath(package);
	if (package_path!=std::string("")) {
		std::string rel_path=file.substr(file.find_first_of('/', spos+3));
		std::string full_path=package_path+rel_path;
		return osgDB::readNodeFile(full_path);
	} else {
		ROS_ERROR("Cannot find path to package %s", package.c_str());
		return NULL;
	}
	return NULL;
}


MeshResourceMarker::MeshResourceMarker(osg::Node* parent_node)
: MarkerBase(parent_node)
, entity_(0)
{
}

MeshResourceMarker::~MeshResourceMarker()
{
	reset();
}

void MeshResourceMarker::reset()
{
	//destroy entity
	if (entity_)
	{
		entity_.release();
	}
}


void MeshResourceMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
	ROS_ASSERT(new_message->type == visualization_msgs::Marker::MESH_RESOURCE);

	if (!entity_ || old_message->mesh_resource != new_message->mesh_resource)
	{
		reset();

		if (new_message->mesh_resource.empty())
		{
			return;
		}

		osg::Node *mesh=loadMeshFromResource(std::string(new_message->mesh_resource));
		if (!mesh)
		{
			std::stringstream ss;
			ss << "Mesh resource marker could not load [" << new_message->mesh_resource << "]";
			ROS_DEBUG("%s", ss.str().c_str());
			return;
		}

		static uint32_t count = 0;
		std::stringstream ss;
		ss << "mesh_resource_marker_" << count++;
		std::string id = ss.str();
		entity_ = mesh;
		scene_node_->asGroup()->addChild(entity_);
	}

	osg::Vec3 pos, scale;
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

