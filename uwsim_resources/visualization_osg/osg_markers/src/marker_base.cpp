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

#include "osg_markers/marker_base.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Material>

namespace osg_markers {

MarkerBase::MarkerBase(osg::Node* parent_node)
{
	base_node_=parent_node;
	scene_node_=new osg::PositionAttitudeTransform();
	scene_node_->setName("MarkerBase Scale PAT");
	base_node_->asGroup()->addChild(scene_node_);
	message_.reset();

        scale_base_node=1.0;
}

MarkerBase::~MarkerBase()
{
}

void MarkerBase::setMessage(const Marker& message)
{
	// copy and save to shared pointer
	MarkerConstPtr message_ptr( new Marker(message) );
	setMessage( message_ptr );
}

void MarkerBase::setMessage(const MarkerConstPtr& message)
{
	MarkerConstPtr old = message_;
	message_ = message;

	onNewMessage(old, message);
}

void MarkerBase::setPosition( const osg::Vec3d& position )
{
	osg::Matrixd transform=base_node_->asTransform()->asMatrixTransform()->getMatrix();
	transform.setTrans(position);
	base_node_->asTransform()->asMatrixTransform()->setMatrix( transform );
}

void MarkerBase::setOrientation( const osg::Quat& orientation )
{
	osg::Matrixd transform=base_node_->asTransform()->asMatrixTransform()->getMatrix();
	transform.setRotate(orientation);
	base_node_->asTransform()->asMatrixTransform()->setMatrix( transform );
}

void MarkerBase::setScale( const osg::Vec3d& scale )
{
	scene_node_->asTransform()->asPositionAttitudeTransform()->setScale(scale*scale_base_node);
}


void MarkerBase::setScaleBase( double scale )
{
        scale_base_node=scale;
	scene_node_->asTransform()->asPositionAttitudeTransform()->setScale(getScale()*scale_base_node);
}

void MarkerBase::setColor( const osg::Vec4d& color ){
      	osg::ref_ptr < osg::Material > material = new osg::Material();
      	material->setDiffuse(osg::Material::FRONT_AND_BACK,color);
      	scene_node_->getOrCreateStateSet()->setAttribute(material);
}

const osg::Vec3d MarkerBase::getPosition()
{
	return base_node_->asTransform()->asMatrixTransform()->getMatrix().getTrans();
}

const osg::Quat MarkerBase::getOrientation()
{
	return base_node_->asTransform()->asMatrixTransform()->getMatrix().getRotate();
}

const osg::Vec3d MarkerBase::getScale()
{
	return scene_node_->asTransform()->asPositionAttitudeTransform()->getScale();

}

const double MarkerBase::getScaleBase()
{
	return scale_base_node;

}

}
