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

#include "osg_interactive_markers/interactive_marker_control.h"

#include "osg_interactive_markers/interactive_marker.h"
#include "osg_interactive_markers/draggers.h" 

#include <osg_markers/shape_marker.h>
#include <osg_markers/arrow_marker.h>
#include "osg_markers/text_view_facing_marker.h"
#include "osg_markers/triangle_list_marker.h"
#include "osg_markers/mesh_resource_marker.h"

#include <osg/PositionAttitudeTransform>

#include <osg_utils/osg_utils.h>
#include <osg/Plane>

#include <osg/ComputeBoundsVisitor>

namespace osg_interactive_markers
{

InteractiveMarkerControl::InteractiveMarkerControl(  const visualization_msgs::InteractiveMarkerControl &message,
		osg::ref_ptr<CustomCompositeDragger> int_marker_node, InteractiveMarker *parent )
: dragging_(false)
, int_marker_node_(int_marker_node)
, parent_(parent)
, rotation_(0)
, grab_point_(0,0,0)
, interaction_enabled_(false)
, visible_(true)
{
	name_ = message.name;
	interaction_mode_ = message.interaction_mode;
	always_visible_ = message.always_visible;


	orientation_mode_ = message.orientation_mode;

	description_ = message.description;

	control_orientation_ = osg::Quat(message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w);

	switch (interaction_mode_) {
	case visualization_msgs::InteractiveMarkerControl::MOVE_AXIS: {
		CustomTranslate1DDragger *dragger=new CustomTranslate1DDragger(parent, this);
		dragger->getOrCreateStateSet()->clear();
		markers_node_=dragger;
		//Add to parent composite dragger
		int_marker_node_->addChild(dragger);
		int_marker_node_->addDragger(dragger);
		break;}
	case visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS: {
		CustomRotateCylinderDragger *dragger=new CustomRotateCylinderDragger(parent, this);
		dragger->getOrCreateStateSet()->clear();
		markers_node_=dragger;
		//Add to parent composite dragger
		int_marker_node->addChild(dragger);
		int_marker_node_->addDragger(dragger);
		break;
	}
	case visualization_msgs::InteractiveMarkerControl::MOVE_PLANE: {
		osg::Matrixd m;
		m.makeRotate(control_orientation_);
		osg::Vec3d v(1,0,0), vo=m*v;
		CustomTranslate2DDragger *dragger=new CustomTranslate2DDragger(parent, this, osg::Plane(vo,osg::Vec3d(0,0,0)));
		dragger->getOrCreateStateSet()->clear();
		markers_node_=dragger;
		//Add to parent composite dragger
		int_marker_node_->addChild(dragger);
		int_marker_node_->addDragger(dragger);
		break;
	}
	default:
		ROS_WARN_STREAM("Interaction mode still not supported: " << interaction_mode_);
		markers_node_=new osg::MatrixTransform();
		int_marker_node_->asGroup()->addChild(markers_node_);
	}
	markers_node_->setName("InteractiveMarkerControl markers_node_");

	independent_marker_orientation_ = message.independent_marker_orientation;

	if ( orientation_mode_ == visualization_msgs::InteractiveMarkerControl::INHERIT )
	{
		intitial_orientation_ = parent->getOrientation();
	}

	for (unsigned i = 0; i < message.markers.size(); i++)
	{
		MarkerBasePtr marker;

		// create a marker with the given type
		switch (message.markers[i].type)
		{
		case visualization_msgs::Marker::CUBE:
		case visualization_msgs::Marker::CYLINDER:
		case visualization_msgs::Marker::SPHERE:
		{
			marker.reset(new osg_markers::ShapeMarker(markers_node_.get()));
		}
		break;

		case visualization_msgs::Marker::ARROW:
		{
			marker.reset(new osg_markers::ArrowMarker(markers_node_.get()));
		}
		break;

		case visualization_msgs::Marker::LINE_STRIP:
		{
			ROS_WARN("InteractiveMarkerControl::InteractiveMarkerControl LineStripMarker to be implemented");
			//marker.reset(new LineStripMarker(0, vis_manager_, markers_node_));
		}
		break;
		case visualization_msgs::Marker::LINE_LIST:
		{
			ROS_WARN("InteractiveMarkerControl::InteractiveMarkerControl LineListMarker to be implemented");
			//marker.reset(new LineListMarker(0, vis_manager_, markers_node_));
		}
		break;
		case visualization_msgs::Marker::SPHERE_LIST:
		case visualization_msgs::Marker::CUBE_LIST:
		case visualization_msgs::Marker::POINTS:
		{
			ROS_WARN("InteractiveMarkerControl::InteractiveMarkerControl PointsMarker to be implemented TODO");
			//PointsMarkerPtr points_marker;
			//points_marker.reset(new PointsMarker(0, vis_manager_, markers_node_));
			//points_markers_.push_back( points_marker );
			//marker = points_marker;
		}
		break;
		case visualization_msgs::Marker::TEXT_VIEW_FACING:
		{
			marker.reset(new osg_markers::TextViewFacingMarker(/*0, vis_manager_,*/ markers_node_.get()));
		}
		break;
		case visualization_msgs::Marker::MESH_RESOURCE:
		{
			marker.reset(new osg_markers::MeshResourceMarker(/*0, vis_manager_,*/ markers_node_.get()));
		}
		break;

		case visualization_msgs::Marker::TRIANGLE_LIST:
		{
			marker.reset(new osg_markers::TriangleListMarker(/*0, vis_manager_,*/(osg::Node*)markers_node_.get()));
		}
		break;
		default:
			ROS_ERROR( "Unknown marker type: %d", message.markers[i].type );
		}

		if (marker) {
			marker->setMessage(message.markers[i]);
			markers_.push_back(marker);
			//Apply scale to draggers of mesh resources to keep the right scale in the object.
                        if(message.markers[i].type== visualization_msgs::Marker::MESH_RESOURCE){

                          osg::ComputeBoundsVisitor cbv;
                          marker->scene_node_->accept(cbv);
                          osg::BoundingBox box = cbv.getBoundingBox();

			  double scale= std::max( std::max ( fabs(box.xMax()-box.xMin()) + fabs(box.xMax()+box.xMin())/2 ,fabs(box.yMax()-box.yMin())+ fabs(box.yMax()+box.yMin())/2 )
					 ,fabs(box.zMax()-box.zMin()) + fabs(box.zMax()+box.zMin())/2 ) *1.2;

                          marker->setScaleBase( 1.0/scale );
                          osg::Vec3 trans = int_marker_node->getMatrix().getTrans();
                          int_marker_node->setMatrix( osg::Matrix::scale(osg::Vec3(scale,scale,scale)) * osg::Matrix::translate(trans) );
                        }


		}
	}

}

InteractiveMarkerControl::~InteractiveMarkerControl()
{
	if (int_marker_node_->getNumChildren()>0)
		int_marker_node_->removeChildren(0,int_marker_node_->getNumChildren());
}

void InteractiveMarkerControl::setVisible( bool visible )
{
	visible_ = visible;
	//TODO
}

void InteractiveMarkerControl::update()
{
	if( dragging_ )
	{
		osg::Matrixd transform=int_marker_node_->getWorldMatrices()[0];
		parent_->setPose(transform.getTrans(), transform.getRotate(), name_);
	}
}

void InteractiveMarkerControl::enableInteraction( bool enable )
{
	interaction_enabled_ = enable;
	setVisible(visible_);
}

void InteractiveMarkerControl::interactiveMarkerPoseChanged(
		osg::Vec3d int_marker_position, osg::Quat int_marker_orientation )
{
	osg::Matrixd markers_node_transform;
	markers_node_transform.setTrans(int_marker_position);

	switch (orientation_mode_)
	{
	case visualization_msgs::InteractiveMarkerControl::INHERIT:
		break;

	case visualization_msgs::InteractiveMarkerControl::FIXED:
	{
		//TODO
		//control_frame_node_->setOrientation(intitial_orientation_ * osg::Quat(
		//rotation_, control_orientation_.xAxis()));
		//markers_node_->setMatrix(int_marker_node_->getInverseMatrix());
		break;
	}

	case visualization_msgs::InteractiveMarkerControl::VIEW_FACING:
		if ( independent_marker_orientation_ )
		{
			//TODO
			markers_node_transform.setRotate(int_marker_orientation);
		}
		break;

	default:
		break;
	}
}


}
