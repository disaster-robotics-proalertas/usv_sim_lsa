#include "osg_interactive_markers/draggers.h"
#include "osg_interactive_markers/interactive_marker.h"

namespace osg_interactive_markers {

bool CustomTranslate1DDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool ret=Translate1DDragger::handle(pointer, ea, aa);
	getOrCreateStateSet()->clear();

	//Handle dragging for feedback publishing
	if (ea.getEventType()==osgGA::GUIEventAdapter::DRAG) {
		marker_base->startDragging();
		marker_control->dragging_=true;
	} else if (ea.getEventType()==osgGA::GUIEventAdapter::RELEASE) {
		marker_base->stopDragging();
		marker_control->dragging_=false;
	}

	return ret;
}

bool CustomTranslate2DDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool ret=Translate2DDragger::handle(pointer, ea, aa);
	getOrCreateStateSet()->clear();

	//Handle dragging for feedback publishing
	if (ea.getEventType()==osgGA::GUIEventAdapter::DRAG) {
		marker_base->startDragging();
		marker_control->dragging_=true;
	} else if (ea.getEventType()==osgGA::GUIEventAdapter::RELEASE) {
		marker_base->stopDragging();
		marker_control->dragging_=false;
	}

	return ret;
}


bool CustomRotateCylinderDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool ret=RotateCylinderDragger::handle(pointer, ea, aa);
	getOrCreateStateSet()->clear();

	//Handle dragging for feedback publishing
	if (ea.getEventType()==osgGA::GUIEventAdapter::DRAG) {
		marker_base->startDragging();
		marker_control->dragging_=true;
	} else if (ea.getEventType()==osgGA::GUIEventAdapter::RELEASE) {
		marker_base->stopDragging();
		marker_control->dragging_=false;
	}

	return ret;
}

}
