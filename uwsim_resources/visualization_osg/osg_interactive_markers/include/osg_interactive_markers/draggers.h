#ifndef DRAGGERS_H 
#define DRAGGERS_H

#include <osgManipulator/Dragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/RotateCylinderDragger>
#include <osgManipulator/Projector>
#include <osg/Plane>

#include <osgManipulator/Projector>

namespace osg_interactive_markers {

class InteractiveMarker;
class InteractiveMarkerControl;

/** A custom, initially empty CompositeDragger */
class CustomCompositeDragger: public osgManipulator::CompositeDragger {
public:
	CustomCompositeDragger() : CompositeDragger() {}

	~CustomCompositeDragger() {}
};

/** A custom Translate1DDragger that removes material state set when released */
class CustomTranslate1DDragger: public osgManipulator::Translate1DDragger {
	InteractiveMarker *marker_base;
	InteractiveMarkerControl *marker_control;

public:
	CustomTranslate1DDragger(InteractiveMarker *marker, InteractiveMarkerControl *control) : osgManipulator::Translate1DDragger() {marker_base=marker; marker_control=control;}

	bool handle (const osgManipulator::PointerInfo &pi, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us);

	~CustomTranslate1DDragger() {}
};

/** A custom Translate2DDragger that removes material state set when released */
class CustomTranslate2DDragger: public osgManipulator::Translate2DDragger {
	InteractiveMarker *marker_base;
	InteractiveMarkerControl *marker_control;

public:
	CustomTranslate2DDragger(InteractiveMarker *marker, InteractiveMarkerControl *control, osg::Plane p) : osgManipulator::Translate2DDragger(p) {marker_base=marker; marker_control=control;}

	bool handle (const osgManipulator::PointerInfo &pi, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us);

	~CustomTranslate2DDragger() {}
};

/** A custom RotateCylinder that removes material state set when released */
class CustomRotateCylinderDragger: public osgManipulator::RotateCylinderDragger {
	InteractiveMarker *marker_base;
	InteractiveMarkerControl *marker_control;

public:
	CustomRotateCylinderDragger(InteractiveMarker *marker, InteractiveMarkerControl *control) {
		marker_base=marker; marker_control=control;

		//Modify the default rotation axis
		osg::Cylinder *c=new osg::Cylinder();
		c->setRotation(osg::Quat(M_PI_2,osg::Vec3d(0,1,0)));
		_projector = new osgManipulator::CylinderPlaneProjector(c);
	}

	bool handle (const osgManipulator::PointerInfo &pi, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us);

	~CustomRotateCylinderDragger() {}
};

}
#endif
