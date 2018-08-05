/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */

#ifndef HUDCAMERA_H_
#define HUDCAMERA_H_

#include "SimulatorConfig.h"
#include "CustomWidget.h"

#include <osg/Image>

/** A ROS Camera subscriber that can be displayed as a widget */
class HUDCamera : public CustomWidget
{
  osg::ref_ptr<osgWidget::Widget> widget;

  class widgetUpdateCallback : public osg::Drawable::UpdateCallback
  {
    osg::ref_ptr<osg::Image> image;
  public:
    widgetUpdateCallback(osg::Image *i) :
        osg::Drawable::UpdateCallback()
    {
      this->image = i;
    }
    virtual void update(osg::NodeVisitor *nv, osg::Drawable *d)
    {
      (static_cast<osgWidget::Widget*>(d))->setImage(image, true, false);
    }
  };

public:
  unsigned int width, height; ///< Width and height in pixels of the input image
  unsigned int posx, posy; ///< Default position of the widget, given in pixels wrt bottom-left corner (X to the right, Y upwards)
  double scale; ///< Percentage of default widget scaling (0..1)

  osg::ref_ptr<osg::Image> osg_image; //The osg::Image object where to store the ROS image
  bool ready_; //true if images have been acquired

  /** Constructor from the image and info topics */
  HUDCamera(unsigned int width, unsigned int height, unsigned int posx = 0, unsigned int posy = 0, double scale = 1,
            int blackWhite = 0);

  bool ready()
  {
    return ready_;
  }

  //Interface to be implemented by widgets. Build a widget window with the data to be displayed
  osg::ref_ptr<osgWidget::Window> getWidgetWindow();

  ~HUDCamera();
};

#endif /* HUDCAMERA_H_ */
