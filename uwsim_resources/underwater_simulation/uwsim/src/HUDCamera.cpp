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

#include <uwsim/HUDCamera.h>
#include <string.h>
#include <iostream>

HUDCamera::HUDCamera(unsigned int width, unsigned int height, unsigned int posx, unsigned int posy, double scale,
                     int blackWhite)
{
  this->width = width;
  this->height = height;
  this->posx = posx;
  this->posy = posy;
  this->scale = scale;
  osg_image = new osg::Image();
  if (blackWhite)
  {
    osg_image->allocateImage(width, height, 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    memset(osg_image->data(), 0, width * height * 1 * sizeof(unsigned char));
  }
  else
  {
    osg_image->allocateImage(width, height, 1, GL_RGB, GL_UNSIGNED_BYTE);
    memset(osg_image->data(), 0, width * height * 3 * sizeof(unsigned char));
  }

  ready_ = false;
  //OSG_INFO << "HUDCamera::HUDCamera Constructor finished " << info_topic << std::endl;
}

osg::ref_ptr<osgWidget::Window> HUDCamera::getWidgetWindow()
{
  osg::ref_ptr < osgWidget::Box > box = new osgWidget::Box("HUDCameraBox", osgWidget::Box::HORIZONTAL, true);
  widget = new osgWidget::Widget("HUDCameraWidget", width, height);
  widget->setUpdateCallback(new widgetUpdateCallback(osg_image));
  //widget->setImage(osg_image,true,false);
  box->addWidget(widget);
  box->setX(posx);
  box->setY(posy);
  box->setScale(scale);
  box->getBackground()->setColor(1.0f, 0.0f, 0.0f, 0.8f);
  box->attachMoveCallback();
  box->attachScaleCallback();
  return box;
}

HUDCamera::~HUDCamera()
{
}

