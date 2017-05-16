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

#ifndef CUSTOMWIDGET
#define CUSTOMWIDGET

#include "SimulatorConfig.h"

#include <osgWidget/Util>
#include <osgWidget/WindowManager>
#include <osgWidget/Box>

class CustomWidget
{
public:
  CustomWidget()
  {
  }

  virtual osg::ref_ptr<osgWidget::Window> getWidgetWindow()=0;

  ~CustomWidget()
  {
  }
};

#endif

