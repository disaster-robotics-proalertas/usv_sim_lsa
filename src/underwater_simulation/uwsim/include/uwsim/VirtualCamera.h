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

#ifndef VIRTUALCAMERA_H_
#define VIRTUALCAMERA_H_

#include "SimulatorConfig.h"
#include "CustomWidget.h"
#include "ConfigXMLParser.h"
#include "UWSimUtils.h"

#include <osgViewer/Viewer>
#include <osgGA/NodeTrackerManipulator>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osgGA/GUIEventHandler>
#include <osg/Geometry>
#include <osg/NodeTrackerCallback>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Vec3>
#include <osg/Drawable>
#include <osg/LineWidth>
#include <osg/io_utils>

#include <GL/glu.h>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

/** A camera associated to a viewer */
class VirtualCamera : public CustomWidget, public osg::Referenced
{

  //Custom node tracking
  class MyNodeTrackerCallback : public osg::NodeTrackerCallback
  {
    osg::ref_ptr<osg::Camera> osgcamera;
    osg::ref_ptr<osg::Image> zbuffer;
    osg::Matrixd previous_wMc;

    double show_path_;

    ros::Time previous, current;

    void operator()(osg::Node *node, osg::NodeVisitor *nv)
    {
      //std::cerr << "Node Tracker callback" << std::endl;
      osg::Matrixd m;
      //((osg::Transform*)node)->computeWorldToLocalMatrix(m,nv);
      m = osg::computeWorldToLocal(nv->getNodePath());
      traverse(node, nv);
      osgcamera->setViewMatrix(m);

      //Check for the time ellapsed since the last time. If greater than showfov rate (if defined),
      //and displacement greater than min_displacement, attach the current FOV to the root
      current = ros::Time::now();
      if (show_path_ > 0 && (current - previous).toSec() > show_path_)
      {
        previous = current;

        osg::Matrixd wMc = osgcamera->getViewMatrix();
        if ((wMc.getTrans() - previous_wMc.getTrans()).length() > 0.15)
        {
          previous_wMc = wMc;

          osg::Matrixd projm = osgcamera->getProjectionMatrix();

          GLint viewport[4];
          viewport[0] = osgcamera->getViewport()->x();
          viewport[1] = osgcamera->getViewport()->y();
          viewport[2] = osgcamera->getViewport()->width();
          viewport[3] = osgcamera->getViewport()->height();

          double z_b[4];
          z_b[0] = ((float*)zbuffer->data())[0];
          z_b[1] = ((float*)zbuffer->data())[viewport[2] - 1];
          z_b[2] = ((float*)zbuffer->data())[viewport[2] * viewport[3] - 1];
          z_b[3] = ((float*)zbuffer->data())[viewport[2] * viewport[3] - viewport[2]];

          if (z_b[0] > 0 || z_b[1] > 0 || z_b[2] > 0 || z_b[3] > 0)
          {
            double X[4], Y[4], Z[4];
            gluUnProject(0, 0, z_b[0] * 0.9999, wMc.ptr(), projm.ptr(), viewport, &X[0], &Y[0], &Z[0]);
            gluUnProject(viewport[2], 0, z_b[1] * 0.9999, wMc.ptr(), projm.ptr(), viewport, &X[1], &Y[1], &Z[1]);
            gluUnProject(viewport[2], viewport[3], z_b[2] * 0.9999, wMc.ptr(), projm.ptr(), viewport, &X[2], &Y[2],
                         &Z[2]);
            gluUnProject(0, viewport[3], z_b[3] * 0.9999, wMc.ptr(), projm.ptr(), viewport, &X[3], &Y[3], &Z[3]);

            cameraPathVertices->push_back(osg::Vec3d(X[0], Y[0], Z[0]));
            cameraPathVertices->push_back(osg::Vec3d(X[1], Y[1], Z[1]));
            cameraPathVertices->push_back(osg::Vec3d(X[2], Y[2], Z[2]));
            cameraPathVertices->push_back(osg::Vec3d(X[3], Y[3], Z[3]));

            cameraPathVertices->push_back(osg::Vec3d(X[0], Y[0], Z[0]));
            cameraPathGeometry->setVertexArray(cameraPathVertices);
            ((osg::DrawArrays*)cameraPathPrset.get())->setFirst(0);
            ((osg::DrawArrays*)cameraPathPrset.get())->setCount(cameraPathVertices->size());
          }
        }
      }
    }
  public:
    //Attributes that store the camera path projected on the terrain
    osg::ref_ptr<osg::Switch> cameraPathSwitch;
    osg::ref_ptr<osg::Geode> cameraPathGeode;
    osg::ref_ptr<osg::Geometry> cameraPathGeometry;
    osg::ref_ptr<osg::Vec3Array> cameraPathVertices;
    osg::ref_ptr<osg::Vec4Array> cameraPathColors;
    osg::ref_ptr<osg::PrimitiveSet> cameraPathPrset;

    MyNodeTrackerCallback(osg::Group *uwsim_root, osg::Image *zbuffer, osg::Camera *cam)
    {
      this->zbuffer = zbuffer;
      this->osgcamera = cam;
      show_path_ = 0;

      previous = ros::Time::now();

      cameraPathSwitch = new osg::Switch();
      uwsim_root->addChild(cameraPathSwitch);
      cameraPathSwitch->setAllChildrenOn();

      cameraPathGeode = new osg::Geode();
      cameraPathGeometry = new osg::Geometry();
      cameraPathVertices = new osg::Vec3Array;
      cameraPathGeometry->setVertexArray(cameraPathVertices);
      cameraPathColors = new osg::Vec4Array;
      cameraPathColors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f)); //TODO: Allow user set color
      cameraPathGeometry->setColorArray(cameraPathColors);
      cameraPathGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
      cameraPathPrset = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, 0);
      cameraPathGeometry->addPrimitiveSet(cameraPathPrset);
      cameraPathGeode->addDrawable(cameraPathGeometry);

      osg::LineWidth* linewidth = new osg::LineWidth();
      linewidth->setWidth(3.0f);
      cameraPathGeode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

      cameraPathSwitch->addChild(cameraPathGeode);

    }

    void showPath(double rate)
    {
      show_path_ = rate;
    }
  };

public:
  std::string name, parentLinkName;
  osg::ref_ptr<osg::Group> uwsim_root;
  osg::ref_ptr<osg::Camera> textureCamera;
  osg::Node *trackNode;
  osg::ref_ptr<MyNodeTrackerCallback> node_tracker;

  int width, height, range;
  double fx, fy, cx, cy; ///< intrinsic parameters
  double aspectRatio, fov, far, near, k;
  double baseline; //Only for stereo. Default=0
  double Tx, Ty; //Only for stereo.
  std::string frameId; //Default=""
  int paramsOn;
  int bw; //BlackWhite camera
  int widget; //Widget available or not
  float std;//Camera noise

  osg::ref_ptr<osg::Image> renderTexture; //RGB image
  osg::ref_ptr<osg::Image> depthTexture; //Range image

  VirtualCamera(osg::Group *uwsim_root, std::string name,std::string parentName, osg::Node *trackNode, int width, double fov, double range);
  VirtualCamera(osg::Group *uwsim_root, std::string name,std::string parentName, osg::Node *trackNode, int width, int height, double fov,
                double aspectRatio);
  VirtualCamera(osg::Group *uwsim_root, std::string name,std::string parentName, osg::Node *trackNode, int width, int height, double baseline,
                std::string frameId,double fov,SceneBuilder *oscene,float std, Parameters *params, int range, int bw);
  VirtualCamera();

  void init(osg::Group *uwsim_root, std::string name,std::string parentName, osg::Node *trackNode, int width, int height, double baseline,
            std::string frameId, Parameters *params, int range, double fov, double aspectRatio, double near, double far,
            int bw, int widget,SceneBuilder *oscene, float std);

  //Creates the uniforms and loads the shader for the camera.
  void loadShaders(SceneBuilder *oscene);

  int getTFTransform(tf::Pose & pose, std::string & parent);

  void createCamera();

  void showPath(double rate)
  {
    node_tracker->showPath(rate);
  }

  //Interface to be implemented by widgets. Build a widget window with the data to be displayed
  osg::ref_ptr<osgWidget::Window> getWidgetWindow();
};

#endif /* VIRTUALCAMERA_H_ */
