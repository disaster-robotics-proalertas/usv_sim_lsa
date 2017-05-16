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

#include <uwsim/VirtualCamera.h>
#include <uwsim/UWSimUtils.h>
#include <uwsim/SceneBuilder.h>
#include <iostream>

#include <osg/PositionAttitudeTransform>

class UpdateUnderWater : public osg::Uniform::Callback
{
public:
  UpdateUnderWater(osg::Camera* camera) :
      mCamera(camera)
  {
  }
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    u->set(true); //TODO: Should check waterheight!!
  }

protected:
  osg::Camera* mCamera;
};

class UpdateNoiseSeed : public osg::Uniform::Callback
{
public:
  UpdateNoiseSeed()
  {
  }
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    u->set(osg::Vec4f(rand()/(float)RAND_MAX,rand()/(float)RAND_MAX,rand()/(float)RAND_MAX,rand()/(float)RAND_MAX));
  }

protected:
};

class UpdateEye : public osg::Uniform::Callback
{
public:
  UpdateEye(osg::Camera* camera) :
      mCamera(camera)
  {
  }
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    osg::Vec3d eye, center, up;
    mCamera->getViewMatrixAsLookAt(eye, center, up);
    u->set(eye);
  }

protected:
  osg::Camera* mCamera;
};

class UpdateVMI : public osg::Uniform::Callback
{
public:
  UpdateVMI(osg::Camera* camera) :
      mCamera(camera)
  {
  }
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    u->set(mCamera->getInverseViewMatrix());
  }

protected:
  osg::Camera* mCamera;
};

VirtualCamera::VirtualCamera()
{
}

void VirtualCamera::init(osg::Group *uwsim_root, std::string name, std::string parentName, osg::Node *trackNode, int width,
                         int height, double baseline, std::string frameId, Parameters *params, int range, double fov,
                         double aspectRatio, double near, double far, int bw, int widget, SceneBuilder *oscene, float std)
{
  this->uwsim_root = uwsim_root;
  this->name = name;
  this->parentLinkName=parentName;
  this->std=std;

  this->trackNode = trackNode;
  //Add a switchable frame geometry on the camera frame
  osg::ref_ptr < osg::Node > axis = UWSimGeometry::createSwitchableFrame();
  //Add label to switchable frame
  axis->asGroup()->addChild(UWSimGeometry::createLabel(name));
  this->trackNode->asGroup()->addChild(axis);

  this->width = width;
  this->height = height;
  this->baseline = baseline;
  this->frameId = frameId;
  this->fov = fov;
  this->aspectRatio = aspectRatio;
  this->near = near;
  this->far = far;
  if (params != NULL)
  {
    this->fx = params->fx;
    this->fy = params->fy;
    this->far = params->f;
    this->near = params->n;
    this->cx = params->x0;
    this->cy = params->y0;
    this->k = params->k;
    this->paramsOn = 1;
  }
  else
    this->paramsOn = 0;
  this->range = range;
  this->bw = bw;
  this->widget = widget;

  if (!range)
  {
    renderTexture = new osg::Image();
    renderTexture->allocateImage(width, height, 1, GL_RGB, GL_UNSIGNED_BYTE);
  }
  else
  {
    depthTexture = new osg::Image();
    depthTexture->allocateImage(width, height, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
  }

  createCamera();
  //Add a cull mask to hide Augmented Reality objects from virtual cameras
  if(oscene)
    textureCamera->setCullMask(~oscene->scene->getOceanScene()->getARMask());
  loadShaders(oscene);
}

VirtualCamera::VirtualCamera(osg::Group *uwsim_root, std::string name,std::string parentName, osg::Node *trackNode, int width,
                             double fov, double range)
{ //Used in multibeam
  //Z-buffer has very low resolution near far plane so we extend it and cut far plane later.
  init(uwsim_root, name, parentName, trackNode, 1, width, 0.0, "", NULL, 1, fov, 1.0 / width, 0.8, range * 1.2, 0, 0,NULL,0);

}

VirtualCamera::VirtualCamera(osg::Group *uwsim_root, std::string name,std::string parentName, osg::Node *trackNode, int width,
                             int height, double fov, double aspectRatio)
{ //Used in structured light projector as shadow camera
  init(uwsim_root, name, parentName, trackNode, width, height, 0.0, "", NULL, 1, fov, aspectRatio, 0.1, 20, 0, 0,NULL,0);
}

VirtualCamera::VirtualCamera(osg::Group *uwsim_root, std::string name,std::string parentName, osg::Node *trackNode, int width,
                             int height,double baseline, std::string frameId,double fov,SceneBuilder *oscene,float std,  Parameters *params=NULL, int range=0, int bw=0)
{//Standard camera / depth camera
  init(uwsim_root, name, parentName, trackNode, width, height, baseline, frameId, params, range, fov, width/(float)height, 0.18, 2000, bw, 1,oscene,std);
}

void VirtualCamera::createCamera()
{
  textureCamera = new osg::Camera;
  textureCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  textureCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  textureCamera->setViewport(0, 0, width, height);

  // Frame buffer objects are the best option
  textureCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

  // We need to render to the texture BEFORE we render to the screen
  textureCamera->setRenderOrder(osg::Camera::PRE_RENDER);

  // The camera will render into the texture that we created earlier
  if (!range)
    textureCamera->attach(osg::Camera::COLOR_BUFFER, renderTexture.get());
  else
    textureCamera->attach(osg::Camera::DEPTH_BUFFER, depthTexture.get());

  textureCamera->setName("CamViewCamera");
  textureCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

  if (!paramsOn)
  {
    //set default fov, near and far parameters (default parameters on init function as some of them change depending on camera type (shadow,depth,texture))
    textureCamera->setProjectionMatrixAsPerspective(fov, aspectRatio, near, far);
    osg::Matrixd m;
    m = textureCamera->getProjectionMatrix();
    fx = m(0, 0) * width / 2.0;
    fy = m(1, 1) * height / 2.0;
    cx = -(m(0, 2) - 1) * width / 2.0;
    cy = -(m(1, 2) - 1) * height / 2.0;
  }
  else
  {
    //set opengl projection matrix from calibration parameters fx, fy, w, h, x0, y0, n
    // How to obtain opengl projection matrix from camera calibration parameters:
    // 2.0*fx/w    2.0*k/w    1-2*x0/w       0
    // 0          2.0*fy/h   1-2*y0/h       0
    // 0           0          (f+n)/(n-f)    2*fn/(n-f)
    // 0           0         -1              0
    //osg::Matrixd m(2.0*fx/width,2.0*k/width,1-(2*cx/width),0,0,2.0*fy/height,1-(2.0*cy/height),0,0,0,(far+near)/(near-far),2*far*near/(near-far),0,0,-1,0); //osg Uses trasposed matrix
    osg::Matrixd m(2.0 * fx / width, 0, 0, 0, 2.0 * k / width, 2.0 * fy / height, 0, 0, 1 - (2 * cx / width),
                   1 - (2.0 * cy / height), (far + near) / (near - far), -1, 0, 0, 2 * far * near / (near - far), 0);
    textureCamera->setProjectionMatrix(m);

  }

  Tx = (-fx * baseline);
  Ty = 0.0;

  node_tracker = new MyNodeTrackerCallback(uwsim_root, depthTexture, textureCamera);
  trackNode->setEventCallback(node_tracker);

  //Uniforms for independence from main camera (underwater effects on shaders)
  osg::Uniform* u = new osg::Uniform("osgOcean_EyeUnderwater", true);
  u->setUpdateCallback(new UpdateUnderWater(textureCamera));

  textureCamera->getOrCreateStateSet()->addUniform(u);
  osg::Vec3d eye, center, up;
  textureCamera->getViewMatrixAsLookAt(eye, center, up);
  osg::Uniform* u2 = new osg::Uniform("osgOcean_Eye", eye);
  u2->setUpdateCallback(new UpdateEye(textureCamera));
  textureCamera->getOrCreateStateSet()->addUniform(u2);

  osg::Uniform* u3 = new osg::Uniform(osg::Uniform::FLOAT_MAT4,"osg_ViewMatrixInverse");
  u3->setUpdateCallback(new UpdateVMI(textureCamera));
  textureCamera->getOrCreateStateSet()->addUniform(u3);
}

void VirtualCamera::loadShaders(SceneBuilder *oscene)
{

  if(oscene)
  {
    static const char model_vertex[] = "default_scene.vert";
    static const char model_fragment[] = "default_scene.frag";
    osg::Program* program = osgOcean::ShaderManager::instance().createProgram("object_shader", model_vertex,model_fragment, "", "");

    textureCamera->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);

    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableGlare", oscene->scene->getOceanScene()->isGlareEnabled()) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableUnderwaterScattering", oscene->scene->getOceanScene()->isUnderwaterScatteringEnabled()) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableDOF", oscene->scene->getOceanScene()->isUnderwaterDOFEnabled()) );
 
    float UWFogDensity= oscene->scene->getOceanScene()->getUnderwaterFogDensity();
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterFogDensity", -UWFogDensity*UWFogDensity*1.442695f) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterFogColor", oscene->scene->getOceanScene()->getUnderwaterFogColor() ) );

    float AWFogDensity= oscene->scene->getOceanScene()->getAboveWaterFogDensity();
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_AboveWaterFogDensity", -AWFogDensity*AWFogDensity*1.442695f ) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_AboveWaterFogColor", oscene->scene->getOceanScene()->getAboveWaterFogColor() ) );

    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Near",  oscene->scene->getOceanScene()->getDOFNear() ) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Far",  oscene->scene->getOceanScene()->getDOFFar() ) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Focus",  oscene->scene->getOceanScene()->getDOFFocalDistance() ) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_DOF_Clamp",  oscene->scene->getOceanScene()->getDOFFarClamp() ) );

    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_WaterHeight", float(oscene->scene->getOceanScene()->getOceanSurfaceHeight()) ) );

    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterAttenuation", oscene->scene->getOceanScene()->getUnderwaterAttenuation() ) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_UnderwaterDiffuse", oscene->scene->getOceanScene()->getUnderwaterDiffuse() ) );

    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableHeightmap", false));
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableReflections", false));
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableRefractions", false));
    textureCamera->getStateSet()->addUniform( new osg::Uniform("osgOcean_EnableCrestFoam", false));
    /*textureCamera->getStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
    textureCamera->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
    textureCamera->getStateSet()->addUniform(new osg::Uniform("SLStex", 3));
    textureCamera->getStateSet()->addUniform(new osg::Uniform("SLStex2", 4));*/

    osg::Uniform* u = new osg::Uniform("offsets", osg::Vec4f(1,2,3,4));
    u->setUpdateCallback(new UpdateNoiseSeed());
    textureCamera->getStateSet()->addUniform(u);

    textureCamera->getStateSet()->addUniform( new osg::Uniform("stddev", std ) );
    textureCamera->getStateSet()->addUniform( new osg::Uniform("mean", 0.0f ) );

  }
  else
  {
    textureCamera->getOrCreateStateSet()->setAttributeAndModes(new osg::Program(), osg::StateAttribute::ON); //Unset shader
  }
}

osg::ref_ptr<osgWidget::Window> VirtualCamera::getWidgetWindow()
{
  osg::ref_ptr < osgWidget::Box > box = new osgWidget::Box("VirtualCameraBox", osgWidget::Box::HORIZONTAL, true);
  osg::ref_ptr < osgWidget::Widget > widget = new osgWidget::Widget("VirtualCameraWidget", width, height);
  if (!range)
    widget->setImage(renderTexture.get(), true, false);
  else
    widget->setImage(depthTexture.get(), true, false);
  box->addWidget(widget.get());
  box->getBackground()->setColor(1.0f, 0.0f, 0.0f, 0.8f);
  box->attachMoveCallback();
  box->attachScaleCallback();
  return box;
}

int VirtualCamera::getTFTransform(tf::Pose & pose, std::string & parent){
  parent=parentLinkName;
  pose.setOrigin(tf::Vector3(trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().x(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().y(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().z()));
  pose.setRotation( tf::Quaternion(trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().x(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().y(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().z(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().w()));
  return 1;

}
