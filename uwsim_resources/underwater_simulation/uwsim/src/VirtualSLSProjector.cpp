/*
 * VirtualStructuredLightProjector.cpp
 *
 *  Created on: 06/02/2013
 *      Author: Miquel Massot
 *	Modified by: Javier Perez
 *
 */

#include <uwsim/VirtualSLSProjector.h>

class UpdateLMVPM : public osg::Uniform::Callback
{
public:
  UpdateLMVPM(osg::Camera* camera) :
      mCamera(camera)
  {
  }
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    osg::Matrixd lmvpm = mCamera->getViewMatrix() * mCamera->getProjectionMatrix() * osg::Matrix::translate(1, 1, 1)
        * osg::Matrix::scale(0.5, 0.5, 0.5);

    u->set(lmvpm);
  }

protected:
  osg::Camera* mCamera;
};

VirtualSLSProjector::VirtualSLSProjector()
{
  osg::ref_ptr < osg::Node > node = new osg::Node;
  osg::ref_ptr < osg::Node > root = new osg::Node;
  std::string name = "SLSprojector";
  std::string image_name = "laser_texture.png";
  double range = 0; 
  double fov = 60.0;
  init(name,"base_link", root, node, image_name, range, fov, 0);
}

VirtualSLSProjector::VirtualSLSProjector(std::string name,std::string parentName, osg::Node *root, osg::Node *node, std::string image_name,
                                         double fov, bool laser)
{
  double range = 0;
  init(name, parentName, root, node, image_name, range, fov, laser);
}

void VirtualSLSProjector::init(std::string name,std::string parentName, osg::Node *root, osg::Node *node, std::string image_name, double range,
                               double fov, bool laser)
{
  this->name = name;
  this->fov = fov;
  this->range = range;
  this->node = node;
  this->image_name = image_name;
  this->textureUnit = 3; // It shouldn't be fixed

  //Create projected texture
  osg::Texture2D* texture = new osg::Texture2D();
  osg::Image* texture_to_project = osgDB::readImageFile(this->image_name);
  assert(texture_to_project);
  texture->setImage(texture_to_project);
  texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER); // It makes texture not to repeat 
  texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER); // F.M.I: http://lucera-project.blogspot.com.es/2010/06/opengl-wrap.html
  texture->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_BORDER);
  texture->setBorderColor(osg::Vec4d(0.0, 0.0, 0.0, 0.0));
  root->getOrCreateStateSet()->setTextureAttributeAndModes(4, texture, osg::StateAttribute::ON);

  //Shadow camera
  camera = VirtualCamera(root->asGroup(), name,parentName, node, texture_to_project->s(), texture_to_project->t(), fov,
                         texture_to_project->s() / (float)texture_to_project->t());

  //Create depth texture for shadow mapping test
  dbgDepthTexture = new osg::Texture2D;
  dbgDepthTexture->setTextureSize(texture_to_project->s(), texture_to_project->t()); //CHECK: Maybe we can use a smaller texture?
  dbgDepthTexture->setInternalFormat(GL_DEPTH_COMPONENT);
  dbgDepthTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
  dbgDepthTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
  root->getOrCreateStateSet()->setTextureAttributeAndModes(3, dbgDepthTexture, osg::StateAttribute::ON);
  camera.textureCamera->attach(osg::Camera::DEPTH_BUFFER, dbgDepthTexture);

  //Uniform to update texture
  osg::Matrixd lmvpm = camera.textureCamera->getViewMatrix() * camera.textureCamera->getProjectionMatrix()
      * osg::Matrix::translate(1, 1, 1) * osg::Matrix::scale(0.5, 0.5, 0.5);
  osg::Uniform* u = new osg::Uniform(osg::Uniform::FLOAT_MAT4,"LightModelViewProjectionMatrix");
  u->setUpdateCallback(new UpdateLMVPM(camera.textureCamera));
  root->getOrCreateStateSet()->addUniform(u);

  // add Laser uniform to change from laser to light behaviours
  osg::Uniform* laserUniform = new osg::Uniform("isLaser", laser);
  root->getOrCreateStateSet()->addUniform(laserUniform);

}

