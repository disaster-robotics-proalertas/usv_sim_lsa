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

#include <uwsim/MultibeamSensor.h>
#include <osg/PositionAttitudeTransform>

MultibeamSensor::MultibeamSensor(osg::Group *uwsim_root, std::string name, std::string parentName, osg::Node *trackNode, double initAngle,
                                 double finalAngle, double alpha, double range, unsigned int mask, int visible,unsigned int ARMask)
{

  //Decide number of cameras to use -> using a single camera when the fov is greater than 160 an eyefish distortion appears,...
  // fov 180 doesn't work fov>180 is a completely mess. SO we use cameras until 120 fov and merge the result.
  nCams=(int)(finalAngle-initAngle)/120.00000001+1;
  camsFOV=(finalAngle-initAngle)/nCams;
  camPixels=camsFOV / alpha + 1;
  for(int i=0;i<nCams;i++)
  {
    osg::PositionAttitudeTransform * mTc= new osg::PositionAttitudeTransform;
    mTc->setPosition(osg::Vec3d(0,0,0));
    mTc->setAttitude(osg::Quat( (initAngle+camsFOV/2  + camsFOV*i)* M_PI /180.0 , osg::Vec3d(1,0,0)));
    trackNode->asTransform()->addChild(mTc);
    vcams.push_back(VirtualCamera(uwsim_root, name,parentName, mTc, camPixels, camsFOV, range));
  }

  this->numpixels = fabs(finalAngle - initAngle) / alpha + 1;
  this->range = range;
  this->initAngle = initAngle;
  this->finalAngle = finalAngle;
  this->angleIncr = alpha;
  this->name=name;
  this->trackNode = trackNode;
  parentLinkName=parentName;
  preCalcTable();
  for(int i=0;i<nCams;i++)
  {
    vcams[i].textureCamera->setCullMask(mask);
  }

  if (visible)
  {
    osg::ref_ptr<osg::Geometry> beam = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
    osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
    for(double initAux=initAngle;initAux<=finalAngle;initAux+=angleIncr)
    {
      osg::Vec3d start(0, 0, 0);
      osg::Vec3d end(0, sin(initAux*3.14/180.0)*range, -cos(initAux*3.14/180.0)*range);
      points->push_back(start);
      points->push_back(end);
    }
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(0.0, 1.0, 0.0, 0.6));
    beam->setVertexArray(points.get());
    beam->setColorArray(color.get());
    beam->setColorBinding(osg::Geometry::BIND_OVERALL);
    beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, points->size()));
    geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    geode->addDrawable(beam.get());
    geode->setNodeMask(ARMask);
  }
  trackNode->asGroup()->addChild(geode);
}

void MultibeamSensor::preCalcTable()
{

  int iCam=0;
  remapVector.resize(numpixels);
  int current = 0;
  double lastTheta = 0;
  double thetacenter;
  osg::Vec3d first, last, center;
  osg::Matrix *MVPW;
  for (int i = 0; i < numpixels; i++)
  {
    if(i>=camPixels*iCam)
    {
      //Create matrix to unproject camera points to real world
       MVPW = new osg::Matrix(
          vcams[iCam].textureCamera->getViewMatrix() * vcams[iCam].textureCamera->getProjectionMatrix()
          * vcams[iCam].textureCamera->getViewport()->computeWindowMatrix());
      MVPW->invert(*MVPW);

      //Get first last and center points from camera
      first = osg::Vec3d(0, 0, 1) * (*MVPW) ;
      last = osg::Vec3d(0, camPixels - 1, 1) * (*MVPW);
      center = osg::Vec3d(0, camPixels / 2, 1) * (*MVPW);
      thetacenter = acos((first * center) / (center.length() * first.length())) + camsFOV*iCam*M_PI/180;
      iCam++;
    }

    //Interpolate points
    osg::Vec3d point = osg::Vec3d(0, i%camPixels, 1) * (*MVPW);
    double theta = acos(max(min( (first * point) / (first.length() * point.length()),1.0),-1.0)) + camsFOV*(iCam-1)*M_PI/180;
    while (theta >= angleIncr * current * M_PI/180 && current < numpixels)
    {
      if (theta == angleIncr * current*M_PI/180 or current==0 )
      { //usually only first iteration as point has to be exactly the same
        remapVector[current].pixel1 = i;
        remapVector[current].weight1 = 0.50;
        remapVector[current].pixel2 = i;
        remapVector[current].weight2 = 0.50;
      }
      else
      { //Interpolate between this and last point
        double dist = fabs(theta - angleIncr * current*M_PI/180 ), prevdist = fabs(lastTheta - angleIncr * current*M_PI/180 );
        remapVector[current].pixel1 = i;
        remapVector[current].weight1 = prevdist / (dist + prevdist);
        remapVector[current].pixel2 = i - 1;
        remapVector[current].weight2 = dist / (dist + prevdist);
      }
      remapVector[current].distort = 1 / cos(fabs(theta - thetacenter));
      current++;
    }
    lastTheta = theta;
  }
  osg::Vec3d point = osg::Vec3d(0, (numpixels-1)%camPixels, 1) * (*MVPW);
  double theta = acos(max(min( (first * point) / (first.length() * point.length()),1.0),-1.0)) + camsFOV*(iCam-1)*M_PI/180;

  remapVector[numpixels-1].pixel1 = numpixels-1;
  remapVector[numpixels-1].weight1 = 0.50;
  remapVector[numpixels-1].pixel2 = numpixels-1;
  remapVector[numpixels-1].weight2 = 0.50;
  remapVector[numpixels-1].distort = 1 / cos(fabs(theta - thetacenter));
}

int MultibeamSensor::getTFTransform(tf::Pose & pose, std::string & parent){
  parent=parentLinkName;
  pose.setOrigin(tf::Vector3(trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().x(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().y(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().z()));
  pose.setRotation( tf::Quaternion(trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().x(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().y(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().z(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().w()));

  tf::Pose OSGToTFconvention;
  OSGToTFconvention.setOrigin(tf::Vector3(0,0,0));
  OSGToTFconvention.setRotation(tf::Quaternion(tf::Vector3(0,1,0),M_PI/2));  //As we are using camera to simulate it, we need to rotate it
  pose=pose*OSGToTFconvention;

  return 1;

}
