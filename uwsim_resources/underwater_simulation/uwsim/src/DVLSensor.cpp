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

#include <uwsim/UWSimUtils.h>
#include <uwsim/DVLSensor.h>
#include <osg/io_utils>

#include <osg/PositionAttitudeTransform>

osg::Vec3d DVLSensor::getMeasurement()
{
  //Should get world coords and then transform to the localizedWorld
  boost::shared_ptr<osg::Matrix> rMs = getWorldCoords(node_);
  osg::Matrixd lMs = *rMs * osg::Matrixd::inverse(rMl_);
  osg::Vec3d x = lMs.getTrans();

  ros::Time tnow = ros::Time::now();
  ros::Duration ellapsed_time = tnow - tprevious_;
  tprevious_ = tnow;

  //TODO: should compute the velocity at a higher rate inside an update callback?
  osg::Vec3d v = (x - xprevious_) / ellapsed_time.toSec();
  xprevious_ = x;

  //v is given wrt to the localized world. Need to rotate to the dvl frame
  osg::Matrixd sRl = osg::Matrixd::inverse(lMs);
  sRl.setTrans(0, 0, 0);
  osg::Vec4d vh(v.x(), v.y(), v.z(), 1);
  osg::Vec4d vdvl = vh * sRl;

  //Now add some gaussian noise
  static boost::normal_distribution<> normal(0, std_);
  static boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng_, normal);
  vdvl[0] += var_nor();
  vdvl[1] += var_nor();
  vdvl[2] += var_nor();

  return osg::Vec3d(vdvl.x(), vdvl.y(), vdvl.z());
}

int DVLSensor::getTFTransform(tf::Pose & pose, std::string & parent){
  parent=parentLinkName;
  pose.setOrigin(tf::Vector3(parent_->asTransform()->asPositionAttitudeTransform()->getPosition().x(),
                        parent_->asTransform()->asPositionAttitudeTransform()->getPosition().y(),
                        parent_->asTransform()->asPositionAttitudeTransform()->getPosition().z()));
  pose.setRotation( tf::Quaternion(parent_->asTransform()->asPositionAttitudeTransform()->getAttitude().x(),
                        parent_->asTransform()->asPositionAttitudeTransform()->getAttitude().y(),
                        parent_->asTransform()->asPositionAttitudeTransform()->getAttitude().z(),
                        parent_->asTransform()->asPositionAttitudeTransform()->getAttitude().w()));
  return 1;

}

