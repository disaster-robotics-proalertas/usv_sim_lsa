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

#include <uwsim/KinematicChain.h>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

KinematicChain::KinematicChain(int nlinks, int njoints)
{
  jointType.resize(njoints);
  mimic.resize(njoints);
  limits.resize(njoints);
  names.resize(njoints);
  q.resize(njoints);
  qLastSafe.resize(njoints);
  memset(&(q.front()), 0, njoints * sizeof(double));
  started = 0;
}

void KinematicChain::setJointPosition(double *newq, int n)
{
  int offset = 0;
  for (int i = 0; i < getNumberOfJoints(); i++)
  {
    if (i - offset >= n)
      break;

    if (jointType[i] == 0 || mimic[i].joint != i)
    {
      offset++;
      q[i] = 0;
    }
    else
    {
      if (newq[i - offset] < limits[i].first)
        q[i] = limits[i].first;
      else if (newq[i - offset] > limits[i].second)
        q[i] = limits[i].second;
      else
      {
        if (!isnan(q[i]))
          q[i] = newq[i - offset];
        else
        {
          std::cerr << "KinematicChain::setJointPosition received NaN" << std::endl;
          OSG_FATAL << "KinematicChain::setJointPosition received NaN" << std::endl;
        }
      }
    }
  }
  updateJoints (q);
}

void KinematicChain::setJointVelocity(double *qdot, int n)
{
  //Time issues
  double elapsed = 0;
  if (started != 0)
  {
    ros::WallDuration t_diff = ros::WallTime::now() - last;
    elapsed = t_diff.toSec();
    //If elapsed>MAX_ELAPSED, consider this is sent by a different publisher, so that the counter has to restart
    if (elapsed > 1)
      elapsed = 0;
  }

  started = 1;
  last = ros::WallTime::now();

  int offset = 0;
  for (int i = 0; i < getNumberOfJoints(); i++)
  {
    if (i - offset >= n)
      break;

    if (jointType[i] == 0 || mimic[i].joint != i)
      offset++;
    else
    {
      if (q[i] + (qdot[i - offset] * elapsed) < limits[i].first)
        q[i] = limits[i].first;
      else if (q[i] + (qdot[i - offset] * elapsed) > limits[i].second)
        q[i] = limits[i].second;
      else
        q[i] += qdot[i - offset] * elapsed;
    }
  }
  updateJoints (q);
}

void KinematicChain::setJointPosition(std::vector<double> &q, std::vector<std::string> names)
{
  if (names.size() > 0)
  {
    std::vector<double> newq;
    for (int i = 0; i < getNumberOfJoints(); i++)
    {
      if (not (jointType[i] == 0 || mimic[i].joint != i))
      { //Check it is not fixed nor mimic
        int found = 0;
        for (int j = 0; j < names.size() && !found; j++)
        {
          if (this->names[i] == names[j])
          {
            found = 1;
            newq.push_back(q[j]);
          }
        }
        if (!found)
          newq.push_back(this->q[i]);
      }
    }
    setJointPosition(&(newq.front()), newq.size());
  }
  else
    setJointPosition(&(q.front()), q.size());
}

void KinematicChain::setJointVelocity(std::vector<double> &qdot, std::vector<std::string> names)
{
  if (names.size() > 0)
  {
    std::vector<double> newq;
    for (int i = 0; i < getNumberOfJoints(); i++)
    {
      if (not (jointType[i] == 0 || mimic[i].joint != i))
      { //Check it is not fixed nor mimic
        int found = 0;
        for (int j = 0; j < names.size() && !found; j++)
        {
          if (this->names[i] == names[j])
          {
            found = 1;
            newq.push_back(qdot[j]);
          }
        }
        if (!found)
          newq.push_back(0.0);
      }
    }
    setJointVelocity(&(newq.front()), newq.size());
  }
  else
    setJointVelocity(&(qdot.front()), qdot.size());
}

std::vector<double> KinematicChain::getJointPosition()
{
  std::vector<double> validq;
  for (int i = 0; i < getNumberOfJoints(); i++)
  {
    if (jointType[i] != 0 && mimic[i].joint == i)
      validq.push_back(q[i]);
  }
  return validq;
}

std::vector<std::string> KinematicChain::getJointName()
{
  std::vector<std::string> validq;
  for(int i=0;i<getNumberOfJoints();i++)
  {
    if(jointType[i]!=0 && mimic[i].joint==i) 
      validq.push_back(names[i]);
  }
  return validq;
}

std::map<std::string, double> KinematicChain::getFullJointMap()
{
  std::map<std::string, double> map;
  for(int i=0;i<getNumberOfJoints();i++)
  {
    if (jointType[i] != 0)
      map[names[i]]=q[mimic[i].joint];

  }

  return map;
}

KinematicChain::~KinematicChain()
{
}
