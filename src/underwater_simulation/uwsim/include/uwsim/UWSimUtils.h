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

#ifndef UWSIMUTILS_H
#define UWSIMUTILS_H

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Timer>

#include <iostream>
#include <vector>

#include "ConfigXMLParser.h"
#include <resource_retriever/retriever.h>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osg/Version>
#include <osgText/Text>

#include <btBulletDynamicsCommon.h>

//Node data used to check if an object is catchable or not.
class NodeDataType : public osg::Referenced
{
public:
  NodeDataType(int catcha, double origP[3] = NULL, double origR[3] = NULL)
  {
    catchable = catcha;
    if (origP != NULL)
    {
      originalPosition[0] = origP[0];
      originalPosition[1] = origP[1];
      originalPosition[2] = origP[2];
    }
    if (origR != NULL)
    {
      originalRotation[0] = origR[0];
      originalRotation[1] = origR[1];
      originalRotation[2] = origR[2];
    }
    rigidBody = NULL; //initiated if physics is ON
  }
  ;
  int catchable;
  double originalPosition[3], originalRotation[3];
  btRigidBody * rigidBody;

};

typedef std::vector<osg::Node*> nodeListType;

class findNodeVisitor : public osg::NodeVisitor
{
public:

  findNodeVisitor();

  findNodeVisitor(const std::string &searchName);

  virtual void apply(osg::Node &searchNode);

  void setNameToFind(const std::string &searchName);

  osg::Node* getFirst();

  nodeListType& getNodeList()
  {
    return foundNodeList;
  }

private:

  std::string searchForName;
  nodeListType foundNodeList;

};

class findRoutedNode
{
public:

  findRoutedNode();

  findRoutedNode(const std::string &searchName);
  void setNameToFind(const std::string &searchName);
  void find(osg::ref_ptr<osg::Node> searchNode);
  osg::Node* getFirst();

private:

  findNodeVisitor nodeVisitor;
  std::string searchRoute;
  nodeListType rootList;

};

osg::Node * findRN(std::string target, osg::Group * root);

class ScopedTimer
{
public:
  ScopedTimer(const std::string& description, std::ostream& output_stream = std::cout, bool endline_after_time = true) :
      _output_stream(output_stream), _start(), _endline_after_time(endline_after_time)
  {
    _output_stream << description << std::flush;
    _start = osg::Timer::instance()->tick();
  }

  ~ScopedTimer()
  {
    osg::Timer_t end = osg::Timer::instance()->tick();
    _output_stream << osg::Timer::instance()->delta_s(_start, end) << "s";
    if (_endline_after_time)
      _output_stream << std::endl;
    else
      _output_stream << std::flush;
  }

private:
  std::ostream& _output_stream;
  osg::Timer_t _start;
  bool _endline_after_time;
};

class UWSimGeometry
{
public:
  static osg::Node* createFrame(double radius = 0.015, double length = 0.2);
  static osg::Node* createSwitchableFrame(double radius = 0.015, double length = 0.2, unsigned int mask=0x40);
  static osg::Node* createOSGBox(osg::Vec3 size);
  static osg::Node* createOSGCylinder(double radius, double height);
  static osg::Node* createOSGSphere(double radius);
  static osg::Node* createLabel(std::string text,double charSize=0.02, int bb=1, osg::Vec4 color=osg::Vec4(1,1,1,1) );

  static osg::Node * retrieveResource(std::string name);
  static osg::Node * loadGeometry(boost::shared_ptr<Geometry> geom);
private:

};

/***********/

// Visitor to return the world coordinates of a node.
// It traverses from the starting node to the parent.
// The first time it reaches a root node, it stores the world coordinates of 
// the node it started from.  The world coordinates are found by concatenating all 
// the matrix transforms found on the path from the start node to the root node.
class getWorldCoordOfNodeVisitor : public osg::NodeVisitor
{
public:
  getWorldCoordOfNodeVisitor();
  virtual void apply(osg::Node &node);
  boost::shared_ptr<osg::Matrix> giveUpDaMat();
private:
  bool done;
  boost::shared_ptr<osg::Matrix> wcMatrix;
};

// Given a valid node placed in a scene under a transform, return the
// world coordinates in an osg::Matrix.
// Creates a visitor that will update a matrix representing world coordinates
// of the node, return this matrix.
// (This could be a class member for something derived from node also.
boost::shared_ptr<osg::Matrix> getWorldCoords(osg::Node* node);

//Class to get all the catchable objects

class GetCatchableObjects : public osg::NodeVisitor
{
public:
  GetCatchableObjects();
  virtual void apply(osg::Node &node);
  nodeListType& getNodeList()
  {
    return foundNodeList;
  }
private:
  nodeListType foundNodeList;
};

//Dredging


//This is an abstract Dredge interface that must be implemented by devices to be used with a dynamicHF.
//By default Dredge Tool will be used.
class AbstractDredgeTool 
{
  public:
    // The coordinates must be in world coordinates
    virtual boost::shared_ptr<osg::Matrix> getDredgePosition() =0;
    // This function will be called each iteration with an estimation of the number of dredged particles
    virtual void dredgedParticles(int nparticles) =0;
};

class DynamicHF : public osg::Drawable::UpdateCallback
{
  public:
    DynamicHF(osg::HeightField* heightField, boost::shared_ptr<osg::Matrix> mat,  std::vector<boost::shared_ptr<AbstractDredgeTool> > tools);
    virtual void update( osg::NodeVisitor*, osg::Drawable*drawable );
  private:
    osg::HeightField* heightField;
    boost::shared_ptr<osg::Matrix> objectMat;
    std::vector<boost::shared_ptr<AbstractDredgeTool> > dredgeTools;
};

osg::Node* createHeightField(osg::ref_ptr<osg::Node> object, std::string texFile, double percent,  const std::vector<boost::shared_ptr<SimulatedIAUV> >  vehicles);
#endif

