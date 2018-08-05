#ifndef OSG_UTILS_H 
#define OSG_UTILS_H

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Transform>

#include <iostream>
#include <vector>

namespace osg_utils 
{

typedef std::vector<osg::Node*> nodeListType; 

class findNodeVisitor : public osg::NodeVisitor { 
public: 

	findNodeVisitor(); 
	findNodeVisitor(const std::string &searchName) ;

	virtual void apply(osg::Node &searchNode);
	void setNameToFind(const std::string &searchName);
	osg::Node* getFirst(); 
	nodeListType& getNodeList() { return foundNodeList; }

private: 
	std::string searchForName; 
	nodeListType foundNodeList; 

}; 

class findRoutedNode {
public:

	findRoutedNode();

	findRoutedNode(const std::string &searchName) ;
	void setNameToFind(const std::string &searchName);
	void find(osg::ref_ptr<osg::Node> searchNode);
	osg::Node* getFirst();

private:
	findNodeVisitor nodeVisitor;	
	std::string searchRoute;
	nodeListType rootList;
};


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
	osg::Matrixd* giveUpDaMat();
private:
	bool done;
	osg::Matrixd *wcMatrix;
};

// Given a valid node placed in a scene under a transform, return the
// world coordinates in an osg::Matrix.
// Creates a visitor that will update a matrix representing world coordinates
// of the node, return this matrix.
// (This could be a class member for something derived from node also.
osg::Matrixd* getWorldCoords( osg::Node* node);

}
#endif

