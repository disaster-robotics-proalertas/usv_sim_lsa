#include "osg_utils/osg_utils.h"

namespace osg_utils
{

// Default constructor - initialize searchForName to "" and 
// set the traversal mode to TRAVERSE_ALL_CHILDREN
findNodeVisitor::findNodeVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), 
		searchForName() 
{ 
} 

// Constructor that accepts string argument
// Initializes searchForName to user string
// set the traversal mode to TRAVERSE_ALL_CHILDREN
findNodeVisitor::findNodeVisitor(const std::string &searchName) : 
                                				   osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), 
                                				   searchForName(searchName) 
{ 
} 

//The 'apply' method for 'node' type instances.
//Compare the 'searchForName' data member against the node's name.
//If the strings match, add this node to our list
void findNodeVisitor::apply(osg::Node &searchNode) 
{ 
	//std::cerr << "Compare " << searchForName << " to "  << searchNode.getName() << std::endl;
	if (searchNode.getName() == searchForName)
	{
		foundNodeList.push_back(&searchNode);
	}
	traverse(searchNode); 
} 

// Set the searchForName to user-defined string
void findNodeVisitor::setNameToFind(const std::string &searchName) 
{ 
	searchForName = searchName; 
	foundNodeList.clear(); 
} 


osg::Node* findNodeVisitor::getFirst()
{
	if (foundNodeList.size()==0) return NULL;
	else return foundNodeList[0];
}


findRoutedNode::findRoutedNode() : nodeVisitor()
{ 
} 

findRoutedNode::findRoutedNode(const std::string &searchName) :   nodeVisitor(), searchRoute(searchName)
{ 
} 

void findRoutedNode::setNameToFind(const std::string &searchName) 
{ 
	searchRoute = searchName; 
	rootList.clear();
} 

void findRoutedNode::find(osg::ref_ptr<osg::Node> searchNode){
	unsigned int pos=0;
	rootList.clear();
	rootList.push_back(searchNode);
	nodeListType auxList,auxList2;

	while((pos=searchRoute.find("/"))<searchRoute.size()){
		for(unsigned int i=0;i<rootList.size();i++){
			nodeVisitor.setNameToFind(searchRoute.substr(0,pos));
			rootList[i]->accept(nodeVisitor);
			auxList2=nodeVisitor.getNodeList();
			auxList.insert(auxList.end(),auxList2.begin(),auxList2.end());
		}
		searchRoute.erase(0,pos+1);
		rootList=auxList;
		auxList.clear();
	}
	for(unsigned int i=0;i<rootList.size();i++){
		nodeVisitor.setNameToFind(searchRoute);
		rootList[i]->accept(nodeVisitor);
		auxList2=nodeVisitor.getNodeList();
		auxList.insert(auxList.end(),auxList2.begin(),auxList2.end());
	}
	rootList=auxList;
}

osg::Node* findRoutedNode::getFirst()
{
	if (rootList.size()==0) return NULL;
	else return rootList[0];
}



// Visitor to return the world coordinates of a node.
// It traverses from the starting node to the parent.
// The first time it reaches a root node, it stores the world coordinates of 
// the node it started from.  The world coordinates are found by concatenating all 
// the matrix transforms found on the path from the start node to the root node.
getWorldCoordOfNodeVisitor::getWorldCoordOfNodeVisitor(): osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false){
	wcMatrix=new osg::Matrixd;
}

void getWorldCoordOfNodeVisitor::apply(osg::Node &node){
	if (!done){
		if ( 0 == node.getNumParents() ){ // no parents
			wcMatrix->set( osg::computeLocalToWorld(this->getNodePath()) );
			done = true;
		}
		traverse(node);
	}
}

osg::Matrixd* getWorldCoordOfNodeVisitor::giveUpDaMat() {
	return wcMatrix;
}

// Given a valid node placed in a scene under a transform, return the
// world coordinates in an osg::Matrix.
// Creates a visitor that will update a matrix representing world coordinates
// of the node, return this matrix.
// (This could be a class member for something derived from node also.
osg::Matrixd* getWorldCoords( osg::Node* node) 
{
	getWorldCoordOfNodeVisitor* ncv = new getWorldCoordOfNodeVisitor();
	if (node && ncv)
	{
		node->accept(*ncv);
		return ncv->giveUpDaMat();
	}
	else
	{
		return NULL;
	}
} 


}
