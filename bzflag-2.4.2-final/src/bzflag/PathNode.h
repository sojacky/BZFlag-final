/*
* Created by: Weng Lam Sio
* Date: 2/29/16
*/

#ifndef	BZF_PATH_NODE_H
#define	BZF_PATH_NODE_H


// common implementation headers
#include "BZDBCache.h"

// local implementation headers
#include "World.h"

// YAGSBPL libraries
#include "yagsbpl_base.h"
#include "planners/A_star.h"

// a global variable
#define SCALINGFACTOR (BZDBCache::tankRadius)

// a node of the graph
class MyNode {
public:
	// x and y of the node
	int nodeX;
	int nodeY;

	MyNode::MyNode();
	MyNode::MyNode(const float pos[3]);
	bool accessible();

	// check to see if 2 nodes are same
	bool operator==(const MyNode& node) {
		return (nodeX == node.nodeX && nodeY == node.nodeY);
	};

};


// functions that describe the graph, placed inside a class
// redefining the virtual functions SearchGraphDescriptorFunctionContainer in yagsbpl_base.h
class GraphFunctionContainer : public SearchGraphDescriptorFunctionContainer<MyNode, double>
{
	// variables and functions
private:
	// map size
	int Xmin, Xmax, Ymin, Ymax;
	int hashTableSize;

public:
	// constructors
	GraphFunctionContainer(std::vector<int> ranges, int size);

	// function headers
	int		getHashBin(MyNode& node);
	bool	isAccessible(MyNode& node);
	void	getSuccessors(MyNode& node, std::vector<MyNode>* s, std::vector<double>* c);
	double		getHeuristics(MyNode& n1, MyNode& n2);

};

#endif