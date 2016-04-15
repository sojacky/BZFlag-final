/*
* Created by: Weng Lam Sio
* Date: 2/29/16
*/


// interface header
#include "PathNode.h"

// MyNode default constructor
MyNode::MyNode() {
}

// MyNode constructor that takes a position
MyNode::MyNode(const float pos[3]) {
	// scaling the position
	nodeX = round(pos[0] / SCALINGFACTOR);
	nodeY = round(pos[1] / SCALINGFACTOR);
	
	// if it's accessible, do nothing
	if (accessible()) {
		return;
	}
	// else if it's not accessible, update the node to a nearby accessible node
	else {
		int x = nodeX;
		int y = nodeY;

		//s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
		for (int a = -1; a <= 1; a++)
			for (int b = -1; b <= 1; b++) {
				// if a and b is 0, it's the node itself
				if (a == 0 && b == 0) continue;

				// if a and b is not 0, that's the node's neighborhoods
				nodeX = x + a;
				nodeY = y + b;

				if (accessible() == true) {
					return;
				}
			}
	}
}

// check if the node is in the building or out of bounds
bool MyNode::accessible() {
	// convert the node back to a position in the map
	float pos[3];
	pos[0] = nodeX * SCALINGFACTOR;
	pos[1] = nodeY * SCALINGFACTOR;
	pos[2] = 0;

	// check if the positon is in the building or not
	// if it returns NULL, it is NOT in the building
	const Obstacle* ob = World::getWorld()->inBuilding(pos, BZDBCache::tankRadius / 2, BZDBCache::tankHeight);

	// Xmax and Ymax
	float worldHalf = (BZDBCache::worldSize / 2);

	// if the node is not out of bounds and is not in the building
	if ((pos[0] >= (-1 * worldHalf)) && (pos[0] <= worldHalf) && (pos[1] >= (-1 * worldHalf)) && (pos[1] <= worldHalf) && (ob == NULL)) {
		return true;
	}
	// if the node is out of bounds or in the building
	if ((pos[0] < (-1 * worldHalf)) || (pos[0] > worldHalf) || (pos[1] < (-1 * worldHalf)) || (pos[1] > worldHalf) || (ob != NULL)) {
		//getNeighborhood(node);
		return false;
	}
}

// GraphFunctionContainer constructor
GraphFunctionContainer::GraphFunctionContainer(std::vector<int> ranges, int size){
	// map size set from RobotPlayer.cxx
	Xmin = ranges[0];
	Xmax = ranges[1];
	Ymin = ranges[2];
	Ymax = ranges[3];
	hashTableSize = size;
}

// a function that returns the slot a node is stored to
int GraphFunctionContainer::getHashBin(MyNode& node)
{
	// use a hash function to determine which slot the nodes should be store to in the hash table
	int slot = abs(node.nodeX * node.nodeY) % hashTableSize;

	return slot;
}

// check to see if the node is in the building or outside the map
bool GraphFunctionContainer::isAccessible(MyNode& node)
{
	// call MyNode's accessible() function that does the same work
	return node.accessible();
}

// get the next accessible node
void GraphFunctionContainer::getSuccessors(MyNode& node, std::vector<MyNode>* s, std::vector<double>* c) // Define a 8-connected graph
{
	// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
	MyNode tn;
	s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
	for (int a = -1; a <= 1; a++)
		for (int b = -1; b <= 1; b++) {
			// if a and b is 0, it's the node itself
			if (a == 0 && b == 0) continue;
			// if a and b is not 0, that's the node's neighborhoods
			// add the node to the path
			tn.nodeX = node.nodeX + a;
			tn.nodeY = node.nodeY + b;
			s->push_back(tn);
			// add the cost if the node is added to the path
			c->push_back((int) hypotf(a, b));
		}
}

// cost function in A* search
double GraphFunctionContainer::getHeuristics(MyNode& n1, MyNode& n2)
{
	// get the Euclidean distance between 2 nodes by subtract the xs and ys individually and find the hypotenuse of it
	int dx = abs(n1.nodeX - n2.nodeX);
	int dy = abs(n1.nodeY - n2.nodeY);
	int distance = (int) hypotf(dx, dy);

	return distance; // Euclidean distance as heuristics	
}