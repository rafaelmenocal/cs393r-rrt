#ifndef __SRC_NAVIGATION_GRAPH__
#define __SRC_NAVIGATION_GRAPH__


#include <cstdlib>
using std::string;
using std::vector;

struct Node {  
    std::string id; // Used to find a node in the hashmap.
    float x; // x-coordinate in the map_frame
    float y; // y-coordinate in the map_frame
    float cost = -1; // Cost of taking the path through this node (Equal to the path length)
    float heuristic = 0; // This will be the distance between this node and the destination, cartesianally
    bool marked = false; // We're using this to mark visited nodes, initially
    // vector<Node*> neighbors; //Neighbors
    std::vector<std::string> neighbor_ids;
     // The current best path from some arbitrary start node to the current node. 
     // Does not include the current node.
    // vector<Node*> path;
    std::vector<std::string> path_ids;
};

#endif //__SRC_NAVIGATION_GRAPH__