#ifndef __SRC_NAVIGATION_NODE__
#define __SRC_NAVIGATION_NODE__

#include <math.h>


struct Edge {
    std::string start_id; // The if of the starting node of this path
    std::string end_id; // The id of the node that results when you take this path
    float curvature; // The curvature of the path to get from start to end

    Edge(std::string sid, std::string eid, float c) {
        start_id = sid;
        end_id = eid;
        curvature = c;
    };
};

struct Node {  
    std::string id; // Used to find a node in the hashmap. Uses x, y, theta
    float x; // x-coordinate in the map_frame
    float y; // y-coordinate in the map_frame
    float theta; // Heading in degrees, in relation to the horizontal
    float cost; // Unnecessary, remove when refactoring
    std::vector<Edge> parent_paths; // The nodes you can leave to access this node
    std::vector<Edge> child_paths; // The nodes you can reach from this node

    float getDistance(float x_other, float y_other) const {
        return sqrt(pow(x_other - x, 2.0) + pow(y_other - y, 2.0));
    };

    std::string createID(float x, float y, float theta) const {
        return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(theta);
    };

    void addParent(Node* parent, float curvature) {
        parent_paths.push_back(Edge(parent->id, id, curvature));
    };

    void addChild(Node* child, float curvature) {
        child_paths.push_back(Edge(id, child->id, curvature));
    };

    bool isStart() {
        return parent_paths.empty();
    }
};

#endif //__SRC_NAVIGATION_NODE__