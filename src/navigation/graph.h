#ifndef __SRC_NAVIGATION_GRAPH__
#define __SRC_NAVIGATION_GRAPH__

#include <limits>

#include <node.h>


class Graph {
    private:
    std::map<std::string, Node*> Nodes;

    public:
    Graph() {
        //Do nothing;
    };

    void addNode(Node* node) {
        Nodes[node->id] = node;
    };

    void addNode(float x, float y, float theta) {
        std::string id = "" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(theta);
        Node *node = new Node();
        node->x = x;
        node->y = y;
        node->theta = theta;
        node->id = id;
        Nodes[id] = node;
    };

    Node* getNode(std::string id) {
        return Nodes[id];
    };

    Node* getClosestNode(float x, float y) {
        float leastDist = std::numeric_limits<float>::max();
        Node* result = nullptr;

        for (auto& kv : Nodes) {
            float dist = kv.second->getDistance(x, y);
            if (leastDist > dist) {
                leastDist = dist;
                result = kv.second;
            }
        }        
        return result;
    }


    // visualization::DrawPathOption(curvature_value, length, 0.0, local_viz_msg_);
    // visualization::DrawArc(center_of_turn, radius, start_angle, end_angle, 0x68ad7b, global_viz_msg_);

    void printGraph() {
        for (auto& kv : Nodes) {
            if (kv.second->isStart()) {
                printNode(kv.second);
            }
        }
    }
    
    void printNode(Node* n) {
        //print n
        //for each child
            //print path
            //printNode(child)
    }

    void printPath() {
        //Print the path with some curvature, from some start, to some end
    }
    
};
#endif //__SRC_NAVIGATION_GRAPH__