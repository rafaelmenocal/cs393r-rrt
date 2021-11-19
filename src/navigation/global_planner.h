#ifndef __SRC_NAVIGATION_GLOBAL_PLANNER__
#define __SRC_NAVIGATION_GLOBAL_PLANNER__

#include "graph.h"
#include "simple_queue.h"
#include <cmath>
#include <queue>
#include <string>

using std::string;

// Comparator object used in the priority_queue
// Note that if we want n1 to appear before n2, the function must return false
struct CompareNode{
    bool operator()(Node* n1, Node* n2){
        return (n1->cost + n1->heuristic > n2->cost + n2->heuristic);
    }
};

// Calculates the euclidean distance between two nodes.
float distance(Node* n1, Node* n2) {
    float dx = n1->x - n2->x;
    float dy = n1->y - n2->y;
    return sqrt((dx * dx) + (dy * dy));
}

// Prepares all nodes to run A*.
void a_star_prep(Node* goal, std::map<std::string, Node*> &Nodes) {
    for (auto it = Nodes.begin(); it != Nodes.end(); it++) {
        // Add heuristic costs to every node in the graph
        // The heuristic is inversely proportional to the distance from the goal.
        it->second->heuristic = distance(it->second, goal);
        // Reset the nodes, as well
        // When going through all the nodes, clear their "path" vectors
        it->second->path_ids.clear();
        // And set all their "visited" markers to be false
        it->second->marked = false;
        it->second->cost = -1;
    }
}

// Uses the A* algorithm to find the shortest path between two nodes,
// Using a heuristic weight for each node inversely proportional to the distance from the goal.
vector<string> a_star(Node* start, Node* goal, std::map<std::string, Node*> &Nodes) {
    std::priority_queue<Node*, vector<Node*>, CompareNode> PQ;
    a_star_prep(goal, Nodes);

    //Add start to the priorityqueue
    PQ.push(start);

    //While queue isn't empty
    while (!PQ.empty()) {
        Node *currentNode = PQ.top();
        // If first node in queue is the goal, we've found the best path!
        if (currentNode == goal) {
            return goal->path_ids;
        }

        // If the first node isn't "visited"
        if (!currentNode->marked) {
            // Note that it is possible for the node to be "visited", 
            // as there can be duplicates in the PQ if multiple paths get to the same node
            currentNode->marked = true;
            for (string n : currentNode->neighbor_ids) {
                Node* neighbor = Nodes[n];
                if (!neighbor->marked) {
                    if (neighbor->cost == -1 || currentNode->cost > currentNode->cost + 1) {
                        // If they aren't in the PQ, or if we've just found a better path, add them to the PQ
                        // Don't worry about duplicates, we will only use each node once 
                        // (when we've found the optimal path to it)
                        neighbor->cost = currentNode->cost + 1;
                        neighbor->path_ids = currentNode->path_ids;
                        neighbor->path_ids.push_back(currentNode->id);
                        PQ.push(neighbor);
                    }
                }
            }
        }
        // Pop the currentNode to remove it from the queue
        PQ.pop();
    }
    // If we get to this part of the code, the queue is empty. This means there is no path to the goal.
    return vector<string>();
}

// Returns the index in a vector of a matrix element at location (x, y)
// Doesn't matter whether we consider row-based or column-based indexing as long as we're consistent
int PointToIndex(int x, int y, int x_dim, int y_dim){
  return x * y_dim + y; 
}

// Uses the A* algorithm to find the shortest path between two nodes,
// Using a heuristic weight for each node inversely proportional to the distance from the goal.
vector<string> a_star_local_vars(Node* start, Node* goal, std::map<std::string, Node*> &Nodes, int x_dim, int y_dim) {

    std::vector<float> costs = std::vector<float>(x_dim * y_dim, 0.0);
    std::vector<std::vector<string>> paths = std::vector<std::vector<string>>(x_dim * y_dim, std::vector<string>());
    std::vector<bool> markings = std::vector<bool>(x_dim * y_dim, false);
    std::vector<float> heuristics = std::vector<float>(x_dim * y_dim, 0.0);

    for (auto it = Nodes.begin(); it != Nodes.end(); it++) {
        // Add heuristic costs to every node in the graph
        // The heuristic is inversely proportional to the distance from the goal.
        heuristics[PointToIndex(it->second->x, it->second->y, x_dim, y_dim)] = 
                                                                -1 * distance(it->second, goal);
    }

    //Create priority queue
    SimpleQueue<Node*, float> PQ;

    //Add start to the priorityqueue
    PQ.Push(start, 0);

    //While queue isn't empty
    while (!PQ.Empty()) {
        Node *currentNode = PQ.Pop(); //Access and remove
        int cnIndex = PointToIndex(currentNode->x, currentNode->y, x_dim, y_dim);
        // If first node in queue is the goal, we've found the best path!
        if (currentNode == goal) {
            return paths[cnIndex];
        }

        // If the first node isn't "visited"
        if (!markings[cnIndex]) {
            // Note that it is possible for the node to be "visited", 
            // as there can be duplicates in the PQ if multiple paths get to the same node
            markings[cnIndex] = true;
            for (string n : currentNode->neighbor_ids) {
                Node* neighbor = Nodes[n];
                int neighborIndex = PointToIndex(neighbor->x, neighbor->y, x_dim, y_dim);
                if (!markings[neighborIndex]) {
                    if (costs[neighborIndex] == -1 || costs[neighborIndex] > costs[cnIndex] + 1) {
                        // If they aren't in the PQ, or if we've just found a better path, add them to the PQ
                        // Don't worry about duplicates, we will only use each node once 
                        // (when we've found the optimal path to it)
                        //Update cost, then update the path to include the current best path
                        costs[neighborIndex] = costs[cnIndex] + 1;
                        paths[neighborIndex] = paths[cnIndex];
                        paths[neighborIndex].push_back(currentNode->id);
                        PQ.Push(neighbor, costs[neighborIndex] + heuristics[neighborIndex]);
                    }
                }
            }
        }
    }
    // If we get to this part of the code, the queue is empty. This means there is no path to the goal.
    return vector<string>();
}



#endif //__SRC_NAVIGATION_GLOBAL_PLANNER__