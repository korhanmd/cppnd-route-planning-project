#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &this->m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &this->m_Model.FindClosestNode(end_x, end_y);
}


// Calculate the heuristic value of the node

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (RouteModel::Node * neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = this->CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        this->open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


// Sort the open list and return the next node.

bool Compare(RouteModel::Node *first, RouteModel::Node *second) {
    return ((first->h_value + first->g_value) > (second->h_value + second->g_value));
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), Compare);

    RouteModel::Node *next_node = this->open_list.back();
    this->open_list.pop_back();

    return next_node;
}


// Return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Iterate through parents to find the path
    path_found.push_back(*current_node);

    while (current_node->parent != nullptr) {
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
        path_found.push_back(*current_node);
    }
    
    // Reverse the path
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Start with starting node
    start_node->visited = true;
    this->AddNeighbors(start_node);

    // Iterate over the neighbors to find the path
    while (open_list.size() > 0) {
        current_node = NextNode();

        if (current_node == this->end_node) {
            this->m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        this->AddNeighbors(current_node);
    }
}