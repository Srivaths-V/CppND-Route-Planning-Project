#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x,start_y);
    this->end_node = &m_Model.FindClosestNode(end_x,end_y);
}


// Implement the CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

   return node->distance(*end_node);
}


// Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();
    for (auto neighbor_node : current_node->neighbors) {
        neighbor_node->parent = current_node;
        neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->g_value = current_node->g_value +  current_node->distance(*neighbor_node);
        this->open_list.push_back(neighbor_node);
        neighbor_node->visited = true;
    }
    
}


// Complete the NextNode method to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {

    RouteModel::Node* node_with_lowest_sum;

    // Using lambda expression for sorting the open_list
    sort(this->open_list.begin(),this->open_list.end(), [](RouteModel::Node* neighbor_node_A, RouteModel::Node* neighbor_node_B)
    {
        return ((neighbor_node_A->g_value + neighbor_node_A->h_value) > (neighbor_node_B->g_value + neighbor_node_B->h_value));

    });

    node_with_lowest_sum = this->open_list.back();
    this->open_list.pop_back();

    return node_with_lowest_sum;

}


// Complete the ConstructFinalPath method to return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node != this->start_node){
        
        distance = distance + current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
        
    }

    path_found.push_back(*start_node);

    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// Write the A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
 
    // Make the start node as the current node
    current_node = this->start_node;
    current_node->g_value = 0;
    current_node->h_value = this->CalculateHValue(current_node);
    current_node->visited = true;
    this->open_list.emplace_back(current_node);

    while(this->open_list.size()>0){

        current_node = NextNode();
        if (current_node != this->end_node)
        {
          this->AddNeighbors(current_node);
        }
        else
        {
            m_Model.path = ConstructFinalPath(this->end_node);
        }
        
    }

}