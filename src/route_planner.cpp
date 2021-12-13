#include "route_planner.h"
#include <algorithm>
#include <iostream>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    //Initializing the start and the end nodes.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);


}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto node:current_node->neighbors){
        if(node->visited == false){     //making sure that the node was not visited before
            node->parent = current_node;
            node->g_value = (current_node->g_value + current_node->distance(*node));
            node->h_value = CalculateHValue(node);
            open_list.push_back(node);
            node->visited = true;
        }
    }

}




RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(),open_list.end(),[](RouteModel::Node const *node_1, RouteModel::Node const *node_2){
    float f1 = node_1->h_value + node_1->g_value;
    float f2 = node_2->h_value + node_2->g_value;
    return (f1 > f2);
});
    RouteModel::Node *lowest_node = open_list.back();
    open_list.pop_back();
    return lowest_node;
}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

   //completing the path
    while(current_node != start_node){
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// The A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->visited = true;   
    open_list.push_back(start_node); //initializing the open_list with the first point
    while(open_list.size() > 0){
        current_node = NextNode();
        //check if we reached the goal
        if(current_node == end_node){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
    
    
}


