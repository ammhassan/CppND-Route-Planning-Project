#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto &neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        this->open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


bool Compare(const RouteModel::Node *a, const RouteModel::Node *b){
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), Compare);
    RouteModel::Node *lowest_sum_node = open_list.back();
    this->open_list.pop_back();
    return lowest_sum_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent)
    {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = RoutePlanner::start_node;
    this->open_list.push_back(current_node);
    current_node->visited = true;
    while (this->open_list.size() > 0)
    {
        RouteModel::Node *next_node = RoutePlanner::NextNode();
        if(next_node == RoutePlanner::end_node){
            std::vector<RouteModel::Node> final_path = RoutePlanner::ConstructFinalPath(next_node);
            m_Model.path = final_path;
        }
        RoutePlanner::AddNeighbors(next_node);
    }
}