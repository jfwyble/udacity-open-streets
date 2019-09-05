#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    float toPercent = .01f;
    start_x *= toPercent;
    start_y *= toPercent;
    end_x *= toPercent;
    end_y *= toPercent;
    std::cout << "Finding closest starting node..." << std::endl;
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    std::cout << "Finding closest ending node..." << std::endl;
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *parent = nullptr;

    while((parent = current_node->parent))
    {
        distance += current_node->distance(*parent);
        path_found.push_back(*current_node);
        current_node = parent;
    }
    path_found.push_back(*current_node);

    distance *= m_Model.MetricScale();
    return path_found;
}

void RoutePlanner::AStarSearch()
{
    start_node->visited = true;
    open_list.push_back(start_node);

    RouteModel::Node *current_node = nullptr;

    while(open_list.size())
    {
        std::cout << __func__ << ": Size of open list: " << open_list.size() << std::endl;
        current_node = NextNode();

        if(current_node->distance(*end_node) == 0.f)
        {
            auto path = ConstructFinalPath(current_node);
            m_Model.path = path;
            return;
        }
        else
        {
            AddNeighbors(current_node);
        }
        
    }

}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for(RouteModel::Node *neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->g_value = (current_node->g_value + current_node->distance(*neighbor));
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

float RoutePlanner::CalculateHValue(const RouteModel::Node *starting_node)
{
    return starting_node->distance(*end_node);
}

RouteModel::Node* RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node *first, RouteModel::Node *second)->bool
    {
        return first->g_value + first->h_value < second->g_value + second->h_value;
    });
    RouteModel::Node *next = open_list.front();
    open_list.erase(open_list.begin());
    return next;
}
