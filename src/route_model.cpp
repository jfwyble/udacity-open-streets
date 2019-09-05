#include "route_model.h"
#include <iostream>
#include <algorithm>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    size_t node_count = Nodes().size();
    for(size_t idx = 0u; idx < node_count; idx++)
    {
        m_nodes.push_back(RouteModel::Node(idx, this, Nodes()[idx]));
    }

    CreateNodeToRoadHashmap();
}

std::vector<RouteModel::Node>& RouteModel::SNodes(void)
{
    return m_nodes;
}

float RouteModel::Node::distance(RouteModel::Node neighbor) const
{
    return sqrt( pow((x - neighbor.x),2) + pow((y - neighbor.y),2) );
}

void RouteModel::CreateNodeToRoadHashmap()
{
    for(const auto & road : Roads())
    {
        if(road.type != Road::Footway)
        {
            for(const int & road_idx : Ways()[road.way].nodes)
            {
                auto it = node_to_road.find(road_idx),
                    end = node_to_road.end();

                if(it == end)
                {
                    node_to_road.insert(std::make_pair(road_idx, std::vector<const Model::Road *>{}));
                }
                node_to_road[road_idx].push_back(&road);
            }
        }
    }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices)
{
    RouteModel::Node *closestNode = nullptr;
    for(const int & index : node_indices)
    {
        RouteModel::Node node = parent_model->SNodes()[index];
        if(!node.visited && distance(node) != 0)
        {
            if(!closestNode || (distance(node) < distance(*closestNode)))
                closestNode = &(parent_model->SNodes()[index]);
            
        }
    }

    return closestNode;
}

std::unordered_map<int, std::vector<const Model::Road *> >& RouteModel::GetNodeToRoadmap()
{
    return node_to_road;
}

void RouteModel::Node::FindNeighbors()
{

    for(const Road *road : this->parent_model->node_to_road[this->index])
    {
        auto *foo = FindNeighbor(parent_model->Ways()[road->way].nodes);
        if(foo)
            neighbors.push_back(foo);
    }
}

RouteModel::Node& RouteModel::FindClosestNode(float x, float y)
{
    std::cout << __func__ << std::endl;
    RouteModel::Node temp;
    temp.x = x;
    temp.y = y;

    float min_dist = std::numeric_limits<float>::max();

    int closest_idx;

    for(const auto & road : Roads())
    {
        if(road.type != Road::Footway)
        {
            for(const auto idx : Ways()[road.way].nodes)
            {
                // Find a node:
                std::cout << __func__ << ": Accessing SNodes()[" << idx << "]" << std::endl;
                std::cout << __func__ << ": SNodes().size(): " << SNodes().size() << std::endl;
                Node n = SNodes()[idx];
                
                float currentDistance = n.distance(temp);
                if(currentDistance < min_dist)
                {
                    min_dist = currentDistance;
                    closest_idx = idx;
                }
            }
        } 
    }
    std::cout << "Return from " << __func__ << std::endl;
    return SNodes()[closest_idx];
}