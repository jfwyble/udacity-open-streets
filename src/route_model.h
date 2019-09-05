#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iosfwd>
#include <vector>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      
        float distance(Node neighbor) const;
        void FindNeighbors();

        Node *parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0f;
        bool visited = false;
        std::vector<Node *> neighbors;

      private:
        // Add private Node variables and methods here.
        int index;
        RouteModel *parent_model = nullptr;    

        Node* FindNeighbor(std::vector<int>);
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
    std::vector<Node> &SNodes();
    std::unordered_map<int, std::vector<const Model::Road *> >& GetNodeToRoadmap();
    Node &FindClosestNode(float x, float y);
  private:
    // Add private RouteModel variables and methods here.
    std::vector<Node> m_nodes;

    std::unordered_map<int, std::vector<const Model::Road *> > node_to_road;
    void CreateNodeToRoadHashmap();

};

#endif