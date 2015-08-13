#include <stdio.h>
#include <iostream>
#include <vector>

#include <geolib/datatypes.h>

#include "graph_map/graph.h"

namespace graph_map
{

struct Edge
{
    // TODO: make incompleteness of pose relationship possible
    geo::Transform2 pose; // Pose transform from node 1 to node 2

    const Node* n1;
    const Node* n2;
};

struct Node
{
    std::string id;
    std::string type;
    std::vector<Edge*> edges;
};

Node* Graph::addNode(Node &node)
{
    std::cout << "Node being added: " << node.id << std::endl;
    nodes_.push_back(node);
    std::cout << "Node being deleted: " << node.id << std::endl;
    delete node;
    std::cout << "Node at the end of the nodes_ vector: " << nodes_.back().id << std::endl;
    return &nodes_.back();
}

}
