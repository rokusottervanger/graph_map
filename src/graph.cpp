#include <stdio.h>
#include <iostream>
#include <vector>

#include <geolib/datatypes.h>

#include "graph_map/graph.h"

struct graph_map::Edge
{
    geo::Transform2 pose;
    const Node* n1;
    const Node* n2;
};

struct graph_map::Node
{
    std::string id;
    std::string type;
    std::vector<Edge*> edges;
};

class graph_map::Graph
{
private:
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;

    Node* addNode(Node node)
    {
        nodes_.push_back(node);
        return &node;
    }
};

int main(int argc, char** argv)
{
    int i;
    while( i <= 10 )
    {
        i++;
        std::cout << i << std::endl;
    }
}
