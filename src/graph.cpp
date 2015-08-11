#include <stdio.h>
#include <iostream>
#include <vector>

#include <geolib/datatypes.h>

#include "graph_map/graph.h"

namespace graph_map
{

struct Edge
{
    geo::Transform2 pose;
    const Node* n1;
    const Node* n2;
};

struct Node
{
    std::string id;
    std::string type;
    std::vector<Edge*> edges;
};

void Graph::addNode(Node* node)
{
    nodes_.push_back(*node);
}

}

int main(int argc, char** argv)
{
    int i;
    while( i <= 10 )
    {
        i++;
        std::cout << i << std::endl;
    }
}
