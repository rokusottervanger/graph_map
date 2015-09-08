#ifndef GRAPH_MAP_NODE_H_
#define GRAPH_MAP_NODE_H_

#include <vector>
#include "Edge.h"

namespace graph_map
{

struct Node
{
    Node(){}

    Node(const std::string& id):id(id){}

    std::vector<int> edges;

    std::string id;
};

}

#endif
