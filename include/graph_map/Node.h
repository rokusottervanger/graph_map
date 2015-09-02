#ifndef GRAPH_MAP_NODE_H_
#define GRAPH_MAP_NODE_H_

#include <vector>
#include "Edge.h"

namespace graph_map
{

struct Node
{
    Node():id(""){}

    std::vector<Edge*> edges;

    std::string id;
};

}

#endif
