#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>

namespace graph_map
{

struct Edge;

struct Node;

class Graph
{
private:
    Graph(){}
    ~Graph(){}

    std::vector<Node> nodes_;
    std::vector<Edge> edges_;

public:
    Node* addNode(Node &node);
};

}

#endif
