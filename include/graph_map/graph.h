#ifndef GRAPH_H_
#define GRAPH_H_

namespace graph_map
{

struct Edge;

struct Node;

class Graph
{
private:
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;

    void addNode(Node *node);
};

}

#endif
