#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <list>
#include <queue>
#include <geolib/datatypes.h>

namespace graph_map
{

//struct IncompleteTransform
//{
//    // Definition of incomplete transform
//}

// -----------------------------------------------------------------------------------------------

struct Node; // Forward declare Node for use in Edge

// -----------------------------------------------------------------------------------------------

struct Edge
{
    friend class Graph;
    friend std::ostream& operator<<(std::ostream& os, const Node& n);
public:
    Edge(){}
    Edge(Node* n1,Node* n2) { n1_ = n1; n2_ = n2;}
    inline bool operator== (Edge e) { return e.n1_ == n1_ && e.n2_ == n2_ || e.n1_ == n2_ && e.n2_ == n1_; }

protected:
    // TODO: make incompleteness of pose relationship possible
    geo::Transform2 pose; // Pose transform from node 1 to node 2
    double w; // Edge weight

    const Node* n1_;
    const Node* n2_;
};

// -----------------------------------------------------------------------------------------------

struct Node
{
    friend class Graph;
    friend std::ostream& operator<<(std::ostream& os, const Node& n);

protected:
    std::string type;
    std::vector<Edge*> edges;

public:
    std::string id;
};

// -----------------------------------------------------------------------------------------------

class Graph
{
    friend std::ostream& operator<< (std::ostream& os, const Graph& g);
public:
    Graph(){}
    ~Graph(){}

    Node* addNode(const Node &node);
    Edge* addEdge(Node* n1, Node* n2);

    std::queue<Node*> Dijkstra(const Node &n1, const Node &n2);

private:
    std::list<Node> nodes_;
    std::list<Edge> edges_;

};

}

#endif
