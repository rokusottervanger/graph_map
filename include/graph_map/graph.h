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
    Edge(Node* n1,Node* n2, double w) { n1_ = n1; n2_ = n2; w_ = w; }
    inline bool operator== (Edge e) { return e.n1_ == n1_ && e.n2_ == n2_ || e.n1_ == n2_ && e.n2_ == n1_; }
    double w_; // Edge weight

protected:
    // TODO: make incompleteness of pose relationship possible
    geo::Transform2 pose; // Pose transform from node 1 to node 2


    Node* n1_;
    Node* n2_;
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

typedef std::vector<graph_map::Node*> Path;

// -----------------------------------------------------------------------------------------------

class Graph
{
    friend std::ostream& operator<< (std::ostream& os, const Graph& g);
public:
    Graph(){}
    ~Graph(){}

    Node* addNode(const Node &node);
    Edge* addEdge(Node* n1, Node* n2, double w);

    Path Dijkstra(Node *n1, Node *n2);

protected:
    std::list<Node> nodes_;
    std::list<Edge> edges_;

};

}

#endif
