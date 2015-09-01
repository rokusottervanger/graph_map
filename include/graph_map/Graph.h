#ifndef GRAPH_MAP_GRAPH_H_
#define GRAPH_MAP_GRAPH_H_

#include <vector>
#include <list>
#include <queue>
#include <geolib/datatypes.h>
#include <tue/config/configuration.h>
#include <graph_map/Measurement.h>



namespace graph_map
{

//struct UncertainTransform
//{
//    // Definition of incomplete transform
//}

// -----------------------------------------------------------------------------------------------

struct Node;

// -----------------------------------------------------------------------------------------------

struct Edge
{
//    friend class Graph;
//    friend std::ostream& operator<<(std::ostream& os, const Node& n);
    Edge(){}

    Edge(Node* n_1,Node* n_2, geo::Pose3D p) { n1 = n_1; n2 = n_2; pose = p; }

    // Only for testing, because an edge without pose but with weight makes no sense:
    Edge(Node* n_1,Node* n_2, double &weight) {n1 = n_1; n2 = n_2; w = weight;}

    inline bool operator== (Edge e) { return e.n1 == n1 && e.n2 == n2 || e.n1 == n2 && e.n2 == n1; }

    Node *n1, *n2;

    // TODO: make incompleteness of pose relationship possible
    geo::Pose3D pose; // Pose transform from node 1 to node 2

    double w; // Edge weight
};

// -----------------------------------------------------------------------------------------------

struct Node
{
//    friend class Graph;
//    friend std::ostream& operator<<(std::ostream& os, const Node& n);

    std::string type;
    std::vector<Edge*> edges;

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

    Edge* addEdge(Node* n_1, Node* n_2, geo::Pose3D &pose);

    Edge* addEdge(Edge &edge);

    // Only for testing, because an edge without pose but with weight makes no sense:
    Edge* addEdge(Node* n1, Node* n2, double weight);

    Path Dijkstra(Node *n1, Node *n2);

    void configure(tue::Configuration &config);

    Node* findNodeByID(std::string id);

    void update(Measurements);

protected:
    std::list<Node> nodes_;
    std::list<Edge> edges_;

};

}

#endif
