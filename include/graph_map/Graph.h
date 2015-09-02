#ifndef GRAPH_MAP_GRAPH_H_
#define GRAPH_MAP_GRAPH_H_

#include <list>
#include <geolib/datatypes.h>
#include <tue/config/configuration.h>
#include <graph_map/Measurement.h>

#include "Edge.h"
#include "Node.h"
#include "Path.h"

namespace graph_map
{

class Graph
{
public:
    Graph(){}

    ~Graph(){}

    Node* addNode(const Node &node);

    Edge* addEdge(Node* n_1, Node* n_2, geo::Pose3D &pose);

    // Only for testing, because an edge without pose but with weight makes no sense:
    Edge* addEdge(Node* n1, Node* n2, double weight);

    Path Dijkstra(Node *n1, Node *n2);

    bool configure(tue::Configuration &config);

    Node* findNodeByID(std::string id);

    void update(Measurements);

protected:
    std::list<Node> nodes_;
    std::list<Edge> edges_;

};

}

#endif
