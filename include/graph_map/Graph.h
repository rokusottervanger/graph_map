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

    int addNode(const Node&);

    int addNode(const std::string& id) { return addNode(Node(id)); }

    int addEdge(int, int, const geo::Pose3D&);

    // Only for testing, because an edge without pose but with weight makes no sense:
    Edge* addEdge(int, int, double);

    Path Dijkstra(int, int);

    bool configure(tue::Configuration&);

    int findNodeByID(const std::string&);

    void update(const Measurements&);

protected:
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
//    std::list<Node> nodes_; // TODO: Dit
//    std::list<Edge> edges_;

private:
    int addNode_(const Node&);
    int addEdge_(const Edge&);

    std::vector<int> deleted_nodes_;
    std::vector<int> deleted_edges_;

};

}

#endif
