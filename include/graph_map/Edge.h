#ifndef GRAPH_MAP_EDGE_H_
#define GRAPH_MAP_EDGE_H_

#include <geolib/datatypes.h>

#include "IncompleteRelation.h"

namespace graph_map
{

struct Node;

// -----------------------------------------------------------------------------------------------

struct Edge
{
    Edge(Node* n_1,Node* n_2, geo::Pose3D p) { n1 = n_1; n2 = n_2; pose = p; }

    // Only for testing, because an edge without pose but with weight makes no sense:
    Edge(Node* n_1,Node* n_2, double &weight) {n1 = n_1; n2 = n_2; w = weight;}

    // Two edges are equal if they connect the same nodes
    inline bool operator== (Edge e) { return e.n1 == n1 && e.n2 == n2 || e.n1 == n2 && e.n2 == n1; }

    Node *n1, *n2;

    // TODO: make incompleteness of pose relationship possible
    geo::Pose3D pose; // Pose transform from node 1 to node 2
    IncompleteRelation ir; // Incomplete relation, not done yet.

    double w; // Edge weight
};

}

#endif
