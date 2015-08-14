#include <stdio.h>
#include <iostream>
//#include <vector>

#include "graph_map/graph.h"

namespace graph_map
{

Node* Graph::addNode(Node &node)
{
    nodes_.push_back(node);
    return &nodes_.back();
}

Edge* Graph::addEdge(Node* n1, Node* n2)
{
    // Create edge object
    Edge edge(n1,n2);

    // Check if edge already exists
    std::find(edges_.begin(),edges_.end(),edge);

    // Add edge to edges vector
    edges_.push_back(edge);

    // Add edge to edges vectors of respective nodes
    // TODO: Make sure that identical edges cannot be added more than once (or maybe only update the old edge?)
    n1->edges.push_back(&edges_.back());
    n2->edges.push_back(&edges_.back());
    return &edges_.back();
}


/// ------------------------------------------------------------------------------------------------
/// Output streams ---------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const Node& n)
{
    os << "id: " << n.id;
    if( n.edges.size() > 0 )
    {
        os << std::endl << "edges: ";
        for( std::vector<Edge*>::const_iterator it = n.edges.begin(); it != n.edges.end(); it++ ){
            os << std::endl << (*it)->n1_->id << " --> " << (*it)->n2_->id;
        }
    }
    else
        os << std::endl << "no registered edges";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Graph& g)
{
    if( g.nodes_.size() == 0)
        os << "Graph does not contain any nodes";
    else
    {
        os << "Graph contains these nodes:";
        for( std::list<Node>::const_iterator it = g.nodes_.begin(); it != g.nodes_.end(); it++ ){
            os << std::endl << *it;
        }
    }
    return os;
}


}
