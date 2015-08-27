#include <stdio.h>
#include <iostream>
//#include <vector>
#include <queue>
#include <limits>

#include "graph_map/graph.h"

namespace graph_map
{

// -----------------------------------------------------------------------------------------------

Node* Graph::addNode(const Node &node)
{
    nodes_.push_back(node);
    return &nodes_.back();
}

// -----------------------------------------------------------------------------------------------

Edge* Graph::addEdge(Node* n1, Node* n2, double w)
{
    // Create edge object
    Edge edge(n1,n2,w);

    std::list<Edge>::iterator edge_it = std::find(edges_.begin(),edges_.end(),edge);

    // Check if edge already exists (two edges are the same if they define relations between the same two nodes)
    // Todo: Decide whether or not an edge from 1 to the 2 is the same as from 2 to 1 (but inverse, of course)
    if ( edge_it == edges_.end()) // Edge does not yet exist
    {
        // Add edge to edges vector
        edges_.push_back(edge);

        // Add edge to edges vectors of respective nodes. Todo: invert edge before adding to second node?
        n1->edges.push_back(&edges_.back());
        n2->edges.push_back(&edges_.back());

        return &edges_.back();
    }
    else // Edge already exists
    {
        // todo: update edge
        return &(*edge_it);
    }


}

// -----------------------------------------------------------------------------------------------

// Todo: Maybe calculate shortest path tree when adding/updating edges/nodes with robot as root.

typedef std::pair< int, Node* > Neighbor;
const double inf = std::numeric_limits<double>::infinity();

Path Graph::Dijkstra(Node* source, Node* target)
{
    Node* u;
    Node* v;
    double c, w;
    Path path;
    std::map<Node*, Node*> prev;

    std::priority_queue<Neighbor, std::vector<Neighbor>, std::greater<Neighbor> > Q;

    // Initialize cost map with infinity and queue of unvisited nodes
    std::map<Node*, double> d;
    for(std::list<Node>::iterator it = nodes_.begin(); it != nodes_.end(); it++)
    {
        Node* n_ptr = &(*it);
        if (n_ptr != source)
        {
            d[n_ptr] = inf;
        }
    }
    Q.push(Neighbor(0,source));
    d[source] = 0;

    while(!Q.empty())
    {
        // Take the cheapest node from the queue
        u = Q.top().second; // node
        c = Q.top().first;  // cost so far
        Q.pop();

        // If node already visited, continue
        if ( d.find(u) == d.end() )
            continue;

        // When the target is reached, trace back path and return
        if ( u == target )
        {
            Node* n = u;
            path.push_back(n);
            while ( n != source )
            {
                n = prev[n];
                path.push_back(n);
            }
            return path;
        }

        // Run through nodes connected to cheapest node so far
        for ( std::vector<Edge*>::iterator e_it = u->edges.begin(); e_it != u->edges.end(); e_it++ )
        {
            // Retrieve the right nodes from the edges.
            Edge* e_ptr = *e_it;
            if( e_ptr->n1_->id == u->id )
                v = e_ptr->n2_;
            else if ( e_ptr->n2_->id == u->id )
                v = e_ptr->n1_;
            else
            {
                std::cout << "\033[31m" << "Warning! Edge does not connect two nodes." << std::endl;
                continue;
            }

            // If this node was already visited, continue
            if ( d.find(v) == d.end() )
                continue;

            // Get weight from edge
            w = e_ptr->w_;

            /* If path to second node is cheaper than before,
            update cost to that node, add it to priority queue
            of potential nodes to visit and record from which
            node this cost came.
            */
            if (d[v] > d[u] + w)
            {
                d[v] = d[u] + w;
                Q.push(Neighbor(d[v], v));
                prev[v] = u;
            }
        }

        // After visiting node, remove it from map of nodes with weights.
        d.erase(u);
    }
}


/// ------------------------------------------------------------------------------------------------
/// Output streams ---------------------------------------------------------------------------------

// Output stream conversion for node
std::ostream& operator<<(std::ostream& os, const Node& n)
{
    os << "id: " << n.id;
    if( n.edges.size() > 0 )
    {
        os << std::endl << "edges: ";
        for( std::vector<Edge*>::const_iterator it = n.edges.begin(); it != n.edges.end(); it++ ){
            Edge* e_ptr = *it;
            os << std::endl << e_ptr->n1_->id << " --> " << e_ptr->n2_->id << " (" << e_ptr->w_ << ")";
        }
    }
    else
        os << std::endl << "no registered edges";
    return os;
}

// -----------------------------------------------------------------------------------------------

// Output stream conversion for graph
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
