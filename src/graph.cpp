#include <stdio.h>
#include <iostream>
//#include <vector>
#include <queue>
#include <limits>

#include "graph_map/graph.h"

namespace graph_map
{

Node* Graph::addNode(const Node &node)
{
    nodes_.push_back(node);
    return &nodes_.back();
}

Edge* Graph::addEdge(Node* n1, Node* n2)
{
    // Create edge object
    Edge edge(n1,n2);

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

// Todo: Maybe calculate shortest path tree when adding/updating edges/nodes with robot as root.

typedef std::pair< int, Node* > Neighbor;
const double inf = std::numeric_limits<double>::infinity();

std::vector<Node*> Graph::Dijkstra(Node* source, Node* target)
{
    Node* u;
    Node* v;
    double c, w;
    std::vector<Node*> path;

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

    // Initialize path with first node (not really necessary, because it is obvious?)
//    path.push_back(source);

    while(!Q.empty())
    {
        // Take the cheapest node from the queue
        u = Q.top().second; // node
        c = Q.top().first;  // cost so far
        Q.pop();

///        if(d[u] < c) continue;

        if ( u == target ) return path;

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
                std::cout << "Check yourself before you wreck yourself. This should never happen!" << std::endl;

            // If this node was already visited, continue
            if ( d.find(v) == d.end() )
                continue;

            // Get weight from edge
            w = e_ptr->w;

            // If path to second node is cheaper than before, update cost to that node and add it to priority queue of potential nodes to visit
            if (d[v] > d[u] + w)
            {
                d[v] = d[u] + w;
                Q.push(Neighbor(d[v], v));

                // Add visited node to path
                path.push_back(u);
            }
        }

        std::cout << "d = " << std::endl;
        for (std::map<Node*, double>::const_iterator it = d.begin(); it != d.end(); it++)
        {
            std::cout << it->first->id << ", " << it->second << ";" << std::endl;
        }
        std::cout << std::endl;

        std::cout << "path = " << std::endl;
        for (std::vector<Node*>::const_iterator it = path.begin(); it != path.end(); it++)
        {
            Node* n = *it;
            std::cout << n->id << std::endl;
        }

        // After visiting node, remove it from map of nodes with weights.
        d.erase(u);
    }
    return path;
}

// 1 function Dijkstra(Graph, source):
// 2
// 3      dist[source] ← 0                       // Distance from source to source
// 4      prev[source] ← undefined               // Previous node in optimal path initialization
// 5
// 6      create vertex set Q
// 7
// 8      for each vertex v in Graph:             // Initialization
// 9          if v ≠ source:                      // v has not yet been removed from Q (unvisited nodes)
//10              dist[v] ← INFINITY             // Unknown distance from source to v
//11              prev[v] ← UNDEFINED            // Previous node in optimal path from source
//12          add v to Q                          // All nodes initially in Q (unvisited nodes)
//13
//14      while Q is not empty:
//15          u ← vertex in Q with min dist[u]    // Source node in the first case
//16          remove u from Q
//17
//18          for each neighbor v of u:           // where v is still in Q.
//19              alt ← dist[u] + length(u, v)
//20              if alt < dist[v]:               // A shorter path to v has been found
//21                  dist[v] ← alt
//22                  prev[v] ← u
//23
//24      return dist[], prev[]


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
            os << std::endl << (*it)->n1_->id << " --> " << (*it)->n2_->id;
        }
    }
    else
        os << std::endl << "no registered edges";
    return os;
}

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
