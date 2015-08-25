#include <stdio.h>
#include <iostream>
//#include <vector>
#include <queue>

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

//struct Neighbour{
//    Node* target;
//    double weight;
//    Neighbour(Node* target_arg, double weight_arg):
//        target(target_arg), weight(weight_arg) {}
//    operator< (Neighbour n) { weight < n.weight; }
//};



//std::queue<Node*> Graph::Dijkstra(const Node &n1, const Node &n2)
//{
//    std::queue<double> d;

//    // Create vertex set Q
//    std::queue<Node*> Q;

//    for ( std::list<Node>::iterator it = nodes_.begin(); it != nodes_.end(); it++ )
//    {
////        double a = std::numeric_limits<double>::infinity();
//        Q.push(&(*it)); // Push pointer to node into queue
//    }

//    while ( !Q.empty() ) // Only visit nodes once
//    {
//        Node* u = Q.front(); // Vertex in Q with the minimum distance to the source node. Todo: sort queue
//        Q.pop();

//        // Loop through current node's neighbours
//        for ( std::vector<Edge*>::iterator e_it = u->edges.begin(); e_it != u->edges.end(); e_it++ )
//        {
//            if( (*e_it)->n1_->id == u->id )
//                const Node* v = (*e_it)->n2_;
//            else if ( (*e_it)->n2_->id == u->id )
//                const Node* v = (*e_it)->n1_;
//            else
//                std::cout << "Check yourself! This should never happen." << std::endl;
//        }
//    }
//}

typedef std::pair< int, Node* > Neighbor;

/*
Set MAX according to the number of nodes in the graph. Remember,
nodes are numbered from 1 to N. Set INF according to what is the
maximum possible shortest path length going to be in the graph.
This value should match with the default values for d[] array.
*/
const int MAX = 1024;
const int INF = 0x3f3f3f3f;

const double inf = std::numeric_limits<double>::infinity();

/*
pair object for graph is assumed to be (node, weight). d[] array
holds the shortest path from the source. It contains INF if not
reachable from the source.
*/
std::vector< Neighbor > G;
std::vector< Node* > d;

/*
The dijkstra routine. You can send a target node too along with
the source node.
*/
void dijkstra(int source, int target) {
    int u, v, i, c, w;

    priority_queue< Neighbor, vector< Neighbor >, greater< Neighbor > > Q;

    /*
    Reset the distance array and set INF as initial value. The
    source node will have weight 0. We push (0, source) in the
    priority queue as well that denotes source node has 0 weight.
    */
    memset(d, 0x3f, sizeof d);
    Q.push(pii(0, source));
    d[source] = 0;

    /*
    As long as queue is not empty, check each adjacent node of u
    */
    while(!Q.empty()) {
        u = Q.top().second; // node
        c = Q.top().first; // node cost so far
        Q.pop(); // remove the top item.

        /*
        We have discarded the visit array as we do not need it.
        If d[u] has already a better value than the currently
        popped node from queue, discard the operation on this node.
        */
        if(d[u] < c) continue;

        /*
        In case you have a target node, check if u == target node.
        If yes you can early return d[u] at this point.
        */

        /*
        Traverse the adjacent nodes of u. Remember, for the graph,,
        the pair is assumed to be (node, weight). Can be done as
        you like of course.
        */
        for(i = 0; i < G[u].size(); i++) {
            v = G[u][i].first; // node
            w = G[u][i].second; // edge weight

            /*
            Relax only if it improves the already computed shortest
            path weight.
            */
            if(d[v] > d[u] + w) {
                d[v] = d[u] + w;
                Q.push(pii(d[v], v));
            }
        }
    }
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
