#include <stdio.h>
#include <iostream>
//#include <vector>
#include <queue>
#include <limits>
#include <boost/bind.hpp>

#include "graph_map/Graph.h"

namespace graph_map
{

// -----------------------------------------------------------------------------------------------

int Graph::addNode(const Node& node)
{
    int i;
    if ( deleted_nodes_.empty() )
    {
        nodes_.push_back(node);
        i = nodes_.size()-1;
    }
    else
    {
        int i = deleted_nodes_.back();
        nodes_[i] = node;
        deleted_nodes_.pop_back();
    }

    std::cout << "[GRAPH] Added node with id: '" << node.id << "'" << std::endl;

    return i;
}

// -----------------------------------------------------------------------------------------------

//int Graph::addEdge_(const Edge& edge)
//{
//    int i;
//    if ( deleted_edges_.empty() )
//    {
//        edges_.push_back(edge);
//        i = edges_.size()-1;
//    }
//    else
//    {
//        int i = deleted_edges_.back();
//        edges_[i] = edge;
//        deleted_edges_.pop_back();
//    }

//    std::cout << "[GRAPH] Added edge from: '" << nodes_[edge.n1].id << "'' to '" << nodes_[edge.n2].id << "'." << std::endl;

//    return i;
//}

// -----------------------------------------------------------------------------------------------

int Graph::addEdge(int n1, int n2, const geo::Pose3D &pose)
{
    // Create edge object
    Edge edge(n1,n2,pose);

    // Check if edge already exists (two edges are the same if they define a relation between the same two nodes)
    // Somewhere in nodes[n1].edges is an index to an edge that contains n2 as index to the second node.
    for (int i = 0; i != nodes_[n1].edges.size(); i++)
    {
        int j = nodes_[n1].edges[i];    // Index of the current edge in the graph object
        Edge e = edges_[j];             // Copy of the current edge

        if (e.n1 == n2 || e.n2 == n2)   // Check if current edge defines the same relation as the edge that is to be added
        {
            // todo: more advanced updating of edges?
            edges_[j] = edge;
            return j;
        }
    }

    // Add edge to edges vector
    int i;
    if ( deleted_edges_.empty() )
    {
        edges_.push_back(edge);
        i = edges_.size()-1;
    }
    else
    {
        int i = deleted_edges_.back();
        edges_[i] = edge;
        deleted_edges_.pop_back();
    }

    // Add edge to edges vectors of respective nodes.
    nodes_[n1].edges.push_back(i);
    nodes_[n2].edges.push_back(i);

    std::cout << "[GRAPH] Added edge from: '" << nodes_[edge.n1].id << "'' to '" << nodes_[edge.n2].id << "'." << std::endl;

    return i;
}

// -----------------------------------------------------------------------------------------------

bool Graph::configure(tue::Configuration &config)
{
    std::map<std::string,int> nodes;

    if (config.readArray("objects"))
    {
        while (config.nextArrayItem())
        {
            Node node;

            // Check for the 'enabled' field. If it exists and the value is 0, omit this object. This allows
            // the user to easily enable and disable certain objects with one single flag.
            int enabled;
            if (config.value("enabled", enabled, tue::OPTIONAL) && !enabled)
                continue;

            std::string id;
            if (!config.value("id", id))
            {
                std::cout << "\033[31m" << "[GRAPH] ERROR! Node config has no id" << "\033[0m" << std::endl;
                continue;
            }
            else
            {
                node.id = id;
                nodes[id] = addNode(node);
            }
        }
        std::cout << "1" << std::endl;
        config.endArray();
    }

    if (config.readArray("relations"))
    {
        while(config.nextArrayItem())
        {
            std::string id1, id2;
            if (!config.value("n1", id1) || !config.value("n2", id2))
                continue;

            std::map<std::string,int>::iterator n1_it = nodes.find(id1);
            std::map<std::string,int>::iterator n2_it = nodes.find(id2);

            if (n1_it != nodes.end())
            {
                int n1 = n1_it->second;
                int n2 = n2_it->second;

                geo::Pose3D pose = geo::Pose3D::identity();
                if (config.readGroup("pose", tue::REQUIRED))
                {
                    config.value("x", pose.t.x);
                    config.value("y", pose.t.y);
                    config.value("z", pose.t.z);

                    double roll = 0, pitch = 0, yaw = 0;
                    config.value("roll", roll, tue::OPTIONAL);
                    config.value("pitch", pitch, tue::OPTIONAL);
                    config.value("yaw", yaw, tue::OPTIONAL);
                    pose.R.setRPY(roll, pitch, yaw);

                    config.endGroup();
                }
                else
                {
                    std::cout << "Could not find pose group" << std::endl;
                    continue;
                }
                addEdge(n1,n2,pose);
            }
            else
            {
                std::cout << "\033[31m" << "[GRAPH] WARNING! Could not find nodes corresponding to edge" << "\033[0m" << std::endl;
            }

            std::cout << "[GRAPH] Added edge from: '" << id1 << "' to '" << id2 << "'" << std::endl;
        }
    }
    return true;
}

// -----------------------------------------------------------------------------------------------

void Graph::update(const Measurements& measurements)
{
    std::cout << "[GRAPH] Updating graph" << std::endl;
}

// -----------------------------------------------------------------------------------------------

int Graph::findNodeByID(const std::string& id)
{
    for ( int i = 0; i <= nodes_.size(); i++ )
    {
        if ( nodes_[i].id == id)
            return i;
    }
    return -1;
}

// -----------------------------------------------------------------------------------------------

// Todo: Maybe calculate shortest path tree when adding/updating edges/nodes with robot as root.

Path Graph::Dijkstra(int source, int target)
{
    typedef std::pair< int, Node* > Neighbor;
    const double inf = std::numeric_limits<double>::infinity();

    Node* u;
    Node* v;
    double c, w;
    Path path;
    std::map<Node*, Node*> prev;

    std::priority_queue<Neighbor, std::vector<Neighbor>, std::greater<Neighbor> > Q;

    // Initialize cost map with infinity and queue of unvisited nodes
//    std::map<Node*, double> d;
    std::vector<std::pair< double, int > > d; // Double is the cost, int is index of node
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
            path.push(n);
            while ( n != source )
            {
                n = prev[n];
                path.push(n);
            }
            return path;
        }

        // Run through nodes connected to cheapest node so far
        for ( std::vector<Edge*>::iterator e_it = u->edges.begin(); e_it != u->edges.end(); e_it++ )
        {
            // Retrieve the right nodes from the edges.
            Edge* e_ptr = *e_it;
            if( e_ptr->n1->id == u->id )
                v = e_ptr->n2;
            else if ( e_ptr->n2->id == u->id )
                v = e_ptr->n1;
            else
            {
                std::cout << "\033[31m" << "[GRAPH] Warning! Edge does not connect two nodes." << "\033[0m" << std::endl;
                continue;
            }

            // If this node was already visited, continue
            if ( d.find(v) == d.end() )
                continue;

            // Get weight from edge
            w = e_ptr->w;

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

}
