//#include <stdio.h>
#include <iostream>
#include "graph_map/graph.h"

int main(int argc, char** argv)
{
    graph_map::Graph graph;

    graph_map::Node node1, node2, node3, node4;

    node1.id = "node1";
    node2.id = "node2";
    node3.id = "node3";
    node4.id = "node4";

    std::cout << "Adding nodes..." << std::endl;
    graph_map::Node* n1ptr = graph.addNode(node1);
    std::cout << "1" << std::endl;
    graph_map::Node* n2ptr = graph.addNode(node2);
    std::cout << "2" << std::endl;
    graph_map::Node* n3ptr = graph.addNode(node3);
    std::cout << "3" << std::endl;
    graph_map::Node* n4ptr = graph.addNode(node4);
    std::cout << "4" << std::endl;

    std::cout << graph << std::endl << std::endl;

    std::cout << "Adding edge from node 1 to node 2" << std::endl;
    graph_map::Edge* e = graph.addEdge(n1ptr,n2ptr);
    e->w = 1.0;
    std::cout << graph << std::endl << std::endl;

    std::cout << "Adding edge from node 2 to node 3" << std::endl;
    e = graph.addEdge(n2ptr,n3ptr);
    e->w = 1.0;
    std::cout << graph << std::endl << std::endl;

    std::cout << "Adding edge from node 3 to node 4" << std::endl;
    e = graph.addEdge(n3ptr,n4ptr);
    e->w = 1.0;
    std::cout << graph << std::endl << std::endl;

    std::cout << "Adding edge from node 1 to node 3" << std::endl;
    e = graph.addEdge(n1ptr,n3ptr);
    e->w = 1.0;
    std::cout << graph << std::endl << std::endl;

//    std::cout << "Adding edge from node 1 to node 4" << std::endl;
//    e = graph.addEdge(n1ptr,n4ptr);
//    e->w = 1;
//    std::cout << graph << std::endl << std::endl;

    graph.Dijkstra(n1ptr,n4ptr);



}
