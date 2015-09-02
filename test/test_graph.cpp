//#include <stdio.h>
#include <iostream>
#include "graph_map/Graph.h"

int main(int argc, char** argv)
{
    graph_map::Graph graph;

    graph_map::Node node1, node2, node3, node4, node5, node6;

    node1.id = "node1";
    node2.id = "node2";
    node3.id = "node3";
    node4.id = "node4";
    node5.id = "node5";
    node6.id = "node6";

    std::cout << "Adding nodes..." << std::endl;
    graph_map::Node* n1ptr = graph.addNode(node1);
    graph_map::Node* n2ptr = graph.addNode(node2);
    graph_map::Node* n3ptr = graph.addNode(node3);
    graph_map::Node* n4ptr = graph.addNode(node4);
    graph_map::Node* n5ptr = graph.addNode(node5);
    graph_map::Node* n6ptr = graph.addNode(node6);
    std::cout << "Nodes added!" << std::endl;

    std::cout << "Adding edges..." << std::endl;
    graph.addEdge(n1ptr, n2ptr, 1.0);
    graph.addEdge(n1ptr, n4ptr, 2.0);
    graph.addEdge(n2ptr, n3ptr, 1.0);
    graph.addEdge(n3ptr, n6ptr, 10.0);
    graph.addEdge(n4ptr, n5ptr, 3.0);
    graph.addEdge(n5ptr, n6ptr, 3.0);
    std::cout << "Edges added!" << std::endl << std::endl;

    std::cout << "Performing Dijkstra with start node 1 and end node 6" << std::endl;
    graph_map::Path path = graph.Dijkstra(n1ptr,n6ptr);

    std::cout << path.toString() << std::endl;
    std::cout << "again:\n" << path.toString() << std::endl;
}
