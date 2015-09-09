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
    int n1ptr = graph.addNode(node1);
    int n2ptr = graph.addNode(node2);
    int n3ptr = graph.addNode(node3);
    int n4ptr = graph.addNode(node4);
    int n5ptr = graph.addNode(node5);
    int n6ptr = graph.addNode(node6);
    std::cout << "Nodes added!" << std::endl;

    geo::Pose3D pose1(1,0,0,0,0,0); // w = 1
    geo::Pose3D pose2(1,1,0,0,0,0); // w = 1.4
    geo::Pose3D pose3(2,1,0,0,0,0); // w = 2.24
    geo::Pose3D pose4(2,2,0,0,0,0); // w = 2.8
    geo::Pose3D pose5(4,3,0,0,0,0); // w = 5

    std::cout << "Adding edges..." << std::endl;
    graph.addEdge(n1ptr, n2ptr, pose1);
    graph.addEdge(n1ptr, n4ptr, pose1);
    graph.addEdge(n2ptr, n3ptr, pose1);
    graph.addEdge(n2ptr, n4ptr, pose1);
    graph.addEdge(n2ptr, n5ptr, pose1);
    graph.addEdge(n3ptr, n5ptr, pose1);
    graph.addEdge(n3ptr, n6ptr, pose5);
    graph.addEdge(n4ptr, n5ptr, pose1);
    graph.addEdge(n5ptr, n6ptr, pose1);
    std::cout << "Edges added!" << std::endl << std::endl;

    std::cout << "Performing Dijkstra with start node 1 and end node 6" << std::endl;
    graph_map::Path path = graph.Dijkstra(n1ptr,n6ptr);

    std::cout << path.toString() << std::endl;
    std::cout << "again:\n" << path.toString() << std::endl;
}
