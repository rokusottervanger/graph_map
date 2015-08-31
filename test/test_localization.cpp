#include <iostream>
#include <graph_map/graph.h>
//#include <geolib/datatypes.h>
//#include <geolib/Box.h>
#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include "test/World.h"


int main(int argc, char** argv)
{
    tue::Configuration config;
    World world = World();
    graph_map::Graph graph = graph_map::Graph();


    if ( argc < 1 )
    {
        std::cout << "Please provide simulator configuration file as input" << std::endl;
        return 1;
    }

    std::string yaml_filename = argv[0];
    config.loadFromYAMLFile(yaml_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Could not load configuration file:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    world.configure(config);


    // Initialize:
    //  V Load world from config
    //  V Build sim world
    //  - Build graph

    // Give initial guess for robot pose (add robot to graph)

    // Step:
    //  - Run simulator (gives object poses for now)
    //      * Generate measurements based on objects in sim world and simulated robot pose
    //  - Run graph update
    //      * Add new objects
    //      * Improve existing relations
    //      * Add new relations

    // Generate measurements based on simulator state
    // Measurement result can be object pose (?)

    std::cout << "Hello world" << std::endl;

    return 0;
}
