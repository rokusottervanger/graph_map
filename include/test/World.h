#ifndef TEST_SIM_WORLD_H_
#define TEST_SIM_WORLD_H_

#include <geolib/datatypes.h>
#include <geolib/Box.h>
#include <tue/config/configuration.h>

// -----------------------------------------------------------------------------------------------
// Ad Hoc simulator
// -----------------------------------------------------------------------------------------------

struct Object
{
    std::string id;
    geo::Shape shape;
    geo::Pose3D pose;
};

// -----------------------------------------------------------------------------------------------

class World
{
public:
    World(){}

    void addObject(Object object);

    void configure(tue::Configuration &config);

    void step();

private:
    std::vector<Object> objects_;
};

#endif
