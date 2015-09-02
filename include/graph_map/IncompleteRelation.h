#ifndef GRAPH_MAP_INCOMPLETE_RELATION_H_
#define GRAPH_MAP_INCOMPLETE_RELATION_H_

#include <geolib/datatypes.h>

namespace graph_map
{

class IncompleteRelation
{
    // Define common stuff here
    // Uncertainty?
};

class IRDistance: public IncompleteRelation
{
    double r_;
};

class IRCoordinates: public IncompleteRelation
{
    std::string frame_id;
    geo::Pose3D pose;
};

}

#endif
