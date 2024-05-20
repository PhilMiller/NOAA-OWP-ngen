#pragma once

#include <chrono>
#include <string>
#include <boost/variant.hpp>

namespace data_access
{

enum class MeshLocation {
    UNSET = 0,
    NODE = 1,
    EDGE = 2,
    // Definition of FACE is potentially ambiguous with ELEMENT in 2D,
    // as opposed to the boundary surface between two elements in 3D
    // Also, we don't need it right now
    // FACE = 3,
    ELEMENT = 4,
};

struct all_points {};

struct MeshPointsSelector
{
    std::string variable_name;
    std::chrono::time_point init_time;
    std::chrono::duration duration;
    std::string output_units;
    boost::variant<all_points, std::vector<int>> points;
    MeshLocation location;
};

} // namespace data_access
