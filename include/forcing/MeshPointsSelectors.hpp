#pragma once

#include <chrono>
#include <string>
#include <boost/variant.hpp>

struct all_points {};

struct MeshPointsSelector
{
    std::string variable_name;
    std::chrono::time_point init_time;
    std::chrono::duration duration;
    std::string output_units;
    boost::variant<all_points, std::vector<int>> points;
};
