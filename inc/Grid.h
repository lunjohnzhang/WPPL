#pragma once

#include "common.h"

class Grid
{
public:
    Grid(string fname);

    int rows = 0;
    int cols = 0;
    std::vector<int> map;
    string map_name;

    std::vector<int> end_points;
    std::vector<int> agent_home_locations;
    std::vector<int> empty_locations;

    std::vector<char> grid_types;

};
