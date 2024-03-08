#pragma once

#include "common.h"
#include "nlohmann/json.hpp"

// using json = nlohmann::json;

class Grid
{
public:
    Grid(string fname);
    Grid(nlohmann::json map_json);

    Grid() { }

    void load_map_from_path(string fname);
    void load_map_from_json(nlohmann::json map_json);

    int rows = 0;
    int cols = 0;
    std::vector<int> map;
    string map_name;

    std::vector<int> end_points;
    std::vector<int> agent_home_locations;
    std::vector<int> empty_locations;

    std::vector<char> grid_types;



};
