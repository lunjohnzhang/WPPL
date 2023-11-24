#pragma once

#include <vector>
#include <memory>
#include "Routing/Region.h"
#include <unordered_set>

struct Region
{
    int id;
    int y;
    int x;

    float congestion;
    
    std::vector<std::shared_ptr<Region>> neighbors;

    std::unordered_set<int> boundary;

    Region(int id, int y, int x) : id(id), y(y), x(x) {}



};