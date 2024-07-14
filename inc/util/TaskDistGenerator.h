#pragma once

#include "Grid.h"
#include <vector>
#include <cmath>
#include <random>

void generateTaskAndAgentDist(const Grid& map, std::mt19937 MT, vector<double>& w_dist, vector<double>& e_dist);
