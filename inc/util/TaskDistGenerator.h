#pragma once

#include "Grid.h"
#include <vector>
#include <cmath>
#include <random>

void generateTaskAndAgentGaussianDist(const Grid& map, std::mt19937 MT, vector<double>& w_dist, vector<double>& e_dist);
void generateTaskAndAgentLRDist(const Grid& map, std::mt19937 MT, vector<double>& w_dist, vector<double>& e_dist);
