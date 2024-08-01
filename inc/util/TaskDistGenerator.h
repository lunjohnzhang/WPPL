#pragma once

#include "Grid.h"
#include <vector>
#include <cmath>
#include <random>

void generateTaskAndAgentGaussianDist(const Grid& map, std::mt19937& MT, vector<double>& w_dist, vector<double>& e_dist);
void generateTaskAndAgentGaussianEmptyDist(const Grid& map, std::mt19937& MT, vector<double>& empty_weights);
void generateMultiModeGaussianEmptyDist(const Grid& map, std::mt19937& MT,  vector<double>& empty_weights, int K=3);
void generateTaskAndAgentLRDist(const Grid& map, std::mt19937& MT, vector<double>& w_dist, vector<double>& e_dist);
